#!/usr/bin/env python3
"""Route-based FF-PD safety planner. Never publishes /cmd_vel.

DRIVE owns this node. READY owns perception and localization.
It publishes an atomic state + speed limit policy and a rich app/log status.
All distances and rates below are initial, unvalidated tuning values.
"""
import json
import math
import os
import sys
import threading
from collections import deque

import numpy as np
import rospy
import rospkg
import tf.transformations as tft
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ff_pid_core import Route, CTE_DEADZONE_M as CONTROL_CTE_DEADZONE_M
from ff_pid_core import MAX_V_MPS as CONTROL_MAX_V_MPS

localization_scripts = os.path.join(
    rospkg.RosPack().get_path("static_livox_localization"), "scripts")
sys.path.insert(0, localization_scripts)
from body_frame import (pose_correction, reference_correction,
                        route_chair_centre, REFERENCE_CHAIR_CENTRE)
from route_assets import mask_image_path, route_content_sha256, sha256
from route_mask import RouteMask

# ======================== SAFETY TUNING ========================
# Keep all adjustable safety settings here. Unverified physical parameters
# must be measured before any occupied-wheelchair deployment.
MAX_SPEED_MPS = CONTROL_MAX_V_MPS
CTE_DEADZONE_M = CONTROL_CTE_DEADZONE_M     # Corridor expansion begins above 0.10 m
CTE_SPEED_START_M = 0.30                    # Speed cap starts falling beyond 0.30 m
CORRIDOR_HALF_WIDTH_M = 0.50               # From route centre, each side
CHAIR_HALF_WIDTH_M = 0.30                  # Approx. physical half width
CHAIR_FRONT_M = 0.52                       # Axle-centred front estimate
CHAIR_REAR_M = 0.30                        # Provisional rear extent
MASK_CLEARANCE_M = 0.30                    # Predicted/current centre to mask
NEAR_STOP_GAP_M = 0.50                     # Box-to-chair footprint gap
FRONT_STOP_M = 1.00                        # Obstacle path distance from front
FRONT_SLOW_M = 2.00
OBSTACLE_SLOW_MIN_MPS = 0.40               # Limit approached just above FRONT_STOP_M; at/below it STOP
CTE_SLOW_PER_M_MPS = 1.00                  # For each 1 cm beyond 0.30 m, reduce 0.01 m/s
CTE_SPEED_FLOOR_MPS = 0.40
CURVATURE_PREVIEW_M = 2.00                 # Ahead from current route station
CURVATURE_SPEED_FLOOR_MPS = 0.60
CURVATURE_FULL_SLOW_AT_INV_M = 0.40        # |kappa|=0.0 -> 0.8, >=0.4 -> 0.6 m/s
# Limit speed as the chair approaches the final waypoint. The controller still
# owns final stopping at GOAL_TOLERANCE_M=0.40; keep these distances in sync.
GOAL_SLOWDOWN_START_M = 3.00              # Remaining route distance for deceleration [m]
GOAL_TOLERANCE_M = 0.40                    # Matches ff_pid_node.py GOAL_TOLERANCE_M [m]
GOAL_APPROACH_SPEED_MPS = 0.40            # Retains steering margin above 0.30 m/s wheel minimum
POSE_MAX_AGE_S = 0.50
PREDICTED_MAX_AGE_S = 0.30
OBSTACLE_TIMEOUT_S = 1.00
POLICY_PERIOD_S = 0.10
STOP_CLEAR_CONFIRM_S = 0.30               # Brief stable clear before restart
SLOW_THRESHOLD_MPS = 0.005                # Avoid float noise at MAX_SPEED
MAX_OBJECTS = 200
MAX_PROFILE_SAMPLES = 500
# ===============================================================

ROUTES = ("A_to_D4", "A_to_D5", "D4_to_A",
          "D4_to_D5", "D5_to_A", "D5_to_D4")


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def box_of(item):
    try:
        size = item["size"]
        x, y, dx, dy = [float(v) for v in
                         (item["x"], item["y"], size[0], size[1])]
        if dx <= 0 or dy <= 0 or not all(math.isfinite(v) for v in (x, y, dx, dy)):
            return None
        return x, y, dx, dy
    except (KeyError, ValueError, TypeError, IndexError, OverflowError):
        return None


def footprint_gap(box):
    """Distance from detected chair-frame AABB to chair physical rectangle."""
    x, y, dx, dy = box
    gx = max(0.0, x - dx / 2 - CHAIR_FRONT_M,
             -CHAIR_REAR_M - (x + dx / 2))
    gy = max(0.0, y - dy / 2 - CHAIR_HALF_WIDTH_M,
             -CHAIR_HALF_WIDTH_M - (y + dy / 2))
    return math.hypot(gx, gy)


def object_samples(item, box):
    """Chair-frame points from bounding rectangle and optional LiDAR profile."""
    x, y, dx, dy = box
    samples = [(x + i * dx / 2, y + j * dy / 2)
               for i in (-1, 0, 1) for j in (-1, 0, 1)]
    profile = item.get("profile")
    if profile is None:
        return samples
    try:
        step = float(profile["bin_m"])
        start = float(profile["y0"])
        mins = profile["min_x"]
        if not (step > 0 and math.isfinite(step) and math.isfinite(start)
                and isinstance(mins, list) and len(mins) <= MAX_PROFILE_SAMPLES):
            return None
        for i, value in enumerate(mins):
            if value is None:
                continue
            px, py = float(value), start + (i + .5) * step
            if not math.isfinite(px) or not math.isfinite(py):
                return None
            samples.append((px, py))
        return samples
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def path_gap(route, station, chair_pose, samples, corridor_half):
    """Nearest along-route object distance, projected onto forward route segments."""
    if not samples:
        return None
    end = min(route.length, station + FRONT_SLOW_M + CHAIR_FRONT_M + 1.0)
    if end - station < 1e-6:
        return None
    stations = np.r_[station, route.s[(route.s > station) & (route.s < end)], end]
    points = np.column_stack((
        np.interp(stations, route.s, route.xy[:, 0]),
        np.interp(stations, route.s, route.xy[:, 1])))
    seg = np.diff(points, axis=0)
    sq = np.maximum(np.sum(seg * seg, axis=1), 1e-12)
    x, y, yaw = chair_pose
    c, s = math.cos(yaw), math.sin(yaw)
    best = None
    for ox, oy in samples:
        world = np.array((x + c * ox - s * oy, y + s * ox + c * oy))
        diff = world - points[:-1]
        t = np.clip(np.sum(diff * seg, axis=1) / sq, 0, 1)
        feet = points[:-1] + t[:, None] * seg
        d_sq = np.sum((feet - world) ** 2, axis=1)
        idx = int(np.argmin(d_sq))
        if d_sq[idx] > corridor_half ** 2:
            continue
        along = float(stations[idx] + t[idx] * (stations[idx + 1] - stations[idx]))
        gap = max(0.0, along - station - CHAIR_FRONT_M)
        best = gap if best is None else min(best, gap)
    return best


def speed_caps(route, station, cte):
    """CTE and top-20%-curvature speed ceilings; neither modifies steering."""
    excess = max(0.0, abs(cte) - CTE_SPEED_START_M)
    cte_cap = clamp(MAX_SPEED_MPS - CTE_SLOW_PER_M_MPS * excess,
                    CTE_SPEED_FLOOR_MPS, MAX_SPEED_MPS)
    curvature = route.top20_abs_curvature(station, CURVATURE_PREVIEW_M)
    # Linear: 0.0 1/m -> 0.8 m/s; 0.4 1/m -> 0.6 m/s (then clamped).
    curve_cap = clamp(
        MAX_SPEED_MPS - (MAX_SPEED_MPS - CURVATURE_SPEED_FLOOR_MPS)
        * curvature / CURVATURE_FULL_SLOW_AT_INV_M,
        CURVATURE_SPEED_FLOOR_MPS, MAX_SPEED_MPS)
    return cte_cap, curve_cap, curvature


def obstacle_speed_cap(distance):
    if distance is None or distance >= FRONT_SLOW_M:
        return MAX_SPEED_MPS
    if distance <= FRONT_STOP_M:
        return 0.0
    # Provisional linear profile; measured braking distance has priority.
    ratio = (distance - FRONT_STOP_M) / (FRONT_SLOW_M - FRONT_STOP_M)
    return OBSTACLE_SLOW_MIN_MPS + ratio * (MAX_SPEED_MPS - OBSTACLE_SLOW_MIN_MPS)


def goal_speed_cap(remaining_m):
    """Linear approach ceiling; final zero-speed command remains FFNode's job."""
    if remaining_m >= GOAL_SLOWDOWN_START_M:
        return MAX_SPEED_MPS
    if remaining_m <= GOAL_TOLERANCE_M:
        return GOAL_APPROACH_SPEED_MPS
    fraction = ((remaining_m - GOAL_TOLERANCE_M) /
                (GOAL_SLOWDOWN_START_M - GOAL_TOLERANCE_M))
    return GOAL_APPROACH_SPEED_MPS + fraction * (
        MAX_SPEED_MPS - GOAL_APPROACH_SPEED_MPS)


class Safety:
    def __init__(self):
        rospy.init_node("pid_safety")
        self.route_name = str(rospy.get_param("~route"))
        if self.route_name not in ROUTES:
            raise rospy.ROSInitException("Invalid route: " + self.route_name)
        self.test_clear = bool(rospy.get_param("~test_clear", False))
        if abs(CTE_DEADZONE_M - CONTROL_CTE_DEADZONE_M) > 1e-9:
            raise rospy.ROSInitException("Safety/Core CTE deadzones differ")
        if not (0 < OBSTACLE_SLOW_MIN_MPS <= MAX_SPEED_MPS
                and 0 < CTE_SPEED_FLOOR_MPS <= MAX_SPEED_MPS
                and 0 < CURVATURE_SPEED_FLOOR_MPS <= MAX_SPEED_MPS
                and CTE_SPEED_START_M >= CTE_DEADZONE_M
                and CTE_SLOW_PER_M_MPS > 0 and CURVATURE_FULL_SLOW_AT_INV_M > 0
                and FRONT_STOP_M < FRONT_SLOW_M
                and 0 < GOAL_TOLERANCE_M < GOAL_SLOWDOWN_START_M
                and CONTROL_MAX_V_MPS >= GOAL_APPROACH_SPEED_MPS > 0.30
                and 0 < CORRIDOR_HALF_WIDTH_M and 0 < POLICY_PERIOD_S):
            raise rospy.ROSInitException("Invalid safety parameters")
        repo = os.path.dirname(os.path.dirname(
            os.path.realpath(rospkg.RosPack().get_path("pid"))))
        folder = os.path.join(repo, "routes", "route2_manual_20261031")
        self.route = Route(os.path.join(folder, self.route_name + "_waypoints.json"))
        mask_path = os.path.join(folder, "common_drivable_mask.yaml")
        meta = self.route.metadata
        if meta.get("reference_point") != REFERENCE_CHAIR_CENTRE:
            raise rospy.ROSInitException("Route must be chair-centred")
        binding = meta.get("asset_binding", {})
        if (binding.get("route_content_sha256") != route_content_sha256(meta)
                or binding.get("drivable_mask_yaml_sha256") != sha256(mask_path)
                or binding.get("drivable_mask_sha256") != sha256(mask_image_path(mask_path))):
            raise rospy.ROSInitException("Route/mask integrity mismatch")
        self.mask = RouteMask(mask_path)
        self.pose_transform = pose_correction(
            str(rospy.get_param("~body_frame_profile", "builtin")),
            str(meta["body_frame_profile"])) @ reference_correction(
                str(meta["reference_point"]), route_chair_centre(meta))
        self.lock = threading.RLock()
        # Same GICP + odometry propagation as ff_pid_node.py.
        self.gicp_raw, self.gicp_odom_ref, self.gicp_received = None, None, None
        self.odom_history = deque(maxlen=100)
        self.odom_latest, self.odom_received = None, None
        self.predicted_cte, self.predicted_pose, self.predicted_at = None, None, None
        self.objects, self.objects_at = None, None
        self.station = None
        self.last_state, self.last_reason = None, None
        self.release_since = None
        self.policy_pub = rospy.Publisher("/ff_pid/motion_policy", String, queue_size=2)
        self.status_pub = rospy.Publisher("/pid_safety/status", String, queue_size=3)
        rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                         self.on_pose, queue_size=5)
        rospy.Subscriber("/Odometry", Odometry, self.on_odom, queue_size=50)
        rospy.Subscriber("/ff_pid/status", String, self.on_controller, queue_size=3)
        rospy.Subscriber("/perception/objects_summary", String,
                         self.on_obstacles, queue_size=3)
        rospy.Timer(rospy.Duration(POLICY_PERIOD_S), self.tick)
        if self.test_clear:
            rospy.logwarn("TEST_CLEAR bypasses real obstacle protection: NEVER use for occupied driving.")

    def on_odom(self, msg):
        try:
            q = msg.pose.pose.orientation
            p = msg.pose.pose.position
            transform = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
            transform[:3, 3] = (p.x, p.y, p.z)
            stamp = msg.header.stamp.to_sec()
            if stamp <= 0 or not math.isfinite(stamp):
                return
            if not np.isfinite(transform).all():
                return
            now = rospy.Time.now().to_sec()
            with self.lock:
                self.odom_history.append((stamp, transform))
                self.odom_latest = transform
                self.odom_received = now
        except Exception as exc:
            rospy.logwarn_throttle(5, "bad safety odom: %s", exc)

    def on_pose(self, msg):
        try:
            if msg.header.frame_id not in ("map", "/map"):
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            raw_map = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
            raw_map[:3, 3] = (p.x, p.y, p.z)
            now = rospy.Time.now().to_sec()
            stamp = msg.header.stamp.to_sec()
            if stamp <= 0 or abs(now - stamp) > POSE_MAX_AGE_S:
                return
            if not np.isfinite(raw_map).all():
                return
            with self.lock:
                nearest = min(self.odom_history, key=lambda e: abs(e[0] - stamp)) if self.odom_history else None
                odom_ref = nearest[1].copy() if nearest is not None and abs(nearest[0] - stamp) <= 0.10 else None
                self.gicp_raw = raw_map
                self.gicp_odom_ref = odom_ref
                self.gicp_received = now
        except Exception as exc:
            rospy.logwarn_throttle(5, "bad safety pose: %s", exc)

    def current_pose(self, now):
        if self.gicp_raw is None or self.gicp_received is None:
            return None
        if now - self.gicp_received > POSE_MAX_AGE_S:
            return None
        raw_map = self.gicp_raw
        if (self.gicp_odom_ref is not None and self.odom_latest is not None and
                self.odom_received is not None and now - self.odom_received <= 0.25):
            raw_map = raw_map @ np.linalg.inv(self.gicp_odom_ref) @ self.odom_latest
        matrix = raw_map @ self.pose_transform
        _, _, yaw = tft.euler_from_matrix(matrix)
        return float(matrix[0, 3]), float(matrix[1, 3]), float(yaw)

    def on_controller(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or data.get("state") != "RUNNING":
                with self.lock:
                    self.predicted_cte = None
                    self.predicted_pose = None
                    self.predicted_at = None
                return
            if data.get("route") not in (None, self.route_name):
                return
            cte = float(data["predicted_cte"])
            pose = tuple(float(v) for v in data["predicted_pose"])
            if len(pose) != 3 or not all(math.isfinite(v) for v in pose + (cte,)):
                return
            with self.lock:
                self.predicted_cte, self.predicted_pose = cte, pose
                self.predicted_at = rospy.Time.now().to_sec()
        except (TypeError, ValueError, KeyError, OverflowError):
            return

    def on_obstacles(self, msg):
        try:
            objects = json.loads(msg.data)
        except (TypeError, ValueError):
            objects = None
        with self.lock:
            self.objects, self.objects_at = objects, rospy.Time.now().to_sec()

    def decide(self, now, pose, predicted_pose, predicted_cte, objects, received):
        """Return raw policy with diagnostics. Any untrusted input fails closed."""
        info = {"obstacle_id": None, "obstacle_distance_m": None,
                "near_gap_m": None, "cte_used_m": None,
                "corridor_half_width_m": None, "mask_clearance_m": None,
                "current_mask_clearance_m": None, "predicted_mask_clearance_m": None,
                "cte_limit_mps": None, "curve_limit_mps": None,
                "obstacle_limit_mps": None, "preview_curvature_inv_m": None,
                "goal_remaining_m": None, "goal_limit_mps": None}
        if pose is None:
            return "STOP", 0.0, "POSE_MISSING", info
        if received is None or now - received > OBSTACLE_TIMEOUT_S:
            return "STOP", 0.0, "PERCEPTION_STALE", info
        if (not isinstance(objects, dict)
                or objects.get("status") != "OK"
                or objects.get("frame") != "chair_centre"
                or not isinstance(objects.get("objects"), list)
                or len(objects["objects"]) > MAX_OBJECTS):
            return "STOP", 0.0, "PERCEPTION_NOT_OK", info
        stamp = objects.get("stamp")
        if (not isinstance(stamp, (int, float)) or not math.isfinite(stamp)
                or stamp <= 0 or stamp > now + .2
                or now - stamp > OBSTACLE_TIMEOUT_S):
            return "STOP", 0.0, "PERCEPTION_SOURCE_STALE", info
        projected = self.route.project(pose[0], pose[1], self.station)
        station, current_cte = projected[0], projected[1]
        self.station = station
        use_cte = predicted_cte if predicted_cte is not None else current_cte
        info["cte_used_m"] = use_cte
        corridor = CORRIDOR_HALF_WIDTH_M + max(0., abs(use_cte) - CTE_DEADZONE_M)
        info["corridor_half_width_m"] = corridor
        # Before RUNNING there is no controller prediction; check the current
        # position then. Once a prediction is available, require BOTH the
        # current and predicted chair centres to have enough mask clearance.
        current_clearance = float(self.mask.clearance_at(pose[:2]))
        predicted_clearance = (float(self.mask.clearance_at(predicted_pose[:2]))
                               if predicted_pose is not None else None)
        clearances = [current_clearance]
        if predicted_clearance is not None:
            clearances.append(predicted_clearance)
        info["current_mask_clearance_m"] = (
            current_clearance if math.isfinite(current_clearance) else None)
        info["predicted_mask_clearance_m"] = (
            predicted_clearance if predicted_clearance is not None
            and math.isfinite(predicted_clearance) else None)
        if not all(math.isfinite(value) for value in clearances):
            return "STOP", 0., "MASK_BOUNDARY", info
        info["mask_clearance_m"] = min(clearances)
        if any(value < MASK_CLEARANCE_M for value in clearances):
            return "STOP", 0., "MASK_BOUNDARY", info

        cte_cap, curve_cap, curvature = speed_caps(self.route, station, use_cte)
        goal_remaining = max(0.0, self.route.length - station)
        goal_cap = goal_speed_cap(goal_remaining)
        info["goal_remaining_m"] = goal_remaining
        info["goal_limit_mps"] = goal_cap
        info["cte_limit_mps"] = cte_cap
        info["curve_limit_mps"] = curve_cap
        info["preview_curvature_inv_m"] = curvature
        closest, closest_id, near_gap = None, None, None
        for item in objects["objects"]:
            if not isinstance(item, dict):
                return "STOP", 0., "BAD_OBJECT", info
            box = box_of(item)
            if box is None:
                return "STOP", 0., "BAD_OBJECT_GEOMETRY", info
            samples = object_samples(item, box)
            if samples is None:
                return "STOP", 0., "BAD_OBJECT_PROFILE", info
            # The geometric perception 'outside' label refers to the
            # common Drivable Mask (with classification tolerance).
            # Ignore it for both near-footprint and route obstacle decisions,
            # regardless of static/moving/unknown motion classification.
            # 'crossing', 'inside' and 'unavailable' remain checked.
            if item.get("band_relation") == "outside":
                continue
            gap = footprint_gap(box)
            if near_gap is None or gap < near_gap:
                near_gap = gap
            if gap < NEAR_STOP_GAP_M:
                info.update(near_gap_m=gap, obstacle_id=item.get("id"))
                return "STOP", 0., "VERY_CLOSE_OBJECT", info
            distance = path_gap(self.route, station, pose, samples, corridor)
            if distance is not None and (closest is None or distance < closest):
                closest, closest_id = distance, item.get("id")
        info["near_gap_m"] = near_gap
        info["obstacle_distance_m"] = closest
        info["obstacle_id"] = closest_id
        obstacle_cap = obstacle_speed_cap(closest)
        info["obstacle_limit_mps"] = obstacle_cap
        if closest is not None and closest <= FRONT_STOP_M:
            return "STOP", 0., "PATH_OBSTACLE", info
        cap = min(MAX_SPEED_MPS, cte_cap, curve_cap, obstacle_cap, goal_cap)
        if cap < MAX_SPEED_MPS - SLOW_THRESHOLD_MPS:
            if cap == obstacle_cap:
                reason = "PATH_OBSTACLE"
            elif cap == cte_cap:
                reason = "CTE_LIMIT"
            elif cap == curve_cap:
                reason = "CURVATURE_LIMIT"
            else:
                reason = "GOAL_APPROACH"
            return "SLOW", cap, reason, info
        return "CLEAR", MAX_SPEED_MPS, "NO_RESTRICTION", info

    def tick(self, _event):
        now = rospy.Time.now().to_sec()
        with self.lock:
            pose = self.current_pose(now)
            fresh_pred = self.predicted_at is not None and \
                now - self.predicted_at <= PREDICTED_MAX_AGE_S
            pred_pose = self.predicted_pose if fresh_pred else None
            pred_cte = self.predicted_cte if fresh_pred else None
            objects, received = self.objects, self.objects_at
            try:
                if self.test_clear:
                    state, cap, reason = "CLEAR", MAX_SPEED_MPS, "TEST_CLEAR_BYPASS"
                    info = {"test_mode": True}
                else:
                    state, cap, reason, info = self.decide(
                        now, pose, pred_pose, pred_cte, objects, received)
                    if self.last_state == "STOP" and state != "STOP":
                        if self.release_since is None:
                            self.release_since = now
                        if now - self.release_since < STOP_CLEAR_CONFIRM_S:
                            state, cap, reason = "STOP", 0., "RELEASE_CONFIRM"
                    else:
                        self.release_since = None
            except Exception as exc:
                state, cap, reason, info = "STOP", 0., "SAFETY_EXCEPTION", {}
                self.release_since = None
                rospy.logerr_throttle(2, "Safety decision failed: %s", exc)
            # One message atomically couples policy to its speed ceiling.
            policy = {"state": state, "speed_limit_mps": cap,
                      "route": self.route_name, "stamp": now, "reason": reason}
            self.policy_pub.publish(String(data=json.dumps(policy, separators=(",", ":"))))
            status = dict(policy, **info)
            status["predicted_available"] = bool(pred_pose is not None)
            self.status_pub.publish(String(data=json.dumps(
                status, separators=(",", ":"), allow_nan=False)))
            if (state, reason) != (self.last_state, self.last_reason):
                rospy.loginfo("Safety: %s / %s / ceiling %.2f m/s", state, reason, cap)
            self.last_state, self.last_reason = state, reason


if __name__ == "__main__":
    Safety()
    rospy.spin()

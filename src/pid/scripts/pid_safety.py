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
import uuid
from collections import deque

import numpy as np
import rospy
import rospkg
import tf.transformations as tft
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int16MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ff_pid_core import Route, CTE_DEADZONE_M as CONTROL_CTE_DEADZONE_M
from ff_pid_core import MAX_V_MPS as CONTROL_MAX_V_MPS
from replan_path import checked_path, OBSTACLE_RADIUS_M, MASK_RESERVE_M
from pid_occupancy_geometry import OccupiedCells
from pid_static_window import StaticVoteWindow, STATIC_WATCH_M
from receding_motion import RecedingTracks
from obstacle_policy import (obstacle_restriction,
                             FRONT_STOP_M as POLICY_FRONT_STOP_M,
                             FRONT_SLOW_M as POLICY_FRONT_SLOW_M)

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
CTE_DEADZONE_M = CONTROL_CTE_DEADZONE_M     # Corridor expansion follows Core deadzone
CTE_SPEED_START_M = 0.30                    # Speed cap starts falling beyond 0.30 m
CORRIDOR_HALF_WIDTH_M = 0.50               # From route centre, each side
CHAIR_HALF_WIDTH_M = 0.30                  # Approx. physical half width
CHAIR_FRONT_M = 0.52                       # Axle-centred front estimate
CHAIR_REAR_M = 0.30                        # Provisional rear extent
MASK_CLEARANCE_M = 0.20                    # Predicted/current centre to mask
NEAR_STOP_GAP_M = 0.50                     # Box-to-chair footprint gap
FRONT_STOP_M = POLICY_FRONT_STOP_M         # Single source of truth: obstacle_policy.py
FRONT_SLOW_M = POLICY_FRONT_SLOW_M         # Also used by path_gap lookahead
OBSTACLE_SLOW_MIN_MPS = 0.40               # Limit approached just above FRONT_STOP_M; at/below it STOP
CTE_SLOW_PER_M_MPS = 1.00                  # For each 1 cm beyond 0.30 m, reduce 0.01 m/s
CTE_SPEED_FLOOR_MPS = 0.40
CURVATURE_PREVIEW_M = 2.00                 # Ahead from current route station
CURVATURE_SPEED_FLOOR_MPS = 0.60
CURVATURE_SLOW_START_INV_M = 0.20            # Top-20% |kappa| in next 2m >= 0.20 -> 0.6 m/s
# Two mapped speed bumps, localized from FF-PD Nov 9/10 IMU/GICP logs.
# These are map coordinates, not DWA speed-band or waypoint indices.
BUMP_WORLD_XY_M = ((88.17, -12.31), (86.53, 2.44))
BUMP_ROUTE_MATCH_MAX_M = 0.50             # Ignore bumps not traversed by the active route
BUMP_APPROACH_M = 3.00                    # Start lowering the speed ceiling this far ahead
BUMP_EXIT_M = 3.00                        # Hold low cap beyond second bump
BUMP_SPEED_CAP_MPS = 0.40                 # Provisional; physical bump safety NOT verified
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
REPLAN_TIMEOUT_S = 12.0
REPLAN_HANDSHAKE_TIMEOUT_S = 8.0
REPLAN_STAGE_RETRY_S = 1.0
REPLAN_WHEEL_STOP_MPS = 0.06
REPLAN_WHEEL_MAX_AGE_S = 0.35
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


def object_samples(item, box=None):
    """Return observed LiDAR profile samples only; never synthesize AABB points.

    Missing geometry fails closed at the caller, including learned-only detections.
    The optional box argument is ignored for compatibility with offline callers.
    """
    profile = item.get("profile")
    if profile is None:
        return None
    samples = []
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


def path_gap(route, station, chair_pose, samples, corridor_half, horizon_m=FRONT_SLOW_M):
    """Nearest along-route object distance, projected onto forward route segments."""
    if not samples:
        return None
    end = min(route.length, station + horizon_m + CHAIR_FRONT_M + 1.0)
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
    # Fixed cap when the top-20% mean of |kappa| over the next 2 m reaches 0.2 1/m.
    # No curvature SLOW for gentler path segments; obstacle/bump/CTE/goal caps still apply.
    curve_cap = (CURVATURE_SPEED_FLOOR_MPS
                 if curvature >= CURVATURE_SLOW_START_INV_M
                 else MAX_SPEED_MPS)
    return cte_cap, curve_cap, curvature


def obstacle_speed_cap(distance):
    if distance is None or distance >= FRONT_SLOW_M:
        return MAX_SPEED_MPS
    if distance <= FRONT_STOP_M:
        return 0.0
    # Provisional linear profile; measured braking distance has priority.
    ratio = (distance - FRONT_STOP_M) / (FRONT_SLOW_M - FRONT_STOP_M)
    return OBSTACLE_SLOW_MIN_MPS + ratio * (MAX_SPEED_MPS - OBSTACLE_SLOW_MIN_MPS)


def locate_route_bumps(route):
    """Return bump stations in active-route order, only for geometrically matched paths.

    Recompute on each committed A* route; never use original-route indices on a detour.
    The route is assumed to represent the travelled chair-centre reference path.
    """
    stations = []
    for x, y in BUMP_WORLD_XY_M:
        # Reuse Route.project without progress hint; this is a global map lookup.
        station, _, lateral_distance, _ = route.project(x, y, hint=None)
        if lateral_distance <= BUMP_ROUTE_MATCH_MAX_M:
            stations.append(float(station))
    if len(stations) != len(BUMP_WORLD_XY_M):
        return ()
    stations.sort()  # Forward path order, regardless of A->D5 or D5->A.
    return tuple(stations)


def bump_speed_cap(station, bump_stations):
    """0.8->0.4 m/s over 3 m approaching first bump, then hold past second bump.

    No full release until after the second bump plus BUMP_EXIT_M; other Safety
    caps and STOP decisions always have priority. There is no slowdown when
    bumps are not found on the current route.
    """
    if len(bump_stations) != 2:
        return MAX_SPEED_MPS
    first, last = bump_stations
    if station < first - BUMP_APPROACH_M or station > last + BUMP_EXIT_M:
        return MAX_SPEED_MPS
    if station >= first:
        return BUMP_SPEED_CAP_MPS
    fraction = (first - station) / BUMP_APPROACH_M
    return BUMP_SPEED_CAP_MPS + fraction * (MAX_SPEED_MPS - BUMP_SPEED_CAP_MPS)


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
        self.enable_replan = bool(rospy.get_param("~enable_replan", False)) and not self.test_clear
        self.enable_receding = bool(rospy.get_param("~enable_receding", False)) and not self.test_clear
        if abs(CTE_DEADZONE_M - CONTROL_CTE_DEADZONE_M) > 1e-9:
            raise rospy.ROSInitException("Safety/Core CTE deadzones differ")
        if not (0 < OBSTACLE_SLOW_MIN_MPS <= MAX_SPEED_MPS
                and 0 < CTE_SPEED_FLOOR_MPS <= MAX_SPEED_MPS
                and 0 < CURVATURE_SPEED_FLOOR_MPS <= MAX_SPEED_MPS
                and CTE_SPEED_START_M >= CTE_DEADZONE_M
                and CTE_SLOW_PER_M_MPS > 0 and CURVATURE_SLOW_START_INV_M > 0
                and FRONT_STOP_M < FRONT_SLOW_M
                and 0 < BUMP_SPEED_CAP_MPS <= MAX_SPEED_MPS
                and 0 < BUMP_APPROACH_M and 0 <= BUMP_EXIT_M
                and BUMP_ROUTE_MATCH_MAX_M > 0
                and 0 < GOAL_TOLERANCE_M < GOAL_SLOWDOWN_START_M
                and CONTROL_MAX_V_MPS >= GOAL_APPROACH_SPEED_MPS > 0.30
                and 0 < CORRIDOR_HALF_WIDTH_M and 0 < POLICY_PERIOD_S):
            raise rospy.ROSInitException("Invalid safety parameters")
        repo = os.path.dirname(os.path.dirname(
            os.path.realpath(rospkg.RosPack().get_path("pid"))))
        folder = os.path.join(repo, "routes", "route2_manual_20261031")
        self.route = Route(os.path.join(folder, self.route_name + "_waypoints.json"))
        self.bump_stations = locate_route_bumps(self.route)
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
        self.receding_tracks = RecedingTracks()
        self.static_votes = StaticVoteWindow()
        self.replan_clear_since = None
        self.replan_occupancy = None
        self.station = None
        self.route_version = "original"
        self.controller_ever_running = False
        self.wheel_pair = None
        self.wheel_at = None
        self.replan_mode = "IDLE"
        self.replan_id = None
        self.replan_started = None
        self.replan_stage_sent_at = None
        self.replan_candidate = None
        self.replan_obstacle_id = None
        self.replan_pose = None
        self.replan_error = None
        self.request_pub = rospy.Publisher("/ff_pid/replan/request", String, queue_size=1)
        self.stage_pub = rospy.Publisher("/ff_pid/replan/stage", String, queue_size=1)
        self.commit_pub = rospy.Publisher("/ff_pid/replan/commit", String, queue_size=1)
        self.applied_pub = rospy.Publisher("/ff_pid/active_route", String, queue_size=1, latch=True)
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
        rospy.Subscriber("/wheel_status", Int16MultiArray, self.on_wheel, queue_size=5)
        rospy.Subscriber("/ff_pid/replan/candidate", String, self.on_candidate, queue_size=1)
        rospy.Subscriber("/ff_pid/replan/ack", String, self.on_replan_ack, queue_size=5)
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
            if data.get("route_version", "original") != self.route_version:
                return
            cte = float(data["predicted_cte"])
            pose = tuple(float(v) for v in data["predicted_pose"])
            if len(pose) != 3 or not all(math.isfinite(v) for v in pose + (cte,)):
                return
            with self.lock:
                self.controller_ever_running = True
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
            self.receding_tracks.update(objects)

    def on_wheel(self, msg):
        # Read feedback, never assume that commanded zero equals physical stop.
        try:
            values = msg.data
            if len(values) < 6:
                return
            def decode(direction, magnitude):
                if int(direction) not in (67, 87, 83) or not 33 <= int(magnitude) <= 127:
                    raise ValueError("invalid wheel")
                if int(direction) == 83:
                    return 0.
                return (int(magnitude) - 33) / 36. * (1 if int(direction) == 67 else -1)
            pair = (decode(values[2], values[3]), decode(values[4], values[5]))
            with self.lock:
                self.wheel_pair = pair if int(values[1]) == 65 else None
                self.wheel_at = rospy.Time.now().to_sec()
        except (ValueError, TypeError, IndexError):
            with self.lock:
                self.wheel_pair, self.wheel_at = None, None

    def wheels_stopped(self, now):
        return (self.wheel_pair is not None and self.wheel_at is not None
                and now - self.wheel_at <= REPLAN_WHEEL_MAX_AGE_S
                and all(abs(v) <= REPLAN_WHEEL_STOP_MPS for v in self.wheel_pair))

    def cancel_replan(self):
        # A late candidate/ack from this plan_id is ignored thereafter.
        self.replan_mode = "IDLE"
        self.replan_id = None
        self.replan_candidate = None
        self.replan_obstacle_id = None
        self.replan_pose = None
        self.replan_started = None
        self.replan_error = None
        self.replan_clear_since = None
        self.replan_occupancy = None

    def begin_replan(self, now, pose, info, objects):
        if not self.controller_ever_running or not self.wheels_stopped(now):
            return False
        if not isinstance(objects, dict) or objects.get("status") != "OK":
            return False
        obstacle_id = info.get("obstacle_id")
        # Triggered by multi-observation 5 m static vote, not the latest
        # single-frame motion label. The occupancy worker requires this ID.
        confirmed_id = self.static_votes.confirmed()
        blocking = (confirmed_id is not None and obstacle_id == confirmed_id
                    and any(isinstance(obj, dict)
                            and obj.get("id") == obstacle_id
                            and obj.get("band_relation") != "outside"
                            for obj in objects.get("objects", [])))
        if not blocking:
            return False
        self.replan_id = uuid.uuid4().hex
        self.replan_mode = "PLANNING"
        self.replan_started = now
        self.replan_stage_sent_at = None
        self.replan_pose = pose
        self.replan_obstacle_id = obstacle_id
        self.replan_candidate = None
        self.request_pub.publish(String(data=json.dumps({
            "plan_id": self.replan_id, "route": self.route_name,
            "pose": list(pose), "station": float(self.station),
            "blocker_id": obstacle_id,
            "objects": objects["objects"]}, separators=(",", ":"), allow_nan=False)))
        rospy.loginfo("Safety requested static detour for obstacle %s", obstacle_id)
        return True

    def on_candidate(self, msg):
        try:
            data = json.loads(msg.data)
            with self.lock:
                if (self.replan_mode != "PLANNING" or
                        data.get("plan_id") != self.replan_id or
                        data.get("route") != self.route_name):
                    return
                if data.get("status") != "OK":
                    self.replan_error = str(data.get("error", "NO_PATH"))[:160]
                    self.replan_mode = "FAILED"
                    return
                now = rospy.Time.now().to_sec()
                pose = self.current_pose(now)
                if (pose is None or self.replan_pose is None or
                        math.hypot(pose[0] - self.replan_pose[0],
                                   pose[1] - self.replan_pose[1]) > .25 or
                        not self.wheels_stopped(now) or
                        self.objects_at is None or now - self.objects_at > OBSTACLE_TIMEOUT_S or
                        not isinstance(self.objects, dict) or
                        self.objects.get("status") != "OK" or
                        self.objects.get("frame") != "chair_centre"):
                    self.replan_mode, self.replan_error = "FAILED", "INPUT_CHANGED"
                    return
                # A worker result is untrusted. Safety rechecks the actual curve.
                meta = data["metadata"]
                if (not isinstance(meta, dict) or meta.get("route_key") != self.route_name or
                        meta.get("source") != "runtime_static_astar" or
                        meta.get("frame") != "map" or
                        meta.get("reference_point") != "chair_centre" or
                        meta.get("body_frame_profile") != self.route.metadata["body_frame_profile"] or
                        meta.get("chair_centre_in_body_xyz") != self.route.metadata["chair_centre_in_body_xyz"] or
                        len(meta.get("waypoints", [])) > 10000):
                    raise ValueError("candidate metadata rejected")
                route = Route(meta)
                if (np.linalg.norm(route.xy[0] - np.asarray(pose[:2])) > .30 or
                        np.linalg.norm(route.xy[-1] - self.route.xy[-1]) > .12):
                    raise ValueError("wrong start or final goal")
                if not any(
                        isinstance(obj, dict) and obj.get("id") == self.replan_obstacle_id
                        for obj in self.objects["objects"]):
                    self.replan_mode, self.replan_error = "FAILED", "BLOCKER_NOT_OBSERVED"
                    return
                occupied = OccupiedCells(data["occupancy"])
                if not occupied.has_id(self.replan_obstacle_id):
                    raise ValueError("planning blocker absent from occupancy")
                # Planner and Safety validate against the EXACT SAME cells.
                if not checked_path(self.mask, route.xy, pose, occupied,
                                    OBSTACLE_RADIUS_M[0], MASK_RESERVE_M):
                    raise ValueError("candidate fails current Safety collision validation")
                # Safety never changes active path before FF-PD acknowledges commit.
                self.replan_candidate = route
                self.replan_occupancy = data["occupancy"]
                self.replan_mode = "STAGING"
                self.replan_started = now
                self.replan_stage_sent_at = None
                rospy.loginfo("Safety accepted candidate, staging route %s", self.replan_id)
        except Exception as exc:
            with self.lock:
                if self.replan_mode == "PLANNING":
                    self.replan_mode, self.replan_error = "FAILED", str(exc)[:160]
            rospy.logwarn("Rejected replan candidate: %s", exc)

    def on_replan_ack(self, msg):
        try:
            ack = json.loads(msg.data)
            with self.lock:
                if (ack.get("route") != self.route_name or
                        ack.get("plan_id") != self.replan_id or
                        not ack.get("ok")):
                    return
                if self.replan_mode == "STAGING" and ack.get("phase") == "PREPARED":
                    self.replan_mode = "COMMITTING"
                    self.replan_started = rospy.Time.now().to_sec()
                elif self.replan_mode == "COMMITTING" and ack.get("phase") == "APPLIED":
                    if ack.get("route_version") != self.replan_id:
                        return
                    if self.replan_candidate is None:
                        return
                    self.route = self.replan_candidate
                    self.bump_stations = locate_route_bumps(self.route)
                    self.station = 0.0
                    self.route_version = self.replan_id
                    self.replan_mode = "ACTIVE"
                    self.replan_candidate = None
                    self.predicted_cte, self.predicted_pose, self.predicted_at = None, None, None
                    self.applied_pub.publish(String(data=json.dumps({
                        "type": "route", "route": self.route_name,
                        "route_version": self.route_version, "frame": "map",
                        "count_full": len(self.route.xy),
                        "points": [{"x": float(p[0]), "y": float(p[1])}
                                   for p in self.route.xy[::max(1, int(math.ceil(len(self.route.xy) / 400)))]]
                    }, separators=(",", ":"))))
                    rospy.loginfo("Safety/FF-PD switched to route %s", self.route_version)
        except (ValueError, TypeError, KeyError):
            return

    def update_replan(self, now, state, reason, pose, info, objects):
        if not self.enable_replan or self.route_version != "original":
            return
        if self.replan_mode in ("PLANNING", "STAGING", "FAILED"):
            # Cancel a failed/pending detour when the ORIGINAL path has no
            # blocking obstacle for 0.6s. Curvature/CTE/bump/goal SLOW is
            # normal and must not strand the wheelchair in REPLAN_HOLD.
            # STOP, stale perception, or any obstacle still in the path
            # resets this confirmation interval (no single-frame release).
            fresh = (isinstance(objects, dict) and objects.get("status") == "OK"
                     and objects.get("frame") == "chair_centre"
                     and objects.get("stamp") is not None
                     and now - float(objects["stamp"]) <= OBSTACLE_TIMEOUT_S)
            if (state in ("CLEAR", "SLOW") and fresh
                    and info.get("obstacle_distance_m") is None):
                if self.replan_clear_since is None:
                    self.replan_clear_since = now
                if now - self.replan_clear_since >= 0.6:
                    self.cancel_replan()
                    return
            else:
                self.replan_clear_since = None
        if self.replan_mode in ("PLANNING", "STAGING", "COMMITTING"):
            if (self.replan_started is None or
                    now - self.replan_started > (
                        REPLAN_TIMEOUT_S if self.replan_mode == "PLANNING"
                        else REPLAN_HANDSHAKE_TIMEOUT_S)):
                self.replan_mode, self.replan_error = "FAILED", "REPLAN_TIMEOUT"
                return
            if (self.replan_mode == "STAGING" and self.replan_candidate is not None
                    and (self.replan_stage_sent_at is None or
                         now - self.replan_stage_sent_at >= REPLAN_STAGE_RETRY_S)):
                self.stage_pub.publish(String(data=json.dumps({
                    "plan_id": self.replan_id,
                    "metadata": self.replan_candidate.metadata},
                    separators=(",", ":"), allow_nan=False)))
                self.replan_stage_sent_at = now
            elif self.replan_mode == "COMMITTING":
                self.commit_pub.publish(String(data=json.dumps({
                    "plan_id": self.replan_id}, separators=(",", ":"))))
        if (self.replan_mode == "IDLE" and state == "STOP"
                and pose is not None and self.station is not None):
            confirmed_id = self.static_votes.confirmed()
            if confirmed_id is not None:
                request_info = dict(info, obstacle_id=confirmed_id)
                self.begin_replan(now, pose, request_info, objects)

    def decide(self, now, pose, predicted_pose, predicted_cte, objects, received):
        """Return raw policy with diagnostics. Any untrusted input fails closed."""
        info = {"obstacle_id": None, "obstacle_distance_m": None,
                "near_gap_m": None, "cte_used_m": None,
                "corridor_half_width_m": None, "mask_clearance_m": None,
                "current_mask_clearance_m": None, "predicted_mask_clearance_m": None,
                "cte_limit_mps": None, "curve_limit_mps": None,
                "obstacle_limit_mps": None, "preview_curvature_inv_m": None,
                "goal_remaining_m": None, "goal_limit_mps": None,
                "bump_limit_mps": None, "bump_stations_m": None}
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
        bump_cap = bump_speed_cap(station, self.bump_stations)
        info["bump_limit_mps"] = bump_cap
        info["bump_stations_m"] = self.bump_stations
        info["goal_remaining_m"] = goal_remaining
        info["goal_limit_mps"] = goal_cap
        info["cte_limit_mps"] = cte_cap
        info["curve_limit_mps"] = curve_cap
        info["preview_curvature_inv_m"] = curvature
        closest, closest_id, near_gap = None, None, None
        early_observations = []
        early_profile_gaps = {}
        obstacle_cap = MAX_SPEED_MPS
        cap_reason = None
        stopped_distance, stopped_id = None, None
        receding_ignored = []
        receding_near = []
        for item in objects["objects"]:
            if not isinstance(item, dict):
                return "STOP", 0., "BAD_OBJECT", info
            # Preserve existing exclusion of objects entirely outside the
            # common drivable mask BEFORE requiring their collision geometry.
            if item.get("band_relation") == "outside":
                continue
            samples = object_samples(item)
            if not samples:
                # Never infer obstacle geometry from a potentially giant AABB.
                # Unknown/learned-only/malformed profiles fail closed.
                return "STOP", 0., "BAD_OBJECT_PROFILE", info
            # Occupied lateral slices, not the potentially oversized AABB,
            # determine the nearest chair footprint gap. Half a 0.2m
            # profile bin is included as a conservative lateral allowance.
            bin_half = float(item["profile"]["bin_m"]) / 2.
            near_gap_for_obj = min(
                math.hypot(max(0., sx - CHAIR_FRONT_M,
                               -CHAIR_REAR_M - sx),
                           max(0., abs(sy) - bin_half - CHAIR_HALF_WIDTH_M))
                for sx, sy in samples)
            if near_gap_for_obj is not None:
                near_gap = near_gap_for_obj if near_gap is None else min(near_gap, near_gap_for_obj)
            # Collect early 5 m path evidence independently of the normal
            # 1-2 m dynamic slowing rules.
            early_gap = path_gap(self.route, station, pose, samples,
                                 corridor + 0.10, STATIC_WATCH_M)
            if early_gap is not None and early_gap <= STATIC_WATCH_M:
                early_observations.append((item.get("id"), item.get("motion"), early_gap))
                early_profile_gaps[item.get("id")] = early_gap
            if near_gap_for_obj is not None and near_gap_for_obj < NEAR_STOP_GAP_M:
                info.update(near_gap_m=near_gap_for_obj, obstacle_id=item.get("id"))
                self.static_votes.add(stamp, early_observations)
                return "STOP", 0., "VERY_CLOSE_OBJECT", info
            distance = path_gap(self.route, station, pose, samples, corridor)
            if distance is None:
                continue
            if closest is None or distance < closest:
                closest, closest_id = distance, item.get("id")
            moving_away = (item.get("motion") == "moving"
                           and self.receding_tracks.receding(item.get("id"), stamp, now=now))
            item_cap, item_stop, item_reason = obstacle_restriction(
                distance, item.get("motion"), moving_away,
                MAX_SPEED_MPS, OBSTACLE_SLOW_MIN_MPS,
                static_detour_enabled=self.enable_replan,  # static STOP from 2.5m with replan
                receding_enabled=self.enable_receding)
            if item_reason == "RECEDING_IGNORED":
                receding_ignored.append(item.get("id"))
            elif item_reason == "RECEDING_NEAR_SLOW":
                receding_near.append(item.get("id"))
            if item_stop and (stopped_distance is None or distance < stopped_distance):
                stopped_distance, stopped_id = distance, item.get("id")
            if item_cap < obstacle_cap:
                obstacle_cap, cap_reason = item_cap, item_reason
        if self.enable_replan:
            self.static_votes.add(stamp, early_observations)
        info["static_candidate_id"] = self.static_votes.confirmed() if self.enable_replan else None
        info["near_gap_m"] = near_gap
        info["obstacle_distance_m"] = closest
        info["obstacle_id"] = stopped_id if stopped_id is not None else closest_id
        info["obstacle_limit_mps"] = obstacle_cap
        info["receding_ignored_ids"] = receding_ignored
        info["receding_near_slow_ids"] = receding_near
        if stopped_distance is not None:
            return "STOP", 0., "PATH_OBSTACLE", info
        if self.enable_replan and info["static_candidate_id"] in early_profile_gaps:
            info["obstacle_id"] = info["static_candidate_id"]
            info["obstacle_distance_m"] = early_profile_gaps[info["static_candidate_id"]]
            return "STOP", 0., "STATIC_REPLAN_EARLY", info
        cap = min(MAX_SPEED_MPS, cte_cap, curve_cap, obstacle_cap, goal_cap, bump_cap)
        if cap < MAX_SPEED_MPS - SLOW_THRESHOLD_MPS:
            if cap == obstacle_cap:
                reason = "PATH_OBSTACLE"
            elif cap == cte_cap:
                reason = "CTE_LIMIT"
            elif cap == bump_cap:
                reason = "BUMP_LIMIT"
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
            # During a path transaction, motion is held regardless of old-path
            # clearance; only an APPLIED acknowledgement releases the hold.
            if not self.test_clear:
                try:
                    self.update_replan(now, state, reason, pose, info, objects)
                except Exception as exc:
                    self.replan_mode = "FAILED"
                    self.replan_error = "REPLAN_EXCEPTION"
                    rospy.logerr_throttle(2, "Replanning state transition failed: %s", exc)
                if self.replan_mode in ("PLANNING", "STAGING", "COMMITTING", "FAILED"):
                    state, cap, reason = "STOP", 0., "REPLAN_HOLD"
            # One message atomically couples policy to its speed ceiling.
            policy = {"state": state, "speed_limit_mps": cap,
                      "route": self.route_name, "route_version": self.route_version,
                      "stamp": now, "reason": reason}
            self.policy_pub.publish(String(data=json.dumps(policy, separators=(",", ":"))))
            status = dict(policy, **info)
            status["replan_state"] = self.replan_mode
            status["replan_error"] = self.replan_error
            status["predicted_available"] = bool(pred_pose is not None)
            self.status_pub.publish(String(data=json.dumps(
                status, separators=(",", ":"), allow_nan=False)))
            if (state, reason) != (self.last_state, self.last_reason):
                rospy.loginfo("Safety: %s / %s / ceiling %.2f m/s", state, reason, cap)
            self.last_state, self.last_reason = state, reason


if __name__ == "__main__":
    Safety()
    rospy.spin()

#!/usr/bin/env python3
"""FF-PD safety: mask boundary, nearby obstacles and obstacles along the route.

This node only publishes CLEAR/SLOW/STOP. It does not steer or create bypass paths.
"""
import json
import math
import os
import sys
import threading

import numpy as np
import rospy
import rospkg
import tf.transformations as tft
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ff_pid_core import Route

source_scripts = os.path.join(
    rospkg.RosPack().get_path("static_livox_localization"), "scripts")
sys.path.insert(0, source_scripts)
from body_frame import (pose_correction, reference_correction,
                        route_chair_centre, REFERENCE_CHAIR_CENTRE)
from route_mask import RouteMask

MASK_RESERVE_M = 0.30             # predicted centre to mask boundary
CHAIR_HALF_WIDTH_M = 0.30         # user-confirmed total width 0.60 m
CORRIDOR_EXTRA_M = 0.30
NEAR_STOP_M = 0.50                # gap from approximate chair footprint
FRONT_STOP_M = 1.00               # distance ahead along route, from front
FRONT_SLOW_M = 2.00
OBSTACLE_TIMEOUT_S = 1.00        # valid data stream timeout; zero objects is valid
PREDICTED_MAX_AGE_S = 0.30
# Axle-centred front extent 0.52 m is from the existing physical hardware URDF.
# Rear extent has not been measured: this 0.30 m is a provisional approximation.
CHAIR_FRONT_M = 0.52
CHAIR_REAR_M = 0.30
ROUTES = ("A_to_D4", "A_to_D5", "D4_to_A",
          "D4_to_D5", "D5_to_A", "D5_to_D4")


def box_of(item):
    try:
        size = item["size"]
        x, y, dx, dy = [float(v) for v in
                         (item["x"], item["y"], size[0], size[1])]
        if dx <= 0 or dy <= 0 or not all(
                math.isfinite(v) for v in (x, y, dx, dy)):
            return None
        return (x, y, dx, dy)
    except (TypeError, ValueError, KeyError, IndexError):
        return None


def footprint_gap(box):
    x, y, dx, dy = box
    gap_x = max(0.0, x - dx / 2 - CHAIR_FRONT_M,
                -CHAIR_REAR_M - (x + dx / 2))
    gap_y = max(0.0, y - dy / 2 - CHAIR_HALF_WIDTH_M,
                -CHAIR_HALF_WIDTH_M - (y + dy / 2))
    return math.hypot(gap_x, gap_y)


def object_samples(item, box):
    profile = item.get("profile")
    if profile is not None:
        try:
            step = float(profile["bin_m"])
            y0 = float(profile["y0"])
            values = profile["min_x"]
            if step <= 0 or not all(math.isfinite(v) for v in (step, y0)) \
                    or not isinstance(values, list):
                return None
            samples = []
            for i, value in enumerate(values):
                if value is None:
                    continue
                x, y = float(value), y0 + (i + 0.5) * step
                if not all(math.isfinite(v) for v in (x, y)):
                    return None
                samples.append((x, y))
            return samples
        except (TypeError, KeyError, ValueError, OverflowError):
            return None
    x, y, dx, dy = box
    return [(x - dx / 2, y - dy / 2), (x - dx / 2, y),
            (x - dx / 2, y + dy / 2)]


def path_gap(route, station, chair_pose, samples):
    if not samples:
        return None
    end = min(route.length, station + FRONT_SLOW_M + CHAIR_FRONT_M + 1.0)
    ss = np.r_[station, route.s[(route.s > station) & (route.s < end)], end]
    if len(ss) < 2 or ss[-1] - ss[0] < 1e-5:
        return None
    xy = np.column_stack((np.interp(ss, route.s, route.xy[:, 0]),
                          np.interp(ss, route.s, route.xy[:, 1])))
    segments = np.diff(xy, axis=0)
    lengths_sq = np.maximum(np.sum(segments * segments, axis=1), 1e-12)
    x, y, yaw = chair_pose
    c, s = math.cos(yaw), math.sin(yaw)
    half_corridor = CHAIR_HALF_WIDTH_M + CORRIDOR_EXTRA_M
    nearest = None
    for ox, oy in samples:
        point = np.asarray((x + c * ox - s * oy, y + s * ox + c * oy))
        delta = point - xy[:-1]
        t = np.clip(np.sum(delta * segments, axis=1) / lengths_sq, 0, 1)
        feet = xy[:-1] + t[:, None] * segments
        distances_sq = np.sum((feet - point) ** 2, axis=1)
        idx = int(np.argmin(distances_sq))
        if distances_sq[idx] > half_corridor ** 2:
            continue
        distance = max(0.0, float(ss[idx] + t[idx] *
                       (ss[idx + 1] - ss[idx]) - station - CHAIR_FRONT_M))
        nearest = distance if nearest is None else min(nearest, distance)
    return nearest


class Safety:
    def __init__(self):
        rospy.init_node("pid_safety")
        self.test_clear = str(rospy.get_param("~test_clear", False)).strip().lower() in (
            "true", "1", "yes", "on")
        if self.test_clear:
            self.policy_pub = rospy.Publisher(
                "/ff_pid/motion_policy", String, queue_size=2)
            self.status_pub = rospy.Publisher(
                "/pid_safety/status", String, queue_size=2)
            rospy.logwarn("TEST CLEAR ENABLED: obstacle/mask checks disabled")
            rospy.Timer(rospy.Duration(0.1), self.tick_test_clear)
            return
        name = str(rospy.get_param("~route"))
        if name not in ROUTES:
            raise rospy.ROSInitException("Invalid route: " + name)
        repo = os.path.dirname(os.path.dirname(
            os.path.realpath(rospkg.RosPack().get_path("pid"))))
        folder = os.path.join(repo, "routes", "route2_manual_20261031")
        self.route = Route(os.path.join(folder, name + "_waypoints.json"))
        self.mask = RouteMask(os.path.join(folder, "common_drivable_mask.yaml"))
        meta = self.route.metadata
        if meta.get("reference_point") != REFERENCE_CHAIR_CENTRE:
            raise rospy.ROSInitException("Route must be chair-centred")
        self.transform = pose_correction(
            "builtin", str(meta["body_frame_profile"])) @ reference_correction(
            str(meta["reference_point"]), route_chair_centre(meta))
        self.lock = threading.RLock()
        self.pose, self.pose_at = None, None
        self.predicted, self.predicted_at = None, None
        self.objects, self.objects_at = None, None
        self.last_reason = None
        self.policy_pub = rospy.Publisher(
            "/ff_pid/motion_policy", String, queue_size=2)
        self.status_pub = rospy.Publisher(
            "/pid_safety/status", String, queue_size=2)
        rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                         self.on_pose, queue_size=3)
        rospy.Subscriber("/ff_pid/status", String,
                         self.on_controller, queue_size=3)
        rospy.Subscriber("/perception/objects_summary", String,
                         self.on_obstacles, queue_size=3)
        rospy.Timer(rospy.Duration(0.1), self.tick)

    def tick_test_clear(self, _event):
        self.policy_pub.publish(String(data="CLEAR"))
        self.status_pub.publish(String(data=json.dumps({
            "state": "CLEAR", "reason": "TEST_CLEAR_BYPASS",
            "test_mode": True
        }, separators=(",", ":"))))

    def on_pose(self, msg):
        try:
            now = rospy.Time.now().to_sec()
            stamp = msg.header.stamp.to_sec()
            if msg.header.frame_id not in ("map", "/map") or \
                    stamp <= 0 or abs(now - stamp) > 0.5:
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            tf = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
            tf[:3, 3] = (p.x, p.y, p.z)
            tf = tf @ self.transform
            yaw = tft.euler_from_matrix(tf)[2]
            pose = (float(tf[0, 3]), float(tf[1, 3]), float(yaw))
            if not all(math.isfinite(v) for v in pose):
                return
            with self.lock:
                self.pose, self.pose_at = pose, now
        except (ValueError, TypeError):
            return

    def on_controller(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("state") != "RUNNING":
                with self.lock:
                    self.predicted = None
                return
            pose = tuple(float(v) for v in data["predicted_pose"])
            if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
                return
            with self.lock:
                self.predicted, self.predicted_at = pose, rospy.Time.now().to_sec()
        except (ValueError, TypeError, KeyError, AttributeError):
            return

    def on_obstacles(self, msg):
        try:
            obj = json.loads(msg.data)
        except (TypeError, ValueError):
            obj = None
        with self.lock:
            self.objects, self.objects_at = obj, rospy.Time.now().to_sec()

    def decide(self, now, pose, predicted, data, received):
        if pose is None:
            return "STOP", "POSE_MISSING", None
        if received is None or now - received >= OBSTACLE_TIMEOUT_S:
            return "STOP", "PERCEPTION_STALE", None
        if not isinstance(data, dict) or data.get("status") != "OK" or \
                data.get("frame") != "chair_centre" or \
                not isinstance(data.get("objects"), list):
            return "STOP", "PERCEPTION_NOT_OK", None
        stamp = data.get("stamp")
        if not isinstance(stamp, (int, float)) or \
                not math.isfinite(stamp) or stamp <= 0 or \
                stamp > now + 0.2 or now - stamp >= OBSTACLE_TIMEOUT_S:
            return "STOP", "PERCEPTION_SOURCE_STALE", None
        point = predicted if predicted is not None else pose
        if self.mask.clearance_at(point[:2]) < MASK_RESERVE_M:
            return "STOP", "MASK_BOUNDARY", None
        station = self.route.project(pose[0], pose[1])[0]
        closest, close_id = None, None
        for item in data["objects"]:
            if not isinstance(item, dict):
                return "STOP", "BAD_OBJECT", None
            box = box_of(item)
            if box is None:
                return "STOP", "BAD_OBJECT_GEOMETRY", item.get("id")
            # Fully outside-mask static objects cannot trigger the
            # global very-close stop merely by being near a curve.
            if not (item.get("band_relation") == "outside" and
                    item.get("motion") == "static"):
                if footprint_gap(box) < NEAR_STOP_M:
                    return "STOP", "VERY_CLOSE_OBJECT", item.get("id")
            samples = object_samples(item, box)
            if samples is None:
                return "STOP", "BAD_OBJECT_PROFILE", item.get("id")
            distance = path_gap(self.route, station, pose, samples)
            if distance is not None and (closest is None or distance < closest):
                closest, close_id = distance, item.get("id")
        if closest is not None and closest < FRONT_STOP_M:
            return "STOP", "PATH_OBSTACLE", close_id
        if closest is not None and closest < FRONT_SLOW_M:
            return "SLOW", "PATH_OBSTACLE", close_id
        return "CLEAR", "NO_PATH_OBSTACLE", None

    def tick(self, _event):
        now = rospy.Time.now().to_sec()
        with self.lock:
            pose = self.pose if self.pose_at is not None and \
                now - self.pose_at <= 0.5 else None
            predicted = self.predicted if self.predicted_at is not None and \
                now - self.predicted_at <= PREDICTED_MAX_AGE_S else None
            data, received = self.objects, self.objects_at
        try:
            policy, reason, object_id = self.decide(
                now, pose, predicted, data, received)
        except Exception as exc:
            policy, reason, object_id = "STOP", "SAFETY_ERROR", None
            rospy.logerr_throttle(2, "Safety error: %s", exc)
        self.policy_pub.publish(String(data=policy))
        self.status_pub.publish(String(data=json.dumps({
            "state": policy, "reason": reason, "object_id": object_id
        }, separators=(",", ":"))))
        if reason != self.last_reason:
            rospy.loginfo("Safety: %s (%s)", policy, reason)
            self.last_reason = reason


if __name__ == "__main__":
    Safety()
    rospy.spin()

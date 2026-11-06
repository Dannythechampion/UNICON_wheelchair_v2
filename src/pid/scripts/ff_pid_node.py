#!/usr/bin/env python3
"""UNICON FF-PD ROS1 driver node. Paused until /ff_pid/start=True.

Publishes REAL /cmd_vel after explicit start. Requires the existing NUC
wheel_cmd_tmp.py to permit callerid '/ff_pid' and have turn boost removed.
Does not implement obstacle recognition/avoidance; do not use in occupied areas.
"""
import json
import math
import os
import sys
import threading

import numpy as np
import rospy
import rosgraph
import rospkg
import tf.transformations as tft
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool, Int16MultiArray, String
from std_srvs.srv import SetBool, SetBoolResponse

# catkin_install_python may execute a relay script in devel/lib/pid.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ff_pid_core import FFPD, Route

existing_scripts = os.path.join(
    rospkg.RosPack().get_path("static_livox_localization"), "scripts")
sys.path.insert(0, existing_scripts)
from body_frame import (pose_correction, reference_correction,
                        route_chair_centre, REFERENCE_CHAIR_CENTRE)
from route_assets import validate_asset_binding
from route_mask import RouteMask
from safety_band import SafetyBand

AUTO_MODE = 65
TRACK = 0.54


def read_wheel(data):
    """Signed m/s, from the base's C/W/S + 33 + 10 counts/km/h protocol."""
    if len(data) < 6:
        raise ValueError("wheel_status too short")
    def decode(direction, magnitude):
        d = int(direction)
        if d not in (67, 87, 83):
            raise ValueError("invalid wheel direction")
        m = int(magnitude)
        if m < 33 or m > 127:
            raise ValueError("invalid wheel magnitude")
        if d == 83:
            return 0.0
        speed = (m - 33) / 10.0 / 3.6
        return speed if d == 67 else -speed
    return decode(data[2], data[3]), decode(data[4], data[5])


class FFNode:
    def __init__(self):
        rospy.init_node("ff_pid")  # callerid MUST be '/ff_pid' on /cmd_vel
        if rospy.get_name() != "/ff_pid":
            raise rospy.ROSInitException("node name must be exactly /ff_pid")
        # Route, band and mask must all be selected together. Never guess.
        route_path = rospy.get_param("~route")
        band_path = rospy.get_param("~safety_band")
        mask_path = rospy.get_param("~drivable_mask")
        profile = str(rospy.get_param("~body_frame_profile"))
        self.route = Route(route_path)
        metadata = self.route.metadata
        if metadata.get("reference_point") != REFERENCE_CHAIR_CENTRE:
            raise rospy.ROSInitException("only chair-centred routes are allowed")
        if "body_frame_profile" not in metadata:
            raise rospy.ROSInitException("route missing body_frame_profile")
        validate_asset_binding(route_path, band_path, mask_path)
        self.band = SafetyBand(band_path)
        self.mask = RouteMask(mask_path)
        # Same pose-to-route reference correction as existing follower.
        self.pose_transform = pose_correction(
            profile, str(metadata["body_frame_profile"])) @ reference_correction(
            str(metadata["reference_point"]), route_chair_centre(metadata))
        if (self.band.route_centre_clearance_violations() or
                self.band.route_centre_chord_violations()):
            raise rospy.ROSInitException("route/band safety preflight failed")
        # Parameters that affect motor behaviour are explicit, not inferred
        # from a simulation or a different machine.
        self.core = FFPD(
            self.route,
            kp=float(rospy.get_param("~kp", 0.3)),
            kd=float(rospy.get_param("~kd", 0.02)),
            ki=0.0,
            max_v=float(rospy.get_param("~max_v")),
            max_w=float(rospy.get_param("~max_w", 0.5)),
            min_wheel=float(rospy.get_param("~min_wheel_mps")),
            max_wheel=float(rospy.get_param("~max_wheel_mps")),
            wheel_track=TRACK,
            lateral_accel=float(rospy.get_param("~max_lateral_accel", 0.25)),
            preview_m=float(rospy.get_param("~preview_m", 2.0)))
        self.lock = threading.RLock()
        self.enabled = False
        self.finishing = False
        self.external_hold = False
        self.stopped_reports = 0
        self.competing_sources = []
        self.max_cte = float(rospy.get_param("~max_cte", 0.4))
        self.max_heading_error = float(rospy.get_param("~max_heading_error", 0.5))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.4))
        self.stopped_wheel_threshold = float(rospy.get_param("~stopped_wheel_threshold", 0.06))
        self.pose = None
        self.pose_received = None
        self.wheel = None
        self.wheel_pair = None
        self.wheel_received = None
        self.mode = None
        self.tracking = ""
        self.diag_received = None
        self.reason = "PAUSED"
        self.last_tick = None
        self.max_pose_age = float(rospy.get_param("~max_pose_age", 0.5))
        self.max_wheel_age = float(rospy.get_param("~max_wheel_age", 0.35))
        self.max_diag_age = float(rospy.get_param("~max_diag_age", 2.0))
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status = rospy.Publisher("/ff_pid/status", String, queue_size=3)
        rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                         self.on_pose, queue_size=5)
        rospy.Subscriber("/wheel_status", Int16MultiArray,
                         self.on_wheel, queue_size=5)
        rospy.Subscriber("/fast_lio_icp/localization_diagnostics", DiagnosticArray,
                         self.on_diag, queue_size=5)
        # An independent safety supervisor may request STOP; it does not
        # publish a second /cmd_vel and cannot remotely re-arm the controller.
        rospy.Subscriber("/ff_pid/stop_request", Bool, self.on_external_stop,
                         queue_size=2)
        rospy.Service("/ff_pid/start", SetBool, self.on_start)
        # Querying ROS Master can block. Keep it off the 10Hz control path.
        self.peer_timer = rospy.Timer(rospy.Duration(1.0), self.on_peer_check)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.on_tick)
        rospy.on_shutdown(self.stop)
        rospy.loginfo("FF-PD ready: %s, %d waypoints, %.1f m; PAUSED",
                      route_path, len(self.route.xy), self.route.length)

    def on_pose(self, msg):
        try:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            if msg.header.frame_id not in ("map", "/map"):
                return  # Do not treat local odometry as global map coordinates.
            matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
            matrix[:3, 3] = (p.x, p.y, p.z)
            matrix = matrix @ self.pose_transform
            _, _, yaw = tft.euler_from_matrix(matrix)
            value = (float(matrix[0, 3]), float(matrix[1, 3]), float(yaw))
            if not all(math.isfinite(z) for z in value):
                return
            now = rospy.Time.now().to_sec()
            stamp = msg.header.stamp.to_sec()
            if stamp > 0 and abs(now - stamp) > self.max_pose_age:
                return
            with self.lock:
                self.pose, self.pose_received = value, now
        except Exception as exc:
            rospy.logwarn_throttle(5, "bad pose: %s", exc)

    def on_wheel(self, msg):
        try:
            left, right = read_wheel(msg.data)
            v = (left + right)/2
            w = (right - left)/TRACK
            mode = int(msg.data[1])
            with self.lock:
                self.wheel = (v, w)
                self.wheel_pair = (left, right)
                self.wheel_received = rospy.Time.now().to_sec()
                self.mode = mode
                if self.finishing:
                    if (abs(left) <= self.stopped_wheel_threshold and
                            abs(right) <= self.stopped_wheel_threshold):
                        self.stopped_reports += 1
                    else:
                        self.stopped_reports = 0
        except (TypeError, ValueError) as exc:
            rospy.logwarn_throttle(5, "invalid wheel_status: %s", exc)

    def on_diag(self, msg):
        for d in msg.status:
            if d.name == "fast_lio_icp":
                with self.lock:
                    self.tracking = d.message
                    self.diag_received = rospy.Time.now().to_sec()
                return

    def on_external_stop(self, msg):
        with self.lock:
            if msg.data:
                self.external_hold = True
                self.disable("EXTERNAL_STOP_REQUEST")
            else:
                self.external_hold = False
                # Releasing the stop request never re-arms motion.

    def readiness(self, now):
        if self.pose is None or self.pose_received is None or now - self.pose_received > self.max_pose_age:
            return "POSE_STALE"
        if self.wheel is None or self.wheel_received is None or now - self.wheel_received > self.max_wheel_age:
            return "WHEEL_STALE"
        if self.diag_received is None or now - self.diag_received > self.max_diag_age or self.tracking != "TRACKING":
            return "NO_TRACKING"
        if self.mode != AUTO_MODE:
            return "NOT_AUTO_MODE"
        x, y, _ = self.pose
        if not self.mask.contains((x, y)) or not self.band.contains(np.array([x, y])):
            return "OUTSIDE_SAFE_ROUTE"
        return None

    def competing_publishers(self):
        """Never compete with an active tip_guard or other /cmd_vel source."""
        try:
            published = rosgraph.Master(rospy.get_name()).getSystemState()[0]
            publishers = next((nodes for topic, nodes in published
                               if topic == "/cmd_vel"), [])
            return [node for node in publishers if node != rospy.get_name()]
        except Exception:
            return ["ROS_MASTER_UNAVAILABLE"]  # fail closed

    def on_peer_check(self, event):
        sources = self.competing_publishers()  # ROS Master query at 1Hz
        with self.lock:
            self.competing_sources = sources

    def stop(self):
        if hasattr(self, "pub"):
            self.pub.publish(Twist())

    def disable(self, reason):
        """Leave drive mode and clear ALL PID state, including integral."""
        self.enabled = False
        self.core.reset()
        self.last_tick = None
        self.stop()
        self.publish_status(reason)

    def on_start(self, req):
        with self.lock:
            if not req.data:
                self.finishing = False
                self.stopped_reports = 0
                self.disable("PAUSED")
                return SetBoolResponse(True, "FF-PD PAUSED")
            if self.finishing:
                return SetBoolResponse(False, "GOAL_FINISHING")
            if self.external_hold:
                return SetBoolResponse(False, "EXTERNAL_STOP_REQUEST active")
            others = self.competing_publishers()
            if others:
                return SetBoolResponse(False, "REFUSED competing publishers: " + ", ".join(others))
            reason = self.readiness(rospy.Time.now().to_sec())
            if reason:
                return SetBoolResponse(False, "REFUSED: " + reason)
            self.core.reset()
            self.last_tick = None
            self.enabled = True
            self.publish_status("RUNNING")
            return SetBoolResponse(True, "FF-PD STARTED")

    def on_tick(self, event):
        now = rospy.Time.now().to_sec()
        with self.lock:
            others = self.competing_sources
            if others:
                if self.enabled or self.finishing:
                    self.enabled = False
                    self.finishing = False
                    self.core.reset()  # clear I state without racing another publisher
                    self.last_tick = None
                    self.publish_status("COMPETING_CMD_VEL_PUBLISHER")
                    rospy.logerr("FF-PD command conflict: %s", others)
                # Do not fight another publisher, even with zero Twist.
                return
            if self.finishing:
                # Do not exit after only one zero Twist. Existing base uses
                # repeated stop-ramp commands based on fresh wheel feedback.
                self.stop()
                self.publish_status("GOAL_STOPPING")
                if (self.stopped_reports >= 3 and self.wheel_received is not None
                        and now - self.wheel_received <= self.max_wheel_age):
                    self.publish_status("GOAL_REACHED")
                    rospy.loginfo("FF-PD goal reached; wheels stopped; exiting")
                    rospy.signal_shutdown("GOAL_REACHED")
                return
            if not self.enabled:
                self.stop()
                return
            reason = self.readiness(now)
            if reason is None and self.external_hold:
                reason = "EXTERNAL_STOP_REQUEST"
            if reason is not None:
                self.disable("HOLD: " + reason)
                return
            dt = 0.1 if self.last_tick is None else now - self.last_tick
            self.last_tick = now
            try:
                result = self.core.step(self.pose, self.wheel, now, dt)
                # Geometric accept/reject and goal lifecycle live in NODE,
                # not in the FF-PD arithmetic or external safety supervisor.
                if (result["distance"] > self.max_cte or
                        abs(result["heading_error"]) > self.max_heading_error):
                    reason = "PATH_OR_HEADING_DEVIATION"
                elif (result["station"] >= self.route.length - self.goal_tolerance and
                      math.hypot(self.pose[0] - self.route.xy[-1, 0],
                                 self.pose[1] - self.route.xy[-1, 1]) < self.goal_tolerance):
                    # Stop first, wait for reported wheel stop, then exit node.
                    self.disable("GOAL_STOPPING")
                    self.finishing = True
                    self.stopped_reports = 0
                    return
                elif result["predicted_distance"] > self.max_cte:
                    reason = "PREDICTED_PATH_DEVIATION"
                elif not result["feasible"]:
                    reason = result["hold_reason"]
                else:
                    px, py, _ = result["predicted_pose"]
                    if (not self.mask.contains((px, py)) or
                            not self.band.contains(np.array([px, py]))):
                        reason = "PREDICTED_ROUTE_BOUNDARY"
                if reason is not None:
                    self.disable("HOLD: " + reason)
                    rospy.logerr("FF-PD HOLD, manual restart required: %s", reason)
                    return
                command = Twist()
                command.linear.x = result["v"]
                command.angular.z = result["w"]
                self.pub.publish(command)
                self.core.predictor.record(now, result["v"], result["w"])
                self.publish_status("RUNNING", result)
            except Exception as exc:
                self.disable("HOLD: " + str(exc))
                rospy.logerr("FF-PD computation failed: %s", exc)

    def publish_status(self, state, result=None):
        if state != self.reason or result is not None:
            self.reason = state
            values = {"state": state}
            if result is not None:
                values.update({k: v for k, v in result.items() if k != "predicted_pose"})
            self.status.publish(String(data=json.dumps(values)))


if __name__ == "__main__":
    try:
        FFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
"""Read-only ROS1 status/event display for pid_ready.sh (no motor commands).

  python3 pid_ready_monitor.py --status   # one snapshot
  python3 pid_ready_monitor.py --watch    # print only changes and faults

No direct dependency on Livox CustomMsg: the raw LiDAR topic uses AnyMsg for
receipt-liveness monitoring. Never treat this monitor as a safety controller.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

try:
    import rosnode
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Int16MultiArray, String
    from visualization_msgs.msg import MarkerArray
except ImportError as exc:
    sys.exit("[FAIL] ROS Noetic Python environment is missing: %s" % exc)


SIGNALS = {
    "LiDAR": ("lidar", 2.0),
    "IMU": ("imu", 2.0),
    "FAST-LIO": ("odom", 2.0),
    "Map Pose": ("pose", 2.0),
    "GICP Diagnostics": ("diag", 3.5),
    "Perception": ("perception", 2.5),
    "GICP Exclusions": ("exclusion", 2.5),
    "Wheel Status": ("wheel", 2.5),
    "UART Diagnostics": ("uart", 2.5),
}
REQUIRED_NODES = (
    "/uart", "/wheel_cmd", "/stop_watchdog",
    "/moving_icp_localizer", "/hybrid_geometric_objects",
    "/hybrid_object_fusion", "/localization_exclusion_boxes",
)


def when():
    return time.strftime("%H:%M:%S")


def finite_float(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError):
        return None


def yaw_of(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def mode_label(number):
    return {65: "AUTO", 77: "MANUAL"}.get(number, "UNKNOWN(%s)" % number)


def color(code, text):
    if not sys.stdout.isatty() or os.environ.get("NO_COLOR") == "1":
        return text
    return "\033[%sm%s\033[0m" % (code, text)


def event(level, message):
    codes = {"OK": "32", "WARN": "33", "ERROR": "31", "INFO": "36"}
    stamp = "[%s]" % when()
    prefix = color(codes.get(level, "0"), "[%s]" % level)
    print("%s %s %s" % (stamp, prefix, message), flush=True)


class Monitor:
    def __init__(self):
        rospy.init_node("pid_ready_monitor", anonymous=True, disable_signals=True)
        self.lock = threading.RLock()
        self.received = {}
        self.payload = {}
        self.node_names = set()
        self.last_graph_check = 0.0
        self.valid_registration = None
        self.alarm = None
        self.pose_source_stamp = None
        self.perception_source_stamp = None

        self._subscribe("lidar", "/livox/lidar", rospy.AnyMsg)
        self._subscribe("imu", "/livox/imu", Imu)
        self._subscribe("odom", "/Odometry", Odometry)
        self._subscribe("pose", "/fast_lio_icp/pose", PoseWithCovarianceStamped)
        self._subscribe("diag", "/fast_lio_icp/localization_diagnostics", DiagnosticArray)
        self._subscribe("perception", "/perception/objects_summary", String)
        self._subscribe("exclusion", "/perception/dynamic_boxes", MarkerArray)
        self._subscribe("wheel", "/wheel_status", Int16MultiArray)
        self._subscribe("uart", "/uart_tx_diag", String)
        rospy.Subscriber("/stop_watchdog/alarm", String, self.on_alarm, queue_size=2)

    def _subscribe(self, key, topic, typ):
        rospy.Subscriber(topic, typ,
                         lambda msg, k=key: self.on_message(k, msg), queue_size=1)

    def on_message(self, key, msg):
        now = time.monotonic()
        with self.lock:
            self.received[key] = now
            if key == "pose":
                self.pose_source_stamp = msg.header.stamp.to_sec()
                self.payload[key] = msg.pose.pose
            elif key == "diag":
                for status in msg.status:
                    if status.name != "fast_lio_icp":
                        continue
                    values = {v.key: v.value for v in status.values}
                    self.payload[key] = (status.message, values)
                    reason = values.get("reason", "")
                    fitness = finite_float(values.get("fitness"))
                    inlier = finite_float(values.get("inlier_ratio"))
                    elapsed = finite_float(values.get("registration_elapsed_ms"))
                    points = finite_float(values.get("source_points"))
                    # Invalid/no-op diagnostics contain default result fields;
                    # do not show them as a GICP quality measurement.
                    if (reason != "STATIONARY_CORRECTION_SUPPRESSED"
                            and fitness is not None and 0 <= fitness < 1e8
                            and inlier is not None and 0 <= inlier <= 1
                            and elapsed is not None and elapsed > 0
                            and points is not None and points > 0):
                        self.valid_registration = (fitness, inlier, elapsed, now)
            elif key in ("perception", "uart"):
                try:
                    value = json.loads(msg.data)
                except (ValueError, TypeError):
                    value = {"status": "INVALID_JSON"}
                self.payload[key] = value
                if key == "perception":
                    self.perception_source_stamp = finite_float(value.get("stamp"))
            elif key == "wheel":
                self.payload[key] = list(msg.data)

    def on_alarm(self, msg):
        with self.lock:
            try:
                self.alarm = json.loads(msg.data)
            except (TypeError, ValueError):
                self.alarm = {"reason": msg.data}

    def check_nodes(self):
        now = time.monotonic()
        if now - self.last_graph_check < 5.0:
            return
        try:
            names = set(rosnode.get_node_names())
        except Exception:
            names = set()
        with self.lock:
            self.node_names = names
            self.last_graph_check = now

    def snapshot(self):
        self.check_nodes()
        now = time.monotonic()
        wall_now = time.time()
        with self.lock:
            seen = dict(self.received)
            data = dict(self.payload)
            nodes = set(self.node_names)
            last_reg = self.valid_registration
            alarm = self.alarm
            pose_src = self.pose_source_stamp
            perception_src = self.perception_source_stamp
        conditions = {}
        for label, (key, limit) in SIGNALS.items():
            age = now - seen[key] if key in seen else float("inf")
            conditions[label] = "OK" if age <= limit else "STALE"

        if pose_src is not None and (wall_now - pose_src > 2.0 or
                                     pose_src > wall_now + 0.2):
            conditions["Map Pose"] = "STALE_SOURCE_TIMESTAMP"

        loc_state, diagnostic = data.get("diag", ("NO_DATA", {}))
        if conditions["GICP Diagnostics"] != "OK":
            loc_state = "NO_DIAGNOSTICS"
        conditions["GICP State"] = loc_state

        perception = data.get("perception", {})
        if conditions["Perception"] == "OK":
            if perception.get("status") != "OK":
                conditions["Perception"] = "STATUS_" + str(perception.get("status", "UNKNOWN"))
            elif (perception_src is None or wall_now - perception_src > 2.5 or
                  perception_src > wall_now + 0.2):
                conditions["Perception"] = "STALE_SOURCE_TIMESTAMP"

        wheel = data.get("wheel", [])
        mode = int(wheel[1]) if len(wheel) > 1 else None
        if conditions["Wheel Status"] == "OK" and mode not in (65, 77):
            conditions["Wheel Status"] = "INVALID_MODE"
        conditions["Motor Mode"] = mode_label(mode)

        uart = data.get("uart", {})
        for key in ("tx_fail", "tx_slow", "tx_ok"):
            try:
                uart[key] = int(uart.get(key, 0))
            except (ValueError, TypeError):
                uart[key] = 0

        for node in REQUIRED_NODES:
            conditions["Node " + node] = "OK" if node in nodes else "NOT_RUNNING"
        conditions["Stop Watchdog Alarm"] = "ALARM" if alarm else "OK"
        return conditions, data, last_reg, alarm, diagnostic


def print_snapshot(snapshot):
    conditions, data, last_reg, alarm, diagnostic = snapshot
    print("\n" + "=" * 52)
    print("            UNICON PID READY - STATUS")
    print("=" * 52)
    for label in ("LiDAR", "IMU", "FAST-LIO", "Map Pose", "GICP Diagnostics",
                  "Perception", "GICP Exclusions", "Wheel Status",
                  "UART Diagnostics", "Stop Watchdog Alarm"):
        value = conditions[label]
        display = color("32" if value == "OK" else "31", value)
        print(" %-23s : %s" % (label, display))
    print(" GICP State              : %s" % conditions["GICP State"])
    print(" Initialization           : %s" %
          rospy.get_param("/fast_lio_icp/auto_initialization_source", "unknown"))
    pose = data.get("pose")
    if pose is not None and conditions["Map Pose"] == "OK":
        print(" Map Position             : (%.2f, %.2f) m" %
              (pose.position.x, pose.position.y))
        print(" Map Yaw                  : %.1f deg" % yaw_of(pose.orientation))
    else:
        print(" Map Position             : unavailable")
    if last_reg is not None:
        fitness, inlier, elapsed, monotonic_stamp = last_reg
        print(" Last Valid GICP          : fitness %.4f | inlier %.3f | %.1f ms" %
              (fitness, inlier, elapsed))
        print(" GICP Measurement Age     : %.1f s (not pose age)" %
              (time.monotonic() - monotonic_stamp))
    else:
        print(" Last Valid GICP          : unavailable (not measured in this monitor session)")
    print(" Motor Mode               : %s" % conditions["Motor Mode"])
    per = data.get("perception", {})
    objects = per.get("objects", [])
    if isinstance(objects, list) and conditions["Perception"] == "OK":
        moving = sum(1 for item in objects if item.get("motion") == "moving")
        static = sum(1 for item in objects if item.get("motion") == "static")
        print(" Objects                  : %d (moving %d, static %d)" %
              (len(objects), moving, static))
    if alarm:
        print(" WATCHDOG ALARM           : %s" % alarm.get("reason", alarm))
    print("=" * 52)
    print("Snapshot only. Readiness does NOT authorize motion.\n", flush=True)


def watch(monitor):
    time.sleep(1.5)
    previous, prev_data, _, _, _ = monitor.snapshot()
    event("INFO", "Event monitor active. Normal operation stays silent; Ctrl+C exits display only.")
    # On entry, report already-failed conditions once.
    for key, value in sorted(previous.items()):
        if key == "Motor Mode":
            continue
        if value != "OK" and not (key == "GICP State" and value == "TRACKING"):
            event("WARN", "%s: %s" % (key, value))
    prev_uart = dict(prev_data.get("uart", {}))
    while not rospy.is_shutdown():
        time.sleep(2.0)
        conditions, data, _, alarm, _ = monitor.snapshot()
        for key, current in conditions.items():
            past = previous.get(key)
            if current == past:
                continue
            if past is None:
                continue
            if key == "Motor Mode":
                event("INFO", "Motor mode: %s -> %s" % (past, current))
            elif key == "GICP State":
                if current == "TRACKING":
                    event("OK", "GICP: %s -> TRACKING" % past)
                else:
                    event("WARN", "GICP: %s -> %s" % (past, current))
            elif current == "OK":
                event("OK", "%s recovered (%s -> OK)" % (key, past))
            else:
                event("ERROR" if key == "Stop Watchdog Alarm" else "WARN",
                      "%s: %s -> %s" % (key, past, current))
        uart = data.get("uart", {})
        if conditions["UART Diagnostics"] == "OK":
            for key in ("tx_fail", "tx_slow"):
                old = int(prev_uart.get(key, 0))
                new = int(uart.get(key, 0))
                if new > old:
                    event("WARN", "UART %s increased: %d -> %d" % (key, old, new))
        prev_uart = dict(uart)
        previous = conditions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--status", action="store_true")
    group.add_argument("--watch", action="store_true")
    group.add_argument("--check-ready", action="store_true")
    args = parser.parse_args()
    monitor = Monitor()
    try:
        if args.check_ready:
            time.sleep(2.0)
            conditions, data, _, _, _ = monitor.snapshot()
            required = ("LiDAR", "IMU", "FAST-LIO", "Map Pose",
                        "GICP Diagnostics", "Perception", "GICP Exclusions",
                        "Wheel Status", "UART Diagnostics", "Stop Watchdog Alarm")
            failed = [(key, conditions[key]) for key in required
                      if conditions[key] != "OK"]
            failed += [(key, conditions[key]) for key in conditions
                       if key.startswith("Node ") and conditions[key] != "OK"]
            if conditions["GICP State"] != "TRACKING":
                failed.append(("GICP State", conditions["GICP State"]))
            if conditions["Motor Mode"] != "MANUAL":
                failed.append(("Motor Mode", conditions["Motor Mode"]))
            # Stamped init receipts distinguish legitimate tracking from a
            # still-running old or partially initialized localizer.
            if rospy.get_param("/fast_lio_icp/auto_initialization_verified", False) is not True:
                failed.append(("Initial fix", "NOT_VERIFIED"))
            if rospy.get_param("/fast_lio_icp/auto_initialization_stable", False) is not True:
                failed.append(("Initial fix", "NOT_STABLE"))
            if failed:
                for key, value in failed:
                    event("ERROR", "%s: %s" % (key, value))
                sys.exit(1)
            event("OK", "Full ready preflight passed")
        elif args.status:
            time.sleep(1.5)
            print_snapshot(monitor.snapshot())
        else:
            watch(monitor)
    except KeyboardInterrupt:
        print("\n[INFO] Display detached. ROS stack remains running.")


if __name__ == "__main__":
    main()

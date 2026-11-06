#!/usr/bin/env python3
"""Read-only, short-lived ROS preflight measurements for pid_drive.sh.

Does not publish any driving or mode commands.
"""
import argparse
import math
import os
import sys
import threading
import time


def _wheel_speed(direction, magnitude):
    d, m = int(direction), int(magnitude)
    if d not in (67, 87, 83) or not 33 <= m <= 127:
        raise ValueError("invalid wheel_status encoding")
    if d == 83:
        return 0.0
    v = (m - 33) / 10.0 / 3.6
    return v if d == 67 else -v


def watch_wheels(args):
    import rospy
    from std_msgs.msg import Int16MultiArray

    lock = threading.Lock()
    state = {"seen": 0, "ok": 0, "last": "NO_DATA"}
    expected = 65 if args.mode == "AUTO" else 77
    count_needed = 3

    def callback(msg):
        d = msg.data
        with lock:
            state["seen"] += 1
            try:
                if len(d) < 6:
                    raise ValueError("short status")
                mode = int(d[1])
                left = _wheel_speed(d[2], d[3])
                right = _wheel_speed(d[4], d[5])
                state["last"] = "mode=%s, L=%.3f m/s, R=%.3f m/s" % (mode, left, right)
                good = mode == expected and (not args.stopped or
                                             (abs(left) <= args.speed_threshold and
                                              abs(right) <= args.speed_threshold))
                state["ok"] = state["ok"] + 1 if good else 0
            except (TypeError, ValueError) as exc:
                state["last"] = str(exc)
                state["ok"] = 0

    rospy.init_node("pid_drive_check_wheels", anonymous=True, disable_signals=True)
    rospy.Subscriber("/wheel_status", Int16MultiArray, callback, queue_size=10)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        with lock:
            if state["ok"] >= count_needed:
                print("[OK] Wheel feedback: %s" % state["last"], flush=True)
                return 0
        time.sleep(0.05)
    with lock:
        print("[FAIL] Expected %s%s (%d consecutive reports). Last: %s" %
              (args.mode, " and stopped" if args.stopped else "", count_needed,
               state["last"]), file=sys.stderr, flush=True)
    return 1


def check_pose_on_route(args):
    import numpy as np
    import rospy
    import rospkg
    import tf.transformations as tft
    from geometry_msgs.msg import PoseWithCovarianceStamped

    here = os.path.dirname(os.path.realpath(__file__))
    repo = os.path.realpath(os.path.join(here, "../../.."))
    sys.path.insert(0, here)
    from ff_pid_core import Route, wrap

    localization_scripts = os.path.join(
        rospkg.RosPack().get_path("static_livox_localization"), "scripts")
    sys.path.insert(0, localization_scripts)
    from body_frame import (pose_correction, reference_correction,
                            route_chair_centre, REFERENCE_CHAIR_CENTRE)

    route_path = os.path.join(repo, "routes", "route2_manual_20261031",
                              args.route + "_waypoints.json")
    route = Route(route_path)
    metadata = route.metadata
    if metadata.get("reference_point") != REFERENCE_CHAIR_CENTRE:
        raise ValueError("route reference must be chair_centre")
    profile = metadata["body_frame_profile"]
    correction = pose_correction("builtin", str(profile)) @ reference_correction(
        str(metadata["reference_point"]), route_chair_centre(metadata))

    rospy.init_node("pid_drive_check_start", anonymous=True, disable_signals=True)
    deadline = time.monotonic() + args.timeout
    last_issue = "NO_POSE"
    while time.monotonic() < deadline and not rospy.is_shutdown():
        remaining = max(0.1, deadline - time.monotonic())
        try:
            msg = rospy.wait_for_message("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                                         timeout=min(1.0, remaining))
        except rospy.ROSException:
            continue
        if msg.header.frame_id not in ("map", "/map"):
            last_issue = "pose frame is not map"
            continue
        stamp = msg.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if stamp <= 0 or not -0.2 <= age <= 0.5:
            last_issue = "pose timestamp not fresh (age=%.2fs)" % age
            continue
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        matrix[:3, 3] = (p.x, p.y, p.z)
        matrix = matrix @ correction
        _, _, yaw = tft.euler_from_matrix(matrix)
        x, y = float(matrix[0, 3]), float(matrix[1, 3])
        if not np.isfinite([x, y, yaw]).all():
            print("[FAIL] Invalid numeric pose", file=sys.stderr)
            return 1
        # Project onto the WHOLE route, not its first waypoint. The operator
        # may pause and resume at any point along this same route.
        station, cte, dist, tangent = route.project(x, y)
        heading_error_rad = abs(wrap(yaw - tangent))
        print("[INFO] Current chair centre (map): x=%.3f, y=%.3f, yaw=%.1f deg" %
              (x, y, math.degrees(yaw)))
        print("[INFO] Nearest route progress: s=%.2f / %.2f m" %
              (station, route.length))
        print("[INFO] Path offset: %.3f m (CTE %+0.3f m), heading error %.1f deg" %
              (dist, cte, math.degrees(heading_error_rad)))
        # These match the FF node's default path/heading HOLD thresholds.
        # They are NOT constraints on distance from the START waypoint.
        if dist > args.max_distance or heading_error_rad > args.max_heading_rad:
            print("[FAIL] Current pose is not safely aligned to route "
                  "(limit: %.2f m / %.1f deg). Keep MANUAL." %
                  (args.max_distance, math.degrees(args.max_heading_rad)),
                  file=sys.stderr)
            return 1
        print("[OK] Current position is compatible with route (mid-route resume allowed)")
        return 0
    print("[FAIL] No fresh map pose: %s" % last_issue, file=sys.stderr)
    return 1


def show_position(args):
    """One-shot GICP pose query. Do not launch/control the chair."""
    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    import tf.transformations as tft

    rospy.init_node("pid_drive_position_view", anonymous=True, disable_signals=True)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message('/fast_lio_icp/pose', PoseWithCovarianceStamped,
                                         timeout=1.0)
        except rospy.ROSException:
            continue
        if msg.header.frame_id not in ('map', '/map'):
            continue
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0 or not -0.2 <= rospy.Time.now().to_sec() - stamp <= 0.5:
            continue
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = tft.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        print('[POSITION] GICP Map pose (sensor/body reference): '
              'x=%.3f m, y=%.3f m, yaw=%.1f deg' %
              (pos.x, pos.y, math.degrees(yaw)), flush=True)
        return 0
    print('[FAIL] Fresh map pose unavailable', file=sys.stderr)
    return 1


def watch_states(args):
    """DRIVE_EVENT_MONITOR_V1: state changes immediately; progress 30s only."""
    import json
    import rospy
    import rosgraph
    from std_msgs.msg import String

    here = os.path.dirname(os.path.realpath(__file__))
    route_file = os.path.realpath(os.path.join(
        here, '../../../routes/route2_manual_20261031',
        args.route + '_waypoints.json'))
    sys.path.insert(0, here)
    from ff_pid_core import Route
    route_length = Route(route_file).length

    lock = threading.RLock()
    state = {'drive': None, 'station': None, 'objects': None,
             'object_updated': 0.0, 'object_state': None,
             'object_pending': None, 'object_pending_at': 0.0}

    def drive_callback(msg):
        try:
            payload = json.loads(msg.data)
            new_state = str(payload.get('state', 'UNKNOWN'))
            station = payload.get('station')
            if station is not None:
                station = float(station)
                if not math.isfinite(station):
                    station = None
        except (TypeError, ValueError):
            new_state, station = 'INVALID_STATUS', None
        with lock:
            if new_state != state['drive']:
                state['drive'] = new_state
                print('[FF-PD] %s' % new_state, flush=True)
            if station is not None:
                state['station'] = station

    def object_callback(msg):
        try:
            payload = json.loads(msg.data)
            objects = payload.get('objects') if payload.get('status') == 'OK' else None
            if not isinstance(objects, list):
                objects = None
        except (TypeError, ValueError, AttributeError):
            objects = None
        with lock:
            state['objects'] = objects
            state['object_updated'] = time.monotonic()

    rospy.init_node('pid_drive_state_view', anonymous=True)
    rospy.Subscriber('/ff_pid/status', String, drive_callback, queue_size=10)
    rospy.Subscriber('/perception/objects_summary', String, object_callback, queue_size=2)
    master = rosgraph.Master(rospy.get_name())
    last_progress = time.monotonic()
    while not rospy.is_shutdown():
        try:
            master.lookupNode('/ff_pid')
        except Exception:
            print('[INFO] /ff_pid node exited.', flush=True)
            break
        now = time.monotonic()
        with lock:
            drive_state = state['drive']
            station = state['station']
            objects = state['objects']
            object_updated = state['object_updated']
            previous_presence = state['object_state']
            if objects is not None and now - object_updated <= 3.0:
                present = len(objects) > 0
                if previous_presence is None:
                    state['object_state'] = present
                    if present:
                        print('[OBJECT] Already detected: %d' % len(objects), flush=True)
                elif present == previous_presence:
                    state['object_pending'] = None
                elif state['object_pending'] != present:
                    state['object_pending'] = present
                    state['object_pending_at'] = now
                elif now - state['object_pending_at'] >= 2.0:
                    state['object_state'] = present
                    state['object_pending'] = None
                    if present:
                        print('[OBJECT] Detected: %d' % len(objects), flush=True)
                    else:
                        print('[OBJECT] Cleared', flush=True)
        if now - last_progress >= 30.0:
            if drive_state == 'RUNNING' and station is not None:
                print('[PROGRESS] %.1f / %.1f m (%.1f%%)' %
                      (station, route_length, 100.0 * station / route_length), flush=True)
            last_progress = now
        rospy.sleep(0.5)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    wheels = sub.add_parser("wheel", help="Wait for repeated matching wheel status")
    wheels.add_argument("--mode", choices=("AUTO", "MANUAL"), required=True)
    wheels.add_argument("--stopped", action="store_true")
    wheels.add_argument("--speed-threshold", type=float, default=0.06)
    wheels.add_argument("--timeout", type=float, default=6.0)
    start = sub.add_parser("pose-on-route", help="Check current chair pose against ANY station on route")
    start.add_argument("--route", required=True)
    start.add_argument("--max-distance", type=float, default=0.70)
    start.add_argument("--max-heading-rad", type=float, default=1.50)
    start.add_argument("--timeout", type=float, default=5.0)
    watch = sub.add_parser("watch", help="Quiet drive status and periodic progress")
    watch.add_argument("--route", required=True)
    sub.add_parser("position", help="Print current GICP Map pose only")
    args = parser.parse_args()
    try:
        if args.command == "wheel":
            return watch_wheels(args)
        if args.command == "pose-on-route":
            return check_pose_on_route(args)
        if args.command == "position":
            return show_position(args)
        return watch_states(args)
    except (ImportError, ValueError, KeyError, OSError, RuntimeError) as exc:
        print("[FAIL] Preflight: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

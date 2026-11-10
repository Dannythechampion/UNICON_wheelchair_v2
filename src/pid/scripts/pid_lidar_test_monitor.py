#!/usr/bin/env python3
"""Read-only ROS1 MID-360 stream watchdog; writes a short atomic JSON snapshot."""
import argparse
import json
import os
from pathlib import Path
import time

import rospy
from sensor_msgs.msg import Imu


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-dir", required=True)
    args = parser.parse_args()
    state_dir = Path(args.state_dir)
    rospy.init_node("pid_lidar_test_monitor", anonymous=True)
    last = {"lidar": None, "imu": None}
    count = {"lidar": 0, "imu": 0}
    losses = {"lidar": 0, "imu": 0}
    previously_ok = {"lidar": False, "imu": False}

    def received(which):
        last[which] = time.monotonic()
        count[which] += 1

    rospy.Subscriber("/livox/lidar", rospy.AnyMsg, lambda msg: received("lidar"), queue_size=1)
    rospy.Subscriber("/livox/imu", Imu, lambda msg: received("imu"), queue_size=1)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        now = time.monotonic()
        payload = {"state": "RUNNING", "timestamp": time.time()}
        for name in ("lidar", "imu"):
            age = None if last[name] is None else now - last[name]
            ok = age is not None and age <= 1.0
            if previously_ok[name] and not ok:
                losses[name] += 1
            previously_ok[name] = ok
            payload[name] = "OK" if ok else "WAITING" if age is None else "STALE"
            payload[name + "_count"] = count[name]
            payload[name + "_dropouts"] = losses[name]
            payload[name + "_age_s"] = None if age is None else round(age, 1)
        for name in ("master", "driver"):
            pidfile = state_dir / (name + ".pid")
            try:
                pid = int(pidfile.read_text().strip())
                stat = Path("/proc/%d/stat" % pid).read_text()
                alive = stat.split(") ", 1)[1][0] != "Z"
            except (OSError, ValueError, IndexError):
                alive = False
            payload[name] = "RUNNING" if alive else "STOPPED"
        target = state_dir / "status.json"
        tmp = state_dir / ("status.%d.tmp" % os.getpid())
        tmp.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        os.replace(str(tmp), str(target))
        rate.sleep()


if __name__ == "__main__":
    main()

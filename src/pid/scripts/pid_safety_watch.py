#!/usr/bin/env python3
"""Read-only terminal event monitor for /pid_safety/status.

Print the first status and subsequent (state, reason, route) changes only.
Do not print the 10 Hz telemetry stream or per-tick speed-cap adjustments.
No ROS motor, start, stop, or mode commands are published.
"""
import json
import math
import threading
import time
from datetime import datetime

import rospy
from std_msgs.msg import String


STALE_AFTER_S = 1.5


def event(message):
    print("%s [SAFETY] %s" %
          (datetime.now().strftime("%H:%M:%S"), message), flush=True)


class SafetyWatch:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_receive = time.monotonic()
        self.last_key = None
        rospy.Subscriber("/pid_safety/status", String, self.on_status, queue_size=10)

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                raise ValueError("status is not an object")
            state = str(status["state"])
            reason = str(status["reason"])
            route = str(status["route"])
            speed_cap = float(status["speed_limit_mps"])
            if state not in ("CLEAR", "SLOW", "STOP"):
                raise ValueError("unexpected state")
            if not math.isfinite(speed_cap) or speed_cap < 0:
                raise ValueError("invalid speed cap")
            key = (state, reason, route)
            message = "%s | reason=%s | limit=%.2f m/s | route=%s" % (
                state, reason, speed_cap, route)
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            key = ("INVALID_STATUS", str(exc))
            message = "INVALID_STATUS | %s" % exc

        with self.lock:
            self.last_receive = time.monotonic()
            if key != self.last_key:
                self.last_key = key
                event(message)

    def watch(self):
        event("모니터 시작. 최초 상태와 상태/사유 변경 시에만 출력합니다.")
        event("Safety가 실행되지 않았다면 첫 상태를 기다립니다.")
        while not rospy.is_shutdown():
            time.sleep(0.25)
            with self.lock:
                if time.monotonic() - self.last_receive > STALE_AFTER_S:
                    if self.last_key != ("NO_STATUS",):
                        self.last_key = ("NO_STATUS",)
                        event("NO_STATUS | Safety 상태 수신 중단 또는 미실행")


def main():
    rospy.init_node("pid_safety_event_watch", anonymous=True, disable_signals=True)
    watcher = SafetyWatch()
    try:
        watcher.watch()
    except KeyboardInterrupt:
        event("표시만 종료합니다. FF-PD/READY는 종료하지 않습니다.")


if __name__ == "__main__":
    main()

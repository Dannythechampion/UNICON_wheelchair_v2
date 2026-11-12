#!/usr/bin/env python3
"""Final-stage command relay: forwards /cmd_vel_gated to /cmd_vel through a
rate limiter, and stops on a stale command stream.

Sits as the LAST node before the wheel base (safety_gate -> tip_guard ->
wheel_cmd_tmp.py). Tip-over detection/prevention (predictive pitch-rate
trip against /livox/imu, fused-pitch release logic, the correlation
self-check and its accel governor) was removed: on this deployment's
terrain the trip trigger fired on ordinary pavement bumps (a bump and a
real tip produce the same brief pitch-rate spike, and ordinary bumps are
far more frequent), and a false stop was judged more dangerous than no
dynamic tip check at all. This node keeps its name and position in the
chain because downstream code authorizes /cmd_vel only from caller
"/tip_guard".
"""

import os
import sys

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, String
# catkin_install_python leaves a relay in devel/lib that exec()s this file,
# so sys.path[0] is the relay's directory, not this one, and the policy
# modules sitting beside this file are not importable - the relay does set
# __file__ to this source path, so recover the directory from it. Without
# this the node dies at import on the vehicle while every offline test,
# which imports the modules directly, still passes.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tip_guard_policy import next_linear_speed, no_reverse_yaw, resume_speed

GUARD_HZ = 50.0
INPUT_STALE_S = 0.6
ACCEL_LIMIT = 0.30
# Where the loaded wheels start turning (dwa_core.TURN_FLOOR_SPEED, kept as
# a literal for the same reason safety_gate keeps its own copy). A launch
# from rest goes straight here instead of crawling up through the deadband;
# ~launch_floor_mps 0.0 restores the plain ramp.
LAUNCH_FLOOR_MPS = 0.35
# The inner wheel never reverses (tip_guard_policy.no_reverse_yaw); the
# track width is the DWA stack's (wheel.launch wheel_separation_m 0.54).
# ~no_reverse false passes the yaw rate through untouched, as before.
WHEEL_SEPARATION_M = 0.54
# Resume from the wheels after a zero (tip_guard_policy.resume_speed);
# ~resume_from_wheels false restarts the ramp from 0 as before. The base
# report runs at 100 Hz, so 0.2 s is twenty frames; under 0.05 m/s (one
# count of the 0.1 km/h report) is rest. Same numbers as dwa_follower.
WHEEL_FRESH_S = 0.2
ROLLING_MPS = 0.05


class TipGuard:
    def __init__(self):
        rospy.init_node("tip_guard")

        self.raw = Twist()
        self.raw_stamp = rospy.Time(0)
        self.current_speed = 0.0
        self.status = ""
        self.launch_floor_mps = float(rospy.get_param(
            "~launch_floor_mps", LAUNCH_FLOOR_MPS))
        self.no_reverse = bool(rospy.get_param("~no_reverse", True))
        self.wheel_separation_m = float(rospy.get_param(
            "~wheel_separation_m", WHEEL_SEPARATION_M))
        if not 0.3 <= self.wheel_separation_m <= 0.8:
            raise ValueError("~wheel_separation_m %r outside 0.3-0.8 m"
                             % self.wheel_separation_m)
        self.clamped = False
        self.resume_from_wheels = bool(rospy.get_param(
            "~resume_from_wheels", True))
        self.wheel_speed = 0.0
        self.wheel_stamp = None
        self.resumed = False
        rospy.loginfo("tip_guard: accel %.2f m/s^2, launch floor %.2f m/s, "
                      "no-reverse yaw %s (track %.2f m), resume from wheels %s",
                      ACCEL_LIMIT, self.launch_floor_mps,
                      "on" if self.no_reverse else "off",
                      self.wheel_separation_m,
                      "on" if self.resume_from_wheels else "off")

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/tip_guard/status", String, queue_size=2)
        rospy.Subscriber("/cmd_vel_gated", Twist, self.on_raw, queue_size=1)
        if self.resume_from_wheels:
            rospy.Subscriber("/wheel_status", Int16MultiArray,
                             self.on_wheel_status, queue_size=5)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))

    # ------------------------------------------------------------ callbacks
    def on_raw(self, message):
        self.raw = message
        self.raw_stamp = rospy.Time.now()

    def on_wheel_status(self, message):
        """Mean forward wheel speed from the base report (C forward, W back)."""
        data = message.data
        if len(data) < 6:
            return

        def one(direction, magnitude):
            speed = (float(magnitude) - 33.0) / 10.0 / 3.6
            if int(direction) == 67:
                return speed
            if int(direction) == 87:
                return -speed
            return 0.0
        self.wheel_speed = 0.5 * (one(data[2], data[3]) + one(data[4], data[5]))
        self.wheel_stamp = rospy.Time.now()

    def rolling_speed(self, now):
        """The wheels' forward speed while fresh and turning, else None."""
        if self.wheel_stamp is None:
            return None
        age = (now - self.wheel_stamp).to_sec()
        if age < 0.0 or age > WHEEL_FRESH_S or self.wheel_speed <= ROLLING_MPS:
            return None
        return self.wheel_speed

    # ------------------------------------------------------------ logic
    def spin(self):
        rate = rospy.Rate(GUARD_HZ)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(1e-3, (now - last).to_sec())
            last = now

            stale = (now - self.raw_stamp).to_sec() > INPUT_STALE_S

            self.resumed = False
            if self.resume_from_wheels and not stale:
                resumed = resume_speed(self.current_speed,
                                       self.raw.linear.x,
                                       self.rolling_speed(now))
                self.resumed = resumed > self.current_speed
                self.current_speed = resumed

            self.current_speed = next_linear_speed(
                self.current_speed,
                self.raw.linear.x,
                ACCEL_LIMIT,
                dt,
                launch_floor=self.launch_floor_mps,
                stop_required=stale)

            out = Twist()
            out.linear.x = self.current_speed
            out.angular.z = 0.0 if stale else self.raw.angular.z
            self.clamped = False
            if self.no_reverse and not stale:
                limited = no_reverse_yaw(out.linear.x, out.angular.z,
                                         self.wheel_separation_m)
                self.clamped = abs(limited - out.angular.z) > 1e-6
                out.angular.z = limited
            self.pub.publish(out)

            state = "STALE" if stale else "OK"
            self.status_pub.publish(String(
                data="%s v=%.2f%s%s" % (
                    state, self.current_speed,
                    " RESUMED" if self.resumed else "",
                    " W_CLAMPED %+.2f->%+.2f" % (self.raw.angular.z,
                                                 out.angular.z)
                    if self.clamped else "")))
            if state != self.status:
                rospy.loginfo("tip_guard: %s", state)
                self.status = state
            rate.sleep()


if __name__ == "__main__":
    TipGuard().spin()

#!/usr/bin/env python3
"""Watches whether a commanded stop actually reaches the wheels and guards
against uncommanded spins in AUTO mode using 200 Hz IMU feedback.

On 2026-08-19 the chair was told to stop, kept the left wheel at 0.72 m/s for
12.97 seconds, and turned about 2.9 times on the spot at the finish line.
On 2026-11-16, 11-18, and 11-19, the chair also experienced sudden 159-212 deg
in-place spins (136-168 deg/s on /livox/imu) during AUTO mode while wheel_cmd
was still issuing drive commands.

This node watches three things in AUTO mode:
  1. Commanded stops (from either /wheel_cmd S/S or /cmd_vel == 0): verifies
     that the wheels actually decelerate, cross-checking /livox/imu so
     stationary single-wheel ghost speeds and brief normal brake lag do not
     trigger false alarms.
  2. Uncommanded spins (200 Hz /livox/imu vs /cmd_vel angular.z): catches
     runaway rotation during BOTH driving and stopping inside ~0.15 s.
  3. Alarm lifecycle: latches /stop_watchdog/alarm for diagnostics, switches
     mode_cmd to MANUAL (77) without auto-resuming, and exposes
     /stop_watchdog/clear (std_srvs/Trigger) so the next operator-initiated
     drive can clear a reviewed alarm without restarting the READY stack.

Decoding, from base_model/src/wheel_cmd_tmp.py and uart.py:
  wheel_cmd     [ll, lv, rr, rv, brk]      direction C forward, W back, S stop
  wheel_status  [72, mode, ll, lv, rr, rv, brk, battery, checksum, 13, 10]
  speed         (byte - 0x21) / 10 / 3.6 m/s
"""

import json
import math
import threading

try:
    import rospy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Int16, Int16MultiArray, String
    from std_srvs.srv import Trigger, TriggerResponse
except ImportError:  # Pure-Python unit tests and offline bag replay
    rospy = None
    Twist = None
    Imu = None
    Int16 = None
    Int16MultiArray = None
    String = None
    Trigger = None
    TriggerResponse = None


AUTO_MODE = 65
MANUAL_MODE = 77
STOP_DIRECTION = ord("S")

# Deceleration envelope floor once a stop has been issued.
STOP_DECEL_FLOOR_MPS2 = 0.15
# Grace window once S/S is on the wire.
GRACE_S = 0.5
# Grace window when armed early by /cmd_vel == 0 before S/S hits the wire
# (accounts for ~0.35-0.55 s base transport latency and 0.028 m/s quantization).
CMD_VEL_STOP_GRACE_S = 1.00
CMD_VEL_STOP_LATENCY_S = 0.55
# Still rolling this long after a stop is a fault whatever the envelope says.
CEILING_S = 5.0
# Speed increase above stop-entry speed is never inertia.
RISE_MARGIN_MPS = 0.06
# One wheel stopped while the other carries >= SPLIT_MPS is a split.
SPLIT_MPS = 0.35
# Immediate split confirmation when IMU confirms real rotation (or IMU absent).
SPLIT_HOLD_S = 0.15
# Longer confirmation when fresh IMU confirms the chair is NOT rotating fast
# (prevents false alarms on 0.16 s normal brake cutoff stagger).
SPLIT_NO_IMU_ROT_HOLD_S = 0.50
SPLIT_IMU_ROT_DEG_S = 25.0
# Below this the report is noise / slow terminal roll-out during a normal stop.
# Both wheels persistently rolling above BOTH_ROLLING_MPS past ceiling_s (5 s)
# is still caught so low-speed creep cannot roll indefinitely.
MOVING_MPS = 0.22
BOTH_ROLLING_MPS = 0.15
# Stationary single-wheel ghost filter: if one wheel is <= 0.06 m/s, the other
# is <= 0.35 m/s, and fresh IMU shows < 8 deg/s (physically impossible if the
# reported wheel were truly spinning at 0.15-0.35 m/s), treat as sensor ghost.
GHOST_STILL_WHEEL_MPS = 0.06
GHOST_MAX_SINGLE_WHEEL_MPS = 0.35
GHOST_MAX_IMU_DEG_S = 8.0
# A command older than this says nothing about now.
COMMAND_FRESH_S = 0.30
# Freshness windows for /livox/imu and /cmd_vel.
IMU_FRESH_S = 0.20
IMU_MAX_GAP_S = 0.05
CMD_VEL_FRESH_S = 0.50
STATUS_FRESH_S = 0.50
# 200 Hz IMU spin guard thresholds (active during both driving and stopping).
# Tuned on 90 real Nov 12-19 drives: harmless road/curb transients peak at
# 51-62 deg/s for ~0.15 s; all 3 real spins exceed 60+ deg/s divergence and
# accelerate past 70 -> 140+ deg/s monotonically.
IMU_SPIN_DIFF_DEG_S = 60.0
IMU_SPIN_MIN_WZ_DEG_S = 55.0
IMU_SPIN_ABS_WZ_DEG_S = 70.0
IMU_SPIN_HOLD_S = 0.20
IMU_SPIN_REARM_WZ_DEG_S = 15.0
IMU_SPIN_REARM_HOLD_S = 0.50
CMD_VEL_STOP_EPS = 0.01
IMU_TOPIC_DEFAULT = "/livox/imu"


def wheel_speed(byte_value):
    return (float(byte_value) - 0x21) / 10.0 / 3.6


def commanded_stop(data):
    """True when the frame on the wire tells both wheels to stop."""
    return len(data) >= 3 and \
        int(data[0]) == STOP_DIRECTION and int(data[2]) == STOP_DIRECTION


def reported_motion(data):
    """(left, right) speed in m/s from a status frame, or None if malformed."""
    if len(data) < 6:
        return None
    return (abs(wheel_speed(data[3])), abs(wheel_speed(data[5])))


class StopHonouredCheck:
    """Stop-honoured decision with optional /cmd_vel early arming and IMU
    cross-checking. Pure Python (no ROS dependency) for unit tests & replay.
    """

    def __init__(self, grace_s=GRACE_S, moving_mps=MOVING_MPS,
                 command_fresh_s=COMMAND_FRESH_S,
                 decel_floor_mps2=STOP_DECEL_FLOOR_MPS2,
                 ceiling_s=CEILING_S, rise_margin_mps=RISE_MARGIN_MPS,
                 split_mps=SPLIT_MPS, split_hold_s=SPLIT_HOLD_S,
                 split_no_imu_rot_hold_s=SPLIT_NO_IMU_ROT_HOLD_S,
                 split_imu_rot_deg_s=SPLIT_IMU_ROT_DEG_S,
                 cmd_vel_stop_grace_s=CMD_VEL_STOP_GRACE_S,
                 imu_fresh_s=IMU_FRESH_S):
        self.grace_s = float(grace_s)
        self.cmd_vel_stop_grace_s = float(cmd_vel_stop_grace_s)
        self.moving_mps = float(moving_mps)
        self.command_fresh_s = float(command_fresh_s)
        self.decel_floor_mps2 = float(decel_floor_mps2)
        self.ceiling_s = float(ceiling_s)
        self.rise_margin_mps = float(rise_margin_mps)
        self.split_mps = float(split_mps)
        self.split_hold_s = float(split_hold_s)
        self.split_no_imu_rot_hold_s = float(split_no_imu_rot_hold_s)
        self.split_imu_rot_rad_s = math.radians(float(split_imu_rot_deg_s))
        self.imu_fresh_s = float(imu_fresh_s)
        self.ghost_max_imu_rad_s = math.radians(GHOST_MAX_IMU_DEG_S)
        self.reset_state()

    def reset_state(self):
        self.split_since = None
        self.stop_since = None
        self.wire_stop_since = None
        self.entry_speed = None
        self.latched = False
        self.last_command_s = 0.0
        self.last_cmd_vel_stop_s = None
        self.imu_wz_rad_s = None
        self.imu_stamp_s = None

    def reset_latch(self):
        self.split_since = None
        self.stop_since = None
        self.wire_stop_since = None
        self.entry_speed = None
        self.latched = False

    def observe_imu(self, wz_rad_s, stamp_s):
        try:
            wz = float(wz_rad_s)
            ts = float(stamp_s)
        except (TypeError, ValueError):
            return
        if not (math.isfinite(wz) and math.isfinite(ts)):
            return
        if self.imu_stamp_s is not None and ts < self.imu_stamp_s:
            return
        self.imu_wz_rad_s = wz
        self.imu_stamp_s = ts

    def _fresh_imu_wz(self, now_s):
        if self.imu_stamp_s is None or self.imu_wz_rad_s is None:
            return None
        age = float(now_s) - self.imu_stamp_s
        if 0.0 <= age <= self.imu_fresh_s:
            return self.imu_wz_rad_s
        return None

    def observe_cmd_vel(self, linear_x, angular_z, now_s):
        """Arm stop watching as soon as /cmd_vel commands zero, closing any
        blind window while wheel_cmd is ramping down before sending S/S."""
        try:
            vx = float(linear_x)
            wz = float(angular_z)
            now = float(now_s)
        except (TypeError, ValueError):
            return
        if not (math.isfinite(vx) and math.isfinite(wz) and math.isfinite(now)):
            return
        if abs(vx) < CMD_VEL_STOP_EPS and abs(wz) < CMD_VEL_STOP_EPS:
            self.last_cmd_vel_stop_s = now
            if self.stop_since is None:
                self.stop_since = now
            self.last_command_s = now
        else:
            self.last_cmd_vel_stop_s = None
            if self.wire_stop_since is None:
                self.stop_since = None
                self.entry_speed = None
                self.split_since = None
                self.latched = False

    def observe_command(self, data, now_s):
        now = float(now_s)
        if commanded_stop(data):
            if self.wire_stop_since is None:
                self.wire_stop_since = now
            if self.stop_since is None:
                self.stop_since = now
        else:
            self.wire_stop_since = None
            cmd_vel_stop_fresh = (
                self.last_cmd_vel_stop_s is not None and
                0.0 <= now - self.last_cmd_vel_stop_s <= self.command_fresh_s
            )
            if not cmd_vel_stop_fresh:
                self.stop_since = None
                self.entry_speed = None
                self.split_since = None
                self.latched = False
        self.last_command_s = now

    def observe_status(self, data, now_s, mode):
        """Returns a reason when the stop has not been honoured, else None."""
        motion = reported_motion(data)
        if motion is None:
            return None
        left, right = motion
        now = float(now_s)
        if int(mode) != AUTO_MODE or self.stop_since is None:
            return None
        if now - self.last_command_s > self.command_fresh_s:
            # The command stream has stopped; uart.py's own watchdog owns it.
            return None
        speed = max(left, right)
        if self.entry_speed is None:
            self.entry_speed = speed
        held = now - self.stop_since
        if speed <= self.moving_mps:
            # Both wheels still rolling above BOTH_ROLLING_MPS past ceiling_s is a fault
            # even though brief terminal speeds (0.14-0.19 m/s) during normal stops are fine.
            if (min(left, right) > BOTH_ROLLING_MPS and
                    held >= self.ceiling_s and not self.latched):
                self.latched = True
                return ("still rolling %.2f s after a stop (left %.2f m/s, right %.2f m/s)"
                        % (held, left, right))
            # Honoured. Arm again for the next stop.
            self.split_since = None
            if min(left, right) <= BOTH_ROLLING_MPS:
                self.latched = False
            return None

        imu_wz = self._fresh_imu_wz(now)
        # Filter physically impossible stationary single-wheel ghost speeds:
        # one wheel at 0, the other <= 0.35 m/s, yet IMU shows < 8 deg/s.
        if (imu_wz is not None and
                min(left, right) <= GHOST_STILL_WHEEL_MPS and
                speed <= GHOST_MAX_SINGLE_WHEEL_MPS and
                abs(imu_wz) < self.ghost_max_imu_rad_s):
            self.split_since = None
            return None

        held = now - self.stop_since
        # Split check: one wheel stopped while the other still carries >= SPLIT_MPS.
        # When fresh IMU is available, require real rotation (>= 25 deg/s) for the
        # fast 0.15 s trip; otherwise wait split_no_imu_rot_hold_s (0.50 s) so
        # brief normal left/right brake cutoff stagger does not false-alarm.
        if min(left, right) <= self.moving_mps and speed >= self.split_mps:
            if self.split_since is None:
                self.split_since = now
            imu_confirms_rot = (
                imu_wz is None or abs(imu_wz) >= self.split_imu_rot_rad_s
            )
            required_hold = (
                self.split_hold_s if imu_confirms_rot
                else self.split_no_imu_rot_hold_s
            )
            if now - self.split_since >= required_hold and not self.latched:
                self.latched = True
                if imu_wz is not None:
                    return (
                        "one wheel ignored a stop: left %.2f m/s, right "
                        "%.2f m/s, %.0f deg/s of pivot (imu %.1f deg/s)"
                        % (left, right, abs(left - right) / 0.54 * 57.3,
                           math.degrees(imu_wz)))
                return ("one wheel ignored a stop: left %.2f m/s, right "
                        "%.2f m/s, %.0f deg/s of pivot"
                        % (left, right, abs(left - right) / 0.54 * 57.3))
        else:
            self.split_since = None

        active_grace = (
            self.grace_s if self.wire_stop_since is not None
            else self.cmd_vel_stop_grace_s
        )
        if held < active_grace or self.latched:
            return None

        # When armed early by /cmd_vel==0 before S/S hits the wire, shift the
        # deceleration envelope by ~0.35 s of base transport latency so normal
        # decelerations are never tripped prematurely.
        effective_decel_t = (
            held if self.wire_stop_since is not None
            else max(0.0, held - CMD_VEL_STOP_LATENCY_S)
        )
        allowed = self.entry_speed - self.decel_floor_mps2 * effective_decel_t
        if speed > self.entry_speed + self.rise_margin_mps:
            fault = ("wheels sped up after a stop: %.2f m/s against %.2f "
                     "when it was issued" % (speed, self.entry_speed))
        elif held >= self.ceiling_s:
            fault = "still rolling %.2f s after a stop" % held
        elif speed > max(self.moving_mps, allowed):
            fault = ("not slowing after a stop: %.2f m/s at %.2f s, "
                     "%.2f expected" % (speed, held, allowed))
        else:
            return None
        self.latched = True
        return ("%s (left %.2f m/s, right %.2f m/s)"
                % (fault, left, right))


class ImuSpinCheck:
    """200 Hz IMU uncommanded spin detector active during BOTH driving and
    stopping in AUTO mode.

    Measures sustain duration strictly across distinct IMU sample timestamps so
    a single stale frame can never trigger a false alarm.
    """

    def __init__(self,
                 diff_deg_s=IMU_SPIN_DIFF_DEG_S,
                 min_wz_deg_s=IMU_SPIN_MIN_WZ_DEG_S,
                 abs_wz_deg_s=IMU_SPIN_ABS_WZ_DEG_S,
                 hold_s=IMU_SPIN_HOLD_S,
                 max_gap_s=IMU_MAX_GAP_S,
                 cmd_vel_fresh_s=CMD_VEL_FRESH_S,
                 rearm_wz_deg_s=IMU_SPIN_REARM_WZ_DEG_S,
                 rearm_hold_s=IMU_SPIN_REARM_HOLD_S):
        self.diff_rad_s = math.radians(float(diff_deg_s))
        self.min_wz_rad_s = math.radians(float(min_wz_deg_s))
        self.abs_wz_rad_s = math.radians(float(abs_wz_deg_s))
        self.hold_s = float(hold_s)
        self.max_gap_s = float(max_gap_s)
        self.cmd_vel_fresh_s = float(cmd_vel_fresh_s)
        self.rearm_wz_rad_s = math.radians(float(rearm_wz_deg_s))
        self.rearm_hold_s = float(rearm_hold_s)
        self.reset()

    def reset(self):
        self.cmd_w_rad_s = 0.0
        self.cmd_w_stamp_s = None
        self.last_imu_stamp_s = None
        self.violation_since = None
        self.violation_samples = 0
        self.calm_since = None
        self.latched = False

    def observe_cmd_vel(self, angular_z, now_s):
        try:
            wz = float(angular_z)
            now = float(now_s)
        except (TypeError, ValueError):
            return
        if math.isfinite(wz) and math.isfinite(now):
            self.cmd_w_rad_s = wz
            self.cmd_w_stamp_s = now

    def _active_cmd_w(self, now_s):
        if self.cmd_w_stamp_s is None:
            return 0.0
        age = float(now_s) - self.cmd_w_stamp_s
        if 0.0 <= age <= self.cmd_vel_fresh_s:
            return self.cmd_w_rad_s
        return 0.0

    def observe_imu(self, wz_rad_s, stamp_s, mode, cmd_now_s=None):
        """Feed one IMU sample. Returns reason string on trip, else None."""
        try:
            wz = float(wz_rad_s)
            ts = float(stamp_s)
        except (TypeError, ValueError):
            return None
        if not (math.isfinite(wz) and math.isfinite(ts)):
            return None

        # Require strictly advancing IMU sample timestamps (no stale reuse) and
        # reset sustain timer if the IMU stream dropped out for > max_gap_s.
        if self.last_imu_stamp_s is not None:
            if ts <= self.last_imu_stamp_s:
                return None
            if ts - self.last_imu_stamp_s > self.max_gap_s:
                self.violation_since = None
                self.violation_samples = 0
                self.calm_since = None
        self.last_imu_stamp_s = ts

        if mode is None or int(mode) != AUTO_MODE:
            self.violation_since = None
            self.violation_samples = 0
            self.calm_since = None
            self.latched = False
            return None

        if self.latched:
            if abs(wz) < self.rearm_wz_rad_s:
                if self.calm_since is None:
                    self.calm_since = ts
                elif ts - self.calm_since >= self.rearm_hold_s:
                    self.latched = False
                    self.calm_since = None
            else:
                self.calm_since = None
            return None

        cmd_w = self._active_cmd_w(ts if cmd_now_s is None else cmd_now_s)
        abs_wz = abs(wz)
        diverged = (abs(wz - cmd_w) >= self.diff_rad_s and
                    abs_wz >= self.min_wz_rad_s)
        overspin = abs_wz >= self.abs_wz_rad_s

        if diverged or overspin:
            if self.violation_since is None:
                self.violation_since = ts
                self.violation_samples = 1
            else:
                self.violation_samples += 1
            held = ts - self.violation_since
            if self.violation_samples >= 2 and held >= self.hold_s:
                self.latched = True
                self.violation_since = None
                self.violation_samples = 0
                return (
                    "uncommanded spin in AUTO: imu %.1f deg/s vs cmd %.1f "
                    "deg/s for %.2f s"
                    % (math.degrees(wz), math.degrees(cmd_w), held))
        else:
            self.violation_since = None
            self.violation_samples = 0
        return None


class StopWatchdog:
    def __init__(self):
        if rospy is None:
            raise RuntimeError("rospy is required to run StopWatchdog node")
        rospy.init_node("stop_watchdog")
        self.lock = threading.Lock()
        self.check = StopHonouredCheck(
            grace_s=float(rospy.get_param("~grace_s", GRACE_S)),
            moving_mps=float(rospy.get_param("~moving_mps", MOVING_MPS)),
            split_hold_s=float(rospy.get_param("~split_hold_s", SPLIT_HOLD_S)),
            split_no_imu_rot_hold_s=float(rospy.get_param(
                "~split_no_imu_rot_hold_s", SPLIT_NO_IMU_ROT_HOLD_S)),
            split_imu_rot_deg_s=float(rospy.get_param(
                "~split_imu_rot_deg_s", SPLIT_IMU_ROT_DEG_S)),
            cmd_vel_stop_grace_s=float(rospy.get_param(
                "~cmd_vel_stop_grace_s", CMD_VEL_STOP_GRACE_S)),
        )
        self.spin_check = ImuSpinCheck(
            diff_deg_s=float(rospy.get_param(
                "~imu_spin_diff_deg_s", IMU_SPIN_DIFF_DEG_S)),
            min_wz_deg_s=float(rospy.get_param(
                "~imu_spin_min_wz_deg_s", IMU_SPIN_MIN_WZ_DEG_S)),
            abs_wz_deg_s=float(rospy.get_param(
                "~imu_spin_abs_wz_deg_s", IMU_SPIN_ABS_WZ_DEG_S)),
            hold_s=float(rospy.get_param(
                "~imu_spin_hold_s", IMU_SPIN_HOLD_S)),
        )
        self.attempt_stop = bool(rospy.get_param("~attempt_stop", True))
        self.imu_topic = str(rospy.get_param(
            "~imu_topic", IMU_TOPIC_DEFAULT)).strip()
        self.tx_diag = {}
        self.current_mode = None
        self.current_mode_stamp = None
        self.last_motion = (0.0, 0.0)
        self.active_alarm = None

        self.alarm_pub = rospy.Publisher(
            "stop_watchdog/alarm", String, queue_size=4, latch=True)
        self.mode_pub = rospy.Publisher("mode_cmd", Int16, queue_size=1)
        self.clear_srv = rospy.Service(
            "stop_watchdog/clear", Trigger, self.on_clear)

        rospy.Subscriber("wheel_cmd", Int16MultiArray, self.on_command)
        rospy.Subscriber("wheel_status", Int16MultiArray, self.on_status)
        rospy.Subscriber("cmd_vel", Twist, self.on_cmd_vel)
        rospy.Subscriber("uart_tx_diag", String, self.on_tx_diag)
        if self.imu_topic:
            rospy.Subscriber(self.imu_topic, Imu, self.on_imu, queue_size=20)
        rospy.loginfo(
            "stop watchdog: %.2f s grace, %.2f m/s floor, imu=%s "
            "(spin diff %.0f / abs %.0f deg/s for %.2f s), stop attempt %s",
            self.check.grace_s, self.check.moving_mps,
            self.imu_topic or "off",
            math.degrees(self.spin_check.diff_rad_s),
            math.degrees(self.spin_check.abs_wz_rad_s),
            self.spin_check.hold_s,
            "on" if self.attempt_stop else "off")

    def _emit_alarm(self, reason, now_s, motion, source="stop_honoured",
                    imu_wz_rad_s=None):
        alarm = {
            "alarm": True,
            "cleared": False,
            "source": source,
            "stamp": now_s,
            "reason": reason,
            "left_mps": round(motion[0], 3),
            "right_mps": round(motion[1], 3),
            "uart_tx": dict(self.tx_diag),
        }
        if imu_wz_rad_s is not None and math.isfinite(imu_wz_rad_s):
            alarm["imu_wz_deg_s"] = round(math.degrees(imu_wz_rad_s), 2)
        self.active_alarm = alarm
        rospy.logerr("WATCHDOG ALARM [%s] - %s | uart tx %s",
                     source, reason, json.dumps(self.tx_diag))
        self.alarm_pub.publish(String(data=json.dumps(alarm)))
        if self.attempt_stop:
            self.mode_pub.publish(Int16(data=MANUAL_MODE))

    def on_clear(self, _req):
        now = rospy.Time.now().to_sec()
        with self.lock:
            if (self.current_mode_stamp is None or
                    not (0.0 <= now - self.current_mode_stamp <= STATUS_FRESH_S)):
                return TriggerResponse(
                    success=False,
                    message="refused: wheel_status unavailable or stale")
            if self.current_mode != MANUAL_MODE:
                return TriggerResponse(
                    success=False,
                    message="refused: chair must be in MANUAL mode to clear alarm")
            if max(self.last_motion) > GHOST_STILL_WHEEL_MPS:
                return TriggerResponse(
                    success=False,
                    message="refused: wheels not stopped (L=%.2f, R=%.2f m/s)"
                    % self.last_motion)
            prev = self.active_alarm
            self.active_alarm = None
            self.check.reset_latch()
            self.spin_check.reset()
            if prev is None:
                return TriggerResponse(success=True, message="no active alarm")
            prev_reason = str(prev.get("reason", ""))
            payload = {
                "alarm": False,
                "cleared": True,
                "stamp": now,
                "previous_reason": prev_reason,
            }
            rospy.logwarn("stop watchdog alarm cleared (was: %s)", prev_reason)
            self.alarm_pub.publish(String(data=json.dumps(payload)))
        return TriggerResponse(
            success=True,
            message="cleared previous alarm: %s" % prev_reason)

    def on_tx_diag(self, message):
        try:
            parsed = json.loads(message.data)
        except ValueError:
            return
        with self.lock:
            self.tx_diag = parsed

    def on_cmd_vel(self, message):
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.check.observe_cmd_vel(
                message.linear.x, message.angular.z, now)
            self.spin_check.observe_cmd_vel(message.angular.z, now)

    def on_command(self, message):
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.check.observe_command(list(message.data), now)

    def on_imu(self, message):
        try:
            wz = float(message.angular_velocity.z)
        except (AttributeError, TypeError, ValueError):
            return
        if not math.isfinite(wz):
            return
        rx_now = rospy.Time.now().to_sec()
        sensor_ts = rx_now
        try:
            hdr_ts = float(message.header.stamp.to_sec())
            if math.isfinite(hdr_ts) and hdr_ts > 0.0:
                sensor_ts = hdr_ts
        except (AttributeError, TypeError, ValueError):
            pass
        with self.lock:
            self.check.observe_imu(wz, rx_now)
            mode = None
            if (self.current_mode_stamp is not None and
                    0.0 <= rx_now - self.current_mode_stamp <= STATUS_FRESH_S):
                mode = self.current_mode
            reason = self.spin_check.observe_imu(
                wz, sensor_ts, mode, cmd_now_s=rx_now)
            motion = self.last_motion
            if reason is not None:
                self._emit_alarm(
                    reason, rx_now, motion, source="imu_spin", imu_wz_rad_s=wz)

    def on_status(self, message):
        data = list(message.data)
        if len(data) < 2:
            return
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.current_mode = int(data[1])
            self.current_mode_stamp = now
            motion = reported_motion(data)
            if motion is not None:
                self.last_motion = motion
            reason = self.check.observe_status(data, now, data[1])
            if reason is not None:
                imu_wz = self.check._fresh_imu_wz(now)
                self._emit_alarm(
                    reason, now, motion or (0.0, 0.0),
                    source="stop_honoured", imu_wz_rad_s=imu_wz)


if __name__ == "__main__":
    StopWatchdog()
    rospy.spin()

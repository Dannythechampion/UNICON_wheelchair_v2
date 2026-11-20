#!/usr/bin/env python3

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16MultiArray


AUTO_MODE = 65
EXPECTED_STATUS_CALLER = "/uart"
MAX_LINEAR_MPS = 1.5
MAX_ANGULAR_RAD_S = 0.6
STOP_COMMAND = (83, 33, 83, 33, 79)

# Per-stack settings. Two controllers share this one node: the DWA stack
# (/tip_guard, start_wheelchair_localization.sh) and the feed-forward PID
# stack (/ff_pid, pid_ready.sh). Each sets its own values as wheel.launch
# args, which roslaunch writes as this node's private params on EVERY
# launch, so one stack's choice can never linger into the other's run.
# The defaults below are what the PID stack has used since 2026-11-06 and
# what a plain `roslaunch base_model wheel.launch` gets.
#   ~wheel_separation_m  track width used to split yaw into wheel speeds
#   ~turn_boost          add equal speed to both wheels so a slow turn
#                        breaks stiction (the 2026-07-29 fix; off = none)
#   ~cmd_callers         comma-separated /cmd_vel publishers allowed to
#                        drive; any other publisher latches a stop
WHEEL_SEPARATION_M = 0.46
TURN_BOOST = False
EXPECTED_CMD_CALLERS = ("/tip_guard", "/ff_pid")
WHEEL_SEPARATION_RANGE_M = (0.30, 0.80)

# Wheel speeds are sent as tenths of a km/h on top of a 33 offset, so one
# count is 0.1 km/h (0.028 m/s).
COUNTS_PER_KMH = 10.0
MAGNITUDE_OFFSET = 33

# S is a stopped state on this base, not a brake command. A stop request
# therefore ramps from measured wheel speeds and changes each wheel to S
# only after that wheel is already close to rest.
RAMP_DECAY = 0.9
RAMP_BLEED_MPS = 0.09
RAMP_TERMINAL_MPS = 0.06
STATUS_FRESH_S = 0.30

# Legacy: 기존 DWA 정지 방식
# Fast: FF-PID 전용 실측 기반 2단계 Stop Ramp (커브 시 0.20 정렬 -> 0.03 감속 -> 0.15 이하 시 S)
STOP_RAMP_MODE = "legacy"
FAST_STOP_PERIOD_S = 0.02  # 50Hz
FAST_STOP_ALIGN_MPS = 0.20
FAST_STOP_ALIGN_DIFF_MPS = 0.08
FAST_STOP_ALIGN_MAX_S = 1.0
FAST_STOP_FINAL_MPS = 0.03
FAST_STOP_S_MPS = 0.15
FAST_STOP_MAX_S = 1.5
FAST_STOP_IMU_TOPIC = "/livox/imu"
FAST_STOP_IMU_FRESH_S = 0.20
FAST_STOP_IMU_MAX_GAP_S = 0.08
FAST_STOP_IMU_ABORT_DEG_S = 50.0
FAST_STOP_IMU_ABORT_HOLD_S = 0.10

# Below this the wheels were observed not to turn the loaded chair at all.
# On 2026-07-29 the follower held its maximum +0.5 rad/s for four seconds
# while the faster wheel sat at 0.9 km/h and the chair rotated at 0.03-0.05
# rad/s - effectively not at all - then rotated at 0.57 rad/s once the
# wheels reached 1.3-1.6 km/h. The differential itself was correct
# throughout: 0.9 km/h across 0.54 m is the 0.46 rad/s that was asked for.
# What was missing was enough absolute speed to break stiction with a rider
# aboard, so the encoded difference described a turn the base never made.
TURN_AUTHORITY_KMH = 1.3
# Reaching that floor means driving both wheels faster, which adds forward
# speed the planner did not ask for. That is capped hard: enough to turn,
# never enough to run away.
TURN_AUTHORITY_MAX_LINEAR_MPS = 0.30
YAW_DEADBAND_RAD_S = 0.05


def wheel_speed_mps(direction, magnitude):
    """Decode one wheel from a status frame, negative when reversing."""
    try:
        speed = (float(magnitude) - MAGNITUDE_OFFSET) / COUNTS_PER_KMH / 3.6
        direction = int(direction)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(speed):
        return 0.0
    if direction == 67:
        return speed
    if direction == 87:
        return -speed
    return 0.0


def stop_ramp_command(left_mps, right_mps):
    """Build a straight measured-speed braking step for both wheels.

    Decaying each wheel independently preserves whatever yaw moment existed
    when the stop arrived. The 2026-08-25 field stop entered at C55/C51;
    that ramp reached left-forward/right-stop and rotated the chair another
    43 degrees toward a wall after the gate had already requested zero.

    Collapse to the slower wheel first, then decay both together. No wheel
    is accelerated, linear braking stays gradual, and commanded yaw becomes
    zero in the first stop frame. A pivot or one-wheel motion stops
    immediately because there is no non-zero straight speed common to both.
    """
    try:
        left = float(left_mps)
        right = float(right_mps)
    except (TypeError, ValueError):
        return STOP_COMMAND
    if not math.isfinite(left) or not math.isfinite(right) or \
            left == 0.0 or right == 0.0 or left * right < 0.0:
        return STOP_COMMAND
    target = min(abs(left), abs(right)) * RAMP_DECAY - RAMP_BLEED_MPS
    if target < RAMP_TERMINAL_MPS:
        return STOP_COMMAND
    direction = 67 if left > 0.0 else 87
    magnitude = int(round(target * 3.6 * COUNTS_PER_KMH)) + \
        MAGNITUDE_OFFSET
    magnitude = min(magnitude, 127)
    return direction, magnitude, direction, magnitude, 79

class FastStopRamp:
    """FF-PID 전용 실측 기반 2단계 Stop Ramp.

    1단계(align): 실측 좌우 바퀴를 align_mps(0.20 m/s) 이하로 수렴·정렬
                  (느린 바퀴는 절대 가속하지 않음: min(align_mps, left, right)).
    2단계(final): 정렬 완료 후 양쪽에 final_mps(0.03 m/s)를 보내 실측 양쪽 모두
                  s_threshold_mps(0.15 m/s) 이하가 되면 S 송신.

    한쪽이라도 이미 정지/역회전(<= 0.06 m/s)이거나, 양쪽 이미 <= 0.15 m/s 이거나,
    IMU 급회전 감지 또는 전체 타임아웃(max_duration_s) 시 즉시 S로 종료.
    """

    def __init__(self,
                 align_mps=FAST_STOP_ALIGN_MPS,
                 align_diff_mps=FAST_STOP_ALIGN_DIFF_MPS,
                 align_max_s=FAST_STOP_ALIGN_MAX_S,
                 final_mps=FAST_STOP_FINAL_MPS,
                 s_threshold_mps=FAST_STOP_S_MPS,
                 max_duration_s=FAST_STOP_MAX_S,
                 imu_abort_deg_s=FAST_STOP_IMU_ABORT_DEG_S,
                 imu_abort_hold_s=FAST_STOP_IMU_ABORT_HOLD_S):
        self.align_mps = float(align_mps)
        self.align_diff_mps = float(align_diff_mps)
        self.align_max_s = float(align_max_s)
        self.final_mps = float(final_mps)
        self.s_threshold_mps = float(s_threshold_mps)
        self.max_duration_s = float(max_duration_s)
        self.imu_abort_rad_s = math.radians(float(imu_abort_deg_s))
        self.imu_abort_hold_s = float(imu_abort_hold_s)
        self.reset()

    def reset(self):
        self.started_at = None
        self.stage = "align"
        self.imu_high_since = None
        self.last_imu_stamp = None
        self.done = False

    def abort(self):
        self.done = True

    def command(self, now, left, right, imu_wz_rad_s=None, imu_stamp=None):
        if self.done:
            return STOP_COMMAND

        if not all(math.isfinite(x) for x in (now, left, right)):
            return self.finish()

        # 한 바퀴라도 정지했거나 역회전/후진(음수)이면 즉시 S (멈춘 바퀴에 전진 명령 금지)
        if (left <= RAMP_TERMINAL_MPS or
                right <= RAMP_TERMINAL_MPS or
                max(left, right) > MAX_LINEAR_MPS):
            return self.finish()

        # 실측 양쪽 바퀴 모두 S 진입 기준(0.15 m/s) 이하이면 즉시 S
        if max(left, right) <= self.s_threshold_mps:
            return self.finish()

        # 정지 감속 중 IMU 급회전 감지 시 단계 무시하고 즉시 S
        # 서로 다른 IMU 샘플 시각(imu_stamp)으로 지속 시간을 판정해 1프레임 스파이크 오인 방지
        if (imu_wz_rad_s is not None and math.isfinite(imu_wz_rad_s) and
                self.imu_abort_rad_s > 0.0):
            sample_t = now if imu_stamp is None else float(imu_stamp)
            if math.isfinite(sample_t):
                if self.last_imu_stamp is None or sample_t > self.last_imu_stamp:
                    if (self.last_imu_stamp is not None and
                            sample_t - self.last_imu_stamp > FAST_STOP_IMU_MAX_GAP_S):
                        self.imu_high_since = None
                    self.last_imu_stamp = sample_t
                    if abs(imu_wz_rad_s) >= self.imu_abort_rad_s:
                        if self.imu_high_since is None:
                            self.imu_high_since = sample_t
                        if sample_t - self.imu_high_since >= self.imu_abort_hold_s:
                            return self.finish()
                    else:
                        self.imu_high_since = None
        else:
            self.imu_high_since = None

        if self.started_at is None:
            self.started_at = now
        elif now < self.started_at:
            return self.finish()

        elapsed = now - self.started_at
        if elapsed >= self.max_duration_s:
            return self.finish()

        # 1단계(align) -> 2단계(final) 전환 (단방향: 직진이거나 좌우 차가 0.08 이하면 바로 0.03 감속)
        if self.stage == "align":
            aligned = abs(left - right) <= self.align_diff_mps + 1e-9
            if aligned or elapsed >= self.align_max_s or \
                    min(left, right) <= self.final_mps:
                self.stage = "final"

        if self.stage == "align":
            # 느린 바퀴를 가속시키지 않으면서 양쪽을 align_mps(0.20)으로 수렴
            target = min(self.align_mps, left, right)
        else:
            # 최종 감속 목표(0.03), 실측보다 높은 명령은 보내지 않음
            target = min(self.final_mps, left, right)

        magnitude = (
            int(round(target * 3.6 * COUNTS_PER_KMH))
            + MAGNITUDE_OFFSET
        )
        magnitude = min(magnitude, 127)

        if magnitude <= MAGNITUDE_OFFSET:
            return self.finish()

        # 좌우 동일 전진(C=67) 속도 명령
        return 67, magnitude, 67, magnitude, 79

    def finish(self):
        self.done = True
        return STOP_COMMAND


def parse_bool(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off", ""):
        return False
    raise ValueError("not a boolean: %r" % (value,))


def parse_callers(value):
    if isinstance(value, (list, tuple)):
        items = value
    else:
        items = str(value).split(",")
    callers = tuple(sorted({str(item).strip() for item in items
                            if str(item).strip()}))
    if not callers or not all(c.startswith("/") for c in callers):
        raise ValueError("cmd_callers must list /node names: %r" % (value,))
    return callers


def load_settings(get_param):
    """(separation, boost, callers) from private params, refusing nonsense.

    A bad value raises rather than falls back: a base that silently drives
    on the other stack's geometry is the failure this exists to prevent.
    """
    separation = float(get_param("~wheel_separation_m", WHEEL_SEPARATION_M))
    low, high = WHEEL_SEPARATION_RANGE_M
    if not math.isfinite(separation) or not low <= separation <= high:
        raise ValueError("~wheel_separation_m %r outside %.2f-%.2f m"
                         % (separation, low, high))
    boost = parse_bool(get_param("~turn_boost", TURN_BOOST))
    callers = parse_callers(get_param("~cmd_callers",
                                      ",".join(EXPECTED_CMD_CALLERS)))
    return separation, boost, callers


def message_caller_id(message):
    header = getattr(message, "_connection_header", None)
    if not isinstance(header, dict):
        return ""
    return str(header.get("callerid", "")).strip()


def compute_wheel_command(linear_x, angular_z,
                          wheel_separation_m=WHEEL_SEPARATION_M,
                          turn_boost=TURN_BOOST):
    try:
        linear_x = float(linear_x)
        angular_z = float(angular_z)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(linear_x) or not math.isfinite(angular_z) or \
            linear_x < 0.0 or linear_x > MAX_LINEAR_MPS or \
            abs(angular_z) > MAX_ANGULAR_RAD_S:
        return None
    left = (linear_x - angular_z * wheel_separation_m / 2.0) * 3.6
    right = (linear_x + angular_z * wheel_separation_m / 2.0) * 3.6

    # Give a requested turn enough wheel speed to actually happen. Adding
    # the SAME amount to both wheels leaves right-left untouched, so the
    # yaw rate is exactly the one asked for; only the forward speed rises,
    # and only as far as TURN_AUTHORITY_MAX_LINEAR_MPS allows.
    if turn_boost and abs(angular_z) > YAW_DEADBAND_RAD_S:
        fastest = max(abs(left), abs(right))
        headroom = (TURN_AUTHORITY_MAX_LINEAR_MPS - linear_x) * 3.6
        boost = min(max(TURN_AUTHORITY_KMH - fastest, 0.0), max(headroom, 0.0))
        left += boost
        right += boost

    def encode(speed):
        direction = 67 if speed > 0.0 else (87 if speed < 0.0 else 83)
        # Round rather than truncate. Truncation drags every wheel toward
        # zero by up to a full count, which at these speeds is most of the
        # command: the 0.198 km/h inner wheel above became 1 count, 0.1
        # km/h, half of what was asked.
        magnitude = int(round(abs(speed) * COUNTS_PER_KMH)) + MAGNITUDE_OFFSET
        if magnitude > 127:
            return None
        return direction, magnitude

    left_encoded = encode(left)
    right_encoded = encode(right)
    if left_encoded is None or right_encoded is None:
        return None
    return (
        left_encoded[0],
        left_encoded[1],
        right_encoded[0],
        right_encoded[1],
        79,
    )


class WheelCommandGuard:
    def __init__(self):
        rospy.init_node("wheel_cmd")
        self.wheel_separation_m, self.turn_boost, self.cmd_callers = \
            load_settings(rospy.get_param)
        self.stop_ramp_mode = str(
            rospy.get_param("~stop_ramp_mode", STOP_RAMP_MODE)
        ).strip().lower()

        if self.stop_ramp_mode not in ("legacy", "fast"):
            raise rospy.ROSInitException("Invalid stop_ramp_mode")

        if self.stop_ramp_mode == "fast" and self.cmd_callers != ("/ff_pid",):
            raise rospy.ROSInitException(
                "fast Stop Ramp requires cmd_callers=/ff_pid"
            )
        rospy.loginfo(
            "wheel_cmd: wheel_separation_m %.3f, turn_boost %s, "
            "cmd_callers %s", self.wheel_separation_m,
            "on" if self.turn_boost else "off", ",".join(self.cmd_callers))
        self.publisher = rospy.Publisher(
            "wheel_cmd", Int16MultiArray, queue_size=1)
        self.mode = None
        self.fault_latched = False
        self.measured_left_mps = 0.0
        self.measured_right_mps = 0.0
        self.measured_stamp = None
        self.measured_imu_wz = None
        self.measured_imu_stamp = None
        self.stop_lock = threading.RLock()
        self.fast_stop_requested = False
        self.fast_stop_ramp = FastStopRamp(
            align_mps=float(rospy.get_param(
                "~fast_stop_align_mps", FAST_STOP_ALIGN_MPS)),
            align_diff_mps=float(rospy.get_param(
                "~fast_stop_align_diff_mps", FAST_STOP_ALIGN_DIFF_MPS)),
            align_max_s=float(rospy.get_param(
                "~fast_stop_align_max_s", FAST_STOP_ALIGN_MAX_S)),
            final_mps=float(rospy.get_param(
                "~fast_stop_final_mps", FAST_STOP_FINAL_MPS)),
            s_threshold_mps=float(rospy.get_param(
                "~fast_stop_s_mps", FAST_STOP_S_MPS)),
            max_duration_s=float(rospy.get_param(
                "~fast_stop_max_s", FAST_STOP_MAX_S)),
            imu_abort_deg_s=float(rospy.get_param(
                "~fast_stop_imu_abort_deg_s", FAST_STOP_IMU_ABORT_DEG_S)),
            imu_abort_hold_s=float(rospy.get_param(
                "~fast_stop_imu_abort_hold_s", FAST_STOP_IMU_ABORT_HOLD_S)),
        )
        self.fast_stop_imu_topic = str(rospy.get_param(
            "~fast_stop_imu_topic", FAST_STOP_IMU_TOPIC)).strip()

        if self.stop_ramp_mode == "fast":
            rospy.loginfo(
                "wheel_cmd: fast stop staged (align %.2f -> final %.2f -> "
                "S <= %.2f m/s, max %.2f s, imu abort %.1f deg/s)",
                self.fast_stop_ramp.align_mps,
                self.fast_stop_ramp.final_mps,
                self.fast_stop_ramp.s_threshold_mps,
                self.fast_stop_ramp.max_duration_s,
                math.degrees(self.fast_stop_ramp.imu_abort_rad_s),
            )
            self.fast_stop_timer = rospy.Timer(
                rospy.Duration(FAST_STOP_PERIOD_S),
                self.on_fast_stop_timer
            )
            if self.fast_stop_imu_topic:
                rospy.Subscriber(
                    self.fast_stop_imu_topic,
                    Imu,
                    self.on_imu,
                    queue_size=10,
                )
        rospy.Subscriber(
            "cmd_vel", Twist, self.on_velocity, queue_size=1)
        rospy.Subscriber(
            "wheel_status",
            Int16MultiArray,
            self.on_wheel_status,
            queue_size=5,
        )

    def on_imu(self, message):
        try:
            wz = float(message.angular_velocity.z)
        except (AttributeError, TypeError, ValueError):
            return
        if not math.isfinite(wz):
            return
        now = rospy.Time.now().to_sec()
        with self.stop_lock:
            self.measured_imu_wz = wz
            self.measured_imu_stamp = now

    def publish(self, values):
        message = Int16MultiArray()
        message.data = list(values)
        self.publisher.publish(message)

    def publish_stop(self):
        now = rospy.Time.now().to_sec()
        if self.measured_stamp is None or \
                now - self.measured_stamp > STATUS_FRESH_S:
            self.publish(STOP_COMMAND)
            return
        self.publish(stop_ramp_command(
            self.measured_left_mps, self.measured_right_mps))

    def force_fast_stop(self):
        """오류 발생 시 Ramp를 중단하고 S 명령."""
        self.fast_stop_requested = True
        self.fast_stop_ramp.abort()
        self.publish(STOP_COMMAND)

    def publish_fast_stop(self):
        now = rospy.Time.now().to_sec()

        if (self.mode != AUTO_MODE or
                self.fault_latched or
                self.measured_stamp is None or
                now < self.measured_stamp or
                now - self.measured_stamp > STATUS_FRESH_S):
            self.force_fast_stop()
            return

        imu_wz = None
        imu_stamp = None
        if (self.measured_imu_stamp is not None and
                0.0 <= now - self.measured_imu_stamp <= FAST_STOP_IMU_FRESH_S):
            imu_wz = self.measured_imu_wz
            imu_stamp = self.measured_imu_stamp

        command = self.fast_stop_ramp.command(
            now,
            self.measured_left_mps,
            self.measured_right_mps,
            imu_wz_rad_s=imu_wz,
            imu_stamp=imu_stamp,
        )
        self.publish(command)

    def on_fast_stop_timer(self, _event):
        with self.stop_lock:
            if self.fast_stop_requested:
                self.publish_fast_stop()

    def on_velocity(self, message):
        with self.stop_lock:

            if self.mode != AUTO_MODE:
                if self.stop_ramp_mode == "fast":
                    self.force_fast_stop()
                else:
                    self.publish_stop()
                return

            if message_caller_id(message) not in self.cmd_callers:
                self.fault_latched = True

            command = compute_wheel_command(
                message.linear.x,
                message.angular.z,
                self.wheel_separation_m,
                self.turn_boost
            )

            if command is None:
                self.fault_latched = True

            if self.fault_latched:
                if self.stop_ramp_mode == "fast":
                    self.force_fast_stop()
                else:
                    self.publish_stop()
                return

            if command == STOP_COMMAND:
                if self.stop_ramp_mode == "fast":
                    if not self.fast_stop_requested:
                        self.fast_stop_requested = True
                        self.publish_fast_stop()
                    # 이후에는 50Hz 타이머가 감속을 계속 수행
                else:
                    self.publish_stop()
                return

            # 정상 주행 명령이 들어오면 정지 상태 초기화
            if self.stop_ramp_mode == "fast":
                self.fast_stop_requested = False
                self.fast_stop_ramp.reset()

            self.publish(command)

    def on_wheel_status(self, message):
        with self.stop_lock:

            if (message_caller_id(message) != EXPECTED_STATUS_CALLER
                    or len(message.data) <= 1):
                self.mode = None
                self.fault_latched = True

                if self.stop_ramp_mode == "fast":
                    self.force_fast_stop()
                else:
                    self.publish_stop()
                return

            if self.stop_ramp_mode == "fast" and len(message.data) < 6:
                self.measured_stamp = None
                self.fault_latched = True
                self.force_fast_stop()
                return

            if len(message.data) >= 6:

                if self.stop_ramp_mode == "fast":
                    try:
                        valid = (
                            all(
                                int(message.data[i]) in (67, 87, 83)
                                for i in (2, 4)
                            )
                            and all(
                                33 <= int(message.data[i]) <= 127
                                for i in (3, 5)
                            )
                        )
                    except (TypeError, ValueError, OverflowError):
                        valid = False

                    if not valid:
                        self.measured_stamp = None
                        self.fault_latched = True
                        self.force_fast_stop()
                        return

                self.measured_left_mps = wheel_speed_mps(
                    message.data[2], message.data[3]
                )
                self.measured_right_mps = wheel_speed_mps(
                    message.data[4], message.data[5]
                )
                self.measured_stamp = rospy.Time.now().to_sec()

            try:
                next_mode = int(message.data[1])
            except (TypeError, ValueError, OverflowError):
                if self.stop_ramp_mode == "fast":
                    self.mode = None
                    self.fault_latched = True
                    self.force_fast_stop()
                    return
                raise

            if next_mode != AUTO_MODE:
                self.fault_latched = False
            elif self.mode != AUTO_MODE:
                self.fault_latched = False

            self.mode = next_mode

            if (self.stop_ramp_mode == "fast"
                    and self.fast_stop_requested
                    and next_mode != AUTO_MODE):
                self.force_fast_stop()


if __name__ == "__main__":
    WheelCommandGuard()
    rospy.spin()

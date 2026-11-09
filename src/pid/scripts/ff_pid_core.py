#!/usr/bin/env python3
"""UNICON FF–PD–Yaw 경로 추종 제어 계산 모듈(ROS 비의존).

경로 JSON은 재샘플링하거나 평활화하지 않고 그대로 사용합니다.
주행 전 경로와 연결된 공통 Drivable Mask는 ROS Node에서 검증합니다.
"""
import bisect
import json
import math

import numpy as np


# ======================= FF–PD–Yaw 제어 파라미터 =======================
# 제어기 성능을 튜닝할 때는 이 부분만 수정합니다.
# Node는 여기서 설정한 값을 사용하고, 주행 명령 계산은 FFPD가 수행합니다.
# 아래 값은 초기 실험용이며 실차에서 검증·튜닝된 최종값이 아닙니다.
KP = 1.00                        # P 게인: 예상 CTE에 비례한 조향 보정
KD = 0.5                        # D 게인: 예상 CTE 변화율에 따른 조향 보정
KI = 0.00                        # I 게인: 사용하지 않음
FF_GAIN = 1.00                   # 경로 곡률 FF 보정 배율
FF_PREVIEW_WAYPOINTS = 5         # 예측 위치의 최근접 Waypoint에서 앞쪽으로 볼 칸 수
YAW_GAIN = 0.50                  # 예측 Heading Error 보정 게인 [1/s]
YAW_DEADZONE_DEG = 3.0           # 예측 Heading Error 연속형 데드존 [deg]
YAW_CURVATURE_SCALE_M = 10.0     # 1/(1+scale*abs(kappa)) 커브별 Yaw 보정 약화 [m]
MAX_V_MPS = 0.80                 # 휠체어 최대 전진 속도 [m/s]
MAX_W_RAD_S = 0.60               # 휠체어 최대 각속도 [rad/s]
WHEEL_TRACK_M = 0.46             # 유효 바퀴 간격 [m]; NUC 구동부와 동일해야 함
MIN_WHEEL_MPS = 0.30             # 좌·우 각각의 바퀴 최소 구동 속도 [m/s]
MAX_WHEEL_MPS = 1.00             # 좌·우 각각의 바퀴 최대 구동 속도 [m/s]
# 곡률·CTE 속도 상한과 SLOW 속도 계획은 Safety에서 수행합니다.
CTE_DEADZONE_M = 0.05           # 작은 횡방향 오차를 무시하는 구간 [m]
D_DEADZONE_MPS = 0.02            # D항 입력(CTE 변화율) 데드존 [m/s]
D_CLAMP_CURVATURE_INV_M = 0.15     # D항 곡률 제한 [1/m]; 최종 각속도 기여는 v를 곱함
LINEAR_ACCEL_LIMIT_MPS2 = 0.80   # RUNNING 선가속도 제한 [m/s²]; 일반 감속은 제한 없음
ANGULAR_ACCEL_LIMIT_RAD_S2 = 2.00  # RUNNING 각속도 변화율 제한 [rad/s²]
COMMAND_LIMIT_MAX_DT_S = 0.05    # 제어 루프 지연 시 한 번의 큰 명령 점프 방지

# 예측 시간과 구동 지연 가정은 서로 다른 개념입니다.
PREDICTION_HORIZON_S = 0.50      # 현재부터 예측할 미래 시간 [s]
PREDICTION_MEASURED_S = 0.10     # 예측 초기 측정 바퀴 속도를 사용하는 시간 [s]
PREDICTION_STEP_S = 0.02         # 예측기 내부 수치 적분 간격 [s]
PREDICTION_DELAY_S = 0.50        # 명령 반영 지연 가정 [s]; 실측 검증 필요
# ================================================================

# 과거 식별자와의 호환성만 유지합니다. 실제 단위는 rad/s가 아니라 [1/m]입니다.
D_CLAMP_RAD_S = D_CLAMP_CURVATURE_INV_M


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def deadzone(x, width):
    return math.copysign(max(0.0, abs(x) - width), x)


def limit_running_command(previous_v, previous_w, target_v, target_w, dt,
                          wheel_track, min_wheel, max_wheel,
                          linear_accel=LINEAR_ACCEL_LIMIT_MPS2,
                          angular_accel=ANGULAR_ACCEL_LIMIT_RAD_S2,
                          diagnostics=None):
    """Slew-limit normal RUNNING commands only; fault/STOP bypasses this function.

    Linear acceleration and angular velocity changes are limited. Normal
    linear deceleration is immediate, as requested. Wheel-speed feasibility
    always takes priority over angular slew rate, including during deceleration.
    """
    values = (previous_v, previous_w, target_v, target_w, dt, wheel_track,
              min_wheel, max_wheel, linear_accel, angular_accel)
    if not all(math.isfinite(float(x)) for x in values):
        raise ValueError("non-finite command limiter input")
    if not (0 < dt < 0.5 and linear_accel > 0 and angular_accel > 0
            and wheel_track > 0 and 0 <= min_wheel < max_wheel
            and 0 <= previous_v <= max_wheel and 0 <= target_v <= max_wheel):
        raise ValueError("invalid command limiter input")
    step = min(dt, COMMAND_LIMIT_MAX_DT_S)
    if target_v >= previous_v:
        v = min(target_v, previous_v + linear_accel * step)
    else:
        v = target_v  # No ordinary deceleration ramp.

    # Restrict steering until forward speed can support it without reversing
    # an inner wheel or exceeding the outer wheel's allowed speed.
    allowed_w = max(0.0, min(
        2.0 * (v - min_wheel) / wheel_track,
        2.0 * (max_wheel - v) / wheel_track))
    target_w_before_wheel_clip = target_w
    target_w = max(-allowed_w, min(allowed_w, target_w))
    max_change = angular_accel * step
    w_before_final_wheel_clip = previous_w + max(
        -max_change, min(max_change, target_w - previous_w))
    w = max(-allowed_w, min(allowed_w, w_before_final_wheel_clip))
    if diagnostics is not None:
        diagnostics.update({
            "limiter_dt_s": step,
            "limiter_previous_v_mps": previous_v,
            "limiter_previous_w_rad_s": previous_w,
            "limiter_target_v_mps": target_v,
            "limiter_target_w_rad_s": target_w_before_wheel_clip,
            "limiter_allowed_w_rad_s": allowed_w,
            "limiter_w_after_wheel_clip_rad_s": target_w,
            "limiter_w_after_angular_slew_rad_s": w_before_final_wheel_clip,
            "limiter_v_accel_limited": bool(target_v > v + 1e-12),
            "limiter_w_wheel_clipped": bool(abs(target_w_before_wheel_clip - target_w) > 1e-12),
            "limiter_w_slew_limited": bool(abs(target_w - w_before_final_wheel_clip) > 1e-12),
            "limiter_w_final_wheel_clipped": bool(abs(w_before_final_wheel_clip - w) > 1e-12),
        })
    return v, w


class Route:
    def __init__(self, filename):
        if isinstance(filename, dict):
            self.metadata = filename
        else:
            with open(filename, encoding="utf-8") as f:
                self.metadata = json.load(f)
        data = self.metadata
        if data.get("frame") != "map" or data.get("count") != len(data.get("waypoints", [])):
            raise ValueError("bad route frame/count")
        points = data["waypoints"]
        if len(points) < 3:
            raise ValueError("route requires >= 3 waypoints")
        self.xy = np.asarray([[p["x"], p["y"]] for p in points], dtype=float)
        if not np.isfinite(self.xy).all():
            raise ValueError("invalid route coordinates")
        self.seg = np.diff(self.xy, axis=0)
        self.seg_len = np.linalg.norm(self.seg, axis=1)
        if np.any(self.seg_len < 1e-5):
            raise ValueError("duplicate or too-close waypoints; fix route asset")
        self.s = np.r_[0.0, np.cumsum(self.seg_len)]
        self.length = float(self.s[-1])
        self.tangent = np.arctan2(self.seg[:, 1], self.seg[:, 0])
        # 경로 JSON에 이미 저장된 yaw_deg를 이용해 곡률을 계산합니다.
        # 제어기 내부에서 경로를 다시 보간하거나 평활화하지 않습니다.
        if not all("yaw_deg" in p for p in points):
            raise ValueError("route needs precomputed yaw_deg at every waypoint")
        yaw = np.unwrap(np.radians([p["yaw_deg"] for p in points]))
        self.curvature = np.gradient(yaw, self.s)
        if not np.isfinite(self.curvature).all():
            raise ValueError("route curvature not finite")

    def project(self, x, y, hint=None, window=3.0):
        pos = np.array([x, y], dtype=float)
        delta = pos - self.xy[:-1]
        ratio = np.clip(np.sum(delta * self.seg, axis=1) / self.seg_len ** 2, 0, 1)
        foot = self.xy[:-1] + ratio[:, None] * self.seg
        distance_sq = np.sum((pos - foot) ** 2, axis=1)
        if hint is not None:
            eligible = (self.s[:-1] <= hint + window) & (self.s[1:] >= hint - window)
            distance_sq[~eligible] = np.inf
        idx = int(np.argmin(distance_sq))
        if not math.isfinite(distance_sq[idx]):
            raise ValueError("no route segment in progress window")
        station = float(self.s[idx] + ratio[idx] * self.seg_len[idx])
        cte = float(np.cross(self.seg[idx] / self.seg_len[idx], pos - foot[idx]))
        return station, cte, math.sqrt(distance_sq[idx]), float(self.tangent[idx])

    def kappa(self, s):
        return float(np.interp(np.clip(s, 0, self.length), self.s, self.curvature))

    def nearest_waypoint_index(self, station):
        """경로 투영 진행도에서 가장 가까운 원본 Waypoint의 인덱스를 반환합니다."""
        s = float(np.clip(station, 0.0, self.length))
        right = bisect.bisect_left(self.s, s)
        if right > 0 and (right == len(self.s) or
                          s - self.s[right - 1] <= self.s[right] - s):
            return right - 1
        return right

    def top20_abs_curvature(self, station, ahead=1.5):
        """전방 구간 절댓값 곡률 중 상위 20%의 평균을 계산합니다.

        원본 경로의 곡률 샘플을 사용하며 양쪽 경계값만 보간합니다.
        경로의 재샘플링이나 추가 평활화는 하지 않습니다.
        """
        start = float(np.clip(station, 0.0, self.length))
        end = min(start + max(0.0, ahead), self.length)
        inside = (self.s > start) & (self.s < end)
        values = np.abs(np.r_[self.kappa(start), self.curvature[inside],
                                self.kappa(end)])
        count = max(1, int(math.ceil(0.20 * len(values))))
        return float(np.mean(np.partition(values, -count)[-count:]))


class SimplePredictor:
    """일정 시간 후의 휠체어 위치·자세를 예측합니다.

    초기 구간은 실제 측정된 속도를 사용하고, 나머지는 이전 명령 이력을
    가정 지연만큼 이동시켜 사용합니다. Smith Predictor는 아닙니다.
    내부 적분 간격은 ROS 제어 주기와 독립적입니다.
    지연 시간은 실측값이 아닌 가정이며 예측 시간과 혼동하면 안 됩니다.
    """

    def __init__(self, horizon=PREDICTION_HORIZON_S,
                 measured_interval=PREDICTION_MEASURED_S,
                 integration_dt=PREDICTION_STEP_S,
                 assumed_delay=PREDICTION_DELAY_S):
        if not all(math.isfinite(float(t)) for t in
                   (integration_dt, measured_interval, horizon, assumed_delay)) or not (
                   0 < integration_dt <= measured_interval < horizon and assumed_delay >= 0):
            raise ValueError("invalid predictor timing")
        self.horizon = float(horizon)
        self.measured_interval = float(measured_interval)
        self.integration_dt = float(integration_dt)
        self.assumed_delay = float(assumed_delay)
        self.stamps = []
        self.commands = []

    def reset(self):
        self.stamps.clear()
        self.commands.clear()

    def record(self, stamp, v, w):
        if not all(math.isfinite(z) for z in (stamp, v, w)):
            raise ValueError("invalid command history")
        if self.stamps and stamp < self.stamps[-1]:
            self.reset()
        self.stamps.append(float(stamp))
        self.commands.append((float(v), float(w)))
        cutoff = bisect.bisect_left(self.stamps, stamp - 3.0)
        del self.stamps[:cutoff]
        del self.commands[:cutoff]

    def predict(self, pose, measured, now):
        x, y, heading = pose
        vm, wm = measured
        t = 0.0
        while t < self.horizon - 1e-10:
            # 측정값 구간과 과거 명령 구간의 경계를 넘지 않도록 적분 간격을 나눕니다.
            boundary = self.measured_interval if t < self.measured_interval - 1e-10 else self.horizon
            h = min(self.integration_dt, boundary - t)
            mid = t + 0.5 * h
            v, w = vm, wm
            if t >= self.measured_interval - 1e-10:
                sample_at = now + mid - self.assumed_delay
                index = bisect.bisect_right(self.stamps, sample_at) - 1
                if index >= 0:
                    v, w = self.commands[index]
            mid_heading = heading + 0.5 * w * h
            x += v * math.cos(mid_heading) * h
            y += v * math.sin(mid_heading) * h
            heading += w * h
            t += h
        return x, y, wrap(heading)


class FFPD:
    def __init__(self, route, kp=KP, kd=KD, ki=KI,
                 ff_gain=FF_GAIN, ff_preview_waypoints=FF_PREVIEW_WAYPOINTS,
                 yaw_gain=YAW_GAIN, yaw_deadzone_deg=YAW_DEADZONE_DEG,
                 yaw_curvature_scale_m=YAW_CURVATURE_SCALE_M,
                 max_v=MAX_V_MPS, max_w=MAX_W_RAD_S, wheel_track=WHEEL_TRACK_M,
                 min_wheel=MIN_WHEEL_MPS, max_wheel=MAX_WHEEL_MPS,
                 cte_deadzone=CTE_DEADZONE_M, d_deadzone=D_DEADZONE_MPS,
                 d_limit=D_CLAMP_CURVATURE_INV_M,
                 prediction_horizon_s=PREDICTION_HORIZON_S,
                 prediction_measured_s=PREDICTION_MEASURED_S,
                 prediction_step_s=PREDICTION_STEP_S,
                 prediction_delay_s=PREDICTION_DELAY_S):
        self.route = route
        self.kp, self.kd, self.ki = kp, kd, ki
        self.ff_gain = ff_gain
        self.ff_preview_waypoints = ff_preview_waypoints
        self.yaw_gain, self.yaw_deadzone_deg = yaw_gain, yaw_deadzone_deg
        self.yaw_curvature_scale_m = yaw_curvature_scale_m
        self.max_v, self.max_w = max_v, max_w
        self.wheel_track, self.min_wheel, self.max_wheel = wheel_track, min_wheel, max_wheel
        self.cte_deadzone, self.d_deadzone, self.d_limit = cte_deadzone, d_deadzone, d_limit
        params = (kp, kd, ki, max_v, max_w, wheel_track, min_wheel,
                  max_wheel, cte_deadzone,
                  d_deadzone, d_limit, ff_gain, yaw_gain,
                  yaw_deadzone_deg, yaw_curvature_scale_m)
        if not all(math.isfinite(float(p)) for p in params):
            raise ValueError("controller parameters must be finite")
        if not (0 <= min_wheel < max_wheel and 0 < max_v <= max_wheel
                and 0 < max_w and wheel_track > 0
                and cte_deadzone >= 0 and d_deadzone >= 0
                and d_limit >= 0 and kp >= 0 and kd >= 0 and ki >= 0
                and ff_gain >= 0 and yaw_gain >= 0
                and 0 <= yaw_deadzone_deg <= 180 and yaw_curvature_scale_m >= 0
                and isinstance(ff_preview_waypoints, int)
                and not isinstance(ff_preview_waypoints, bool)
                and ff_preview_waypoints >= 0):
            raise ValueError("invalid controller tuning parameters")
        self.predictor = SimplePredictor(
            horizon=prediction_horizon_s,
            measured_interval=prediction_measured_s,
            integration_dt=prediction_step_s,
            assumed_delay=prediction_delay_s)
        self.station = None
        self.prev_error = None
        self.integral = 0.0
        self.last_observation_id = None
        self.last_observation_time = None
        self.last_derivative_error = None
        self.derivative = 0.0
        self.derivative_raw = 0.0
        self.derivative_sample_dt_s = None

    def reset(self):
        self.station = None
        self.prev_error = None
        self.integral = 0.0
        self.predictor.reset()
        self.last_observation_id = None
        self.last_observation_time = None
        self.last_derivative_error = None
        self.derivative = 0.0
        self.derivative_raw = 0.0
        self.derivative_sample_dt_s = None

    def step(self, pose, measured, now, dt, observation_id=None,
             observation_time=None, motion_policy="CLEAR", pitch_deg=0.0,
             speed_limit_mps=None):
        """다음 선속도·각속도 명령과 제어 진단값을 계산합니다.

        물리적인 바퀴 속도 제약을 만족하지 못하면 feasible=False로 반환합니다.
        정지·종료·ROS 발행 여부는 Node에서 판단합니다.
        """
        if not (0 < dt < 0.5 and all(math.isfinite(z) for z in pose + measured)
                and math.isfinite(float(pitch_deg))):
            raise ValueError("bad dt/pose/velocity/pitch")
        if motion_policy not in ("CLEAR", "SLOW", "STOP"):
            raise ValueError("invalid obstacle policy")
        if speed_limit_mps is None:
            if motion_policy != "STOP":
                raise ValueError("speed ceiling required for CLEAR/SLOW")
            speed_limit_mps = 0.0
        speed_limit_mps = float(speed_limit_mps)
        if not math.isfinite(speed_limit_mps) or not 0 <= speed_limit_mps <= self.max_v:
            raise ValueError("invalid safety speed ceiling")
        if (motion_policy == "STOP") != (speed_limit_mps == 0):
            raise ValueError("STOP must have zero ceiling; CLEAR/SLOW must be positive")
        x, y, yaw = pose
        self.station, cte, distance, tangent = self.route.project(x, y, self.station)
        heading_error = wrap(yaw - tangent)
        predicted = self.predictor.predict(pose, measured, now)
        predicted_station, raw_error, predicted_distance, predicted_tangent = self.route.project(
            predicted[0], predicted[1], self.station)
        error = deadzone(raw_error, self.cte_deadzone)
        derivative_updated = False
        # D is updated on a new global pose observation, using its real interval.
        if observation_id is not None and observation_id != self.last_observation_id:
            stamp = float(observation_time) if observation_time is not None else float(now)
            if self.last_observation_time is not None and stamp > self.last_observation_time:
                sample_dt = stamp - self.last_observation_time
                raw_de = (error - self.last_derivative_error) / sample_dt
                self.derivative_raw = raw_de
                self.derivative_sample_dt_s = sample_dt
                self.derivative = deadzone(raw_de, self.d_deadzone)
            else:
                self.derivative_raw = 0.0
                self.derivative_sample_dt_s = None
                self.derivative = 0.0
            derivative_updated = True
            self.last_derivative_error = error
            self.last_observation_time = stamp
            self.last_observation_id = observation_id
        elif observation_id is None:
            self.derivative_raw = ((error - self.prev_error) / dt
                                   if self.prev_error is not None else 0.0)
            self.derivative_sample_dt_s = dt if self.prev_error is not None else None
            self.derivative = deadzone(self.derivative_raw, self.d_deadzone)
            derivative_updated = True
        self.prev_error = error
        self.integral = float(np.clip(self.integral + error * dt, -0.5, 0.5))

        # FF는 예측 위치의 최근접 Waypoint에서 정확히 N칸 앞의 곡률을 사용합니다.
        ff_waypoint_index = min(
            self.route.nearest_waypoint_index(predicted_station) + self.ff_preview_waypoints,
            len(self.route.s) - 1)
        k_ff = float(self.route.curvature[ff_waypoint_index])
        # Yaw는 FF 미리보기 끝점이 아니라 예측 위치 자체의 경로 접선을 봅니다.
        predicted_yaw_error = wrap(predicted[2] - predicted_tangent)
        yaw_weight = 1.0 / (1.0 + self.yaw_curvature_scale_m *
                            abs(self.route.kappa(predicted_station)))
        yaw_error_after_deadzone = deadzone(
            predicted_yaw_error, math.radians(self.yaw_deadzone_deg))
        yaw_rate = -self.yaw_gain * yaw_weight * yaw_error_after_deadzone
        # Safety owns all trajectory-related speed planning.
        # Steering and wheel feasibility remain in Core.
        v = min(self.max_v, speed_limit_mps)

        # PD corrects route curvature, with time-based derivative.
        k_p = -self.kp * error
        k_d_before_clamp = -self.kd * self.derivative
        k_d = float(np.clip(k_d_before_clamp, -self.d_limit, self.d_limit))
        k_i = -self.ki * self.integral
        # Always calculate FF-PD; low measured speed limits the final yaw rate below.
        k_cmd = self.ff_gain * k_ff + k_p + k_d + k_i
        steering_state = "TRACKING"
        requested_w = v * k_cmd + yaw_rate
        # The slower wheel must stay above min_wheel in the command, while
        # measured vehicle speed limits how much turning is allowed at launch.
        v_for_steering = min(v, measured[0])
        if v <= 0.0:
            allowable_w = 0.0
        else:
            allowable_w = max(0.0, min(self.max_w,
                2 * (v_for_steering - self.min_wheel) / self.wheel_track,
                2 * (self.max_wheel - v) / self.wheel_track))
        w = float(np.clip(requested_w, -allowable_w, allowable_w))
        vl, vr = v - self.wheel_track * w / 2, v + self.wheel_track * w / 2
        return {"feasible": True, "hold_reason": None,
                "v": v, "w": w,
                "station": self.station, "cte": cte, "distance": distance,
                "heading_error": heading_error,
                "predicted_cte": raw_error,
                "cte_after_deadzone_m": error,
                "d_raw_cte_rate_mps": self.derivative_raw,
                "d_after_deadzone_cte_rate_mps": self.derivative,
                "d_sample_dt_s": self.derivative_sample_dt_s,
                "d_recomputed": derivative_updated,
                "d_curvature_before_clamp_inv_m": k_d_before_clamp,
                "d_curvature_after_clamp_inv_m": k_d,
                "d_curvature_clamped": bool(abs(k_d_before_clamp - k_d) > 1e-12),
                "predicted_distance": predicted_distance,
                "predicted_station": predicted_station,
                "predicted_heading_error": predicted_yaw_error,
                "yaw_error_after_deadzone_rad": yaw_error_after_deadzone,
                "yaw_weight": yaw_weight,
                "yaw_path_curvature_inv_m": self.route.kappa(predicted_station),
                "ff_waypoint_index": ff_waypoint_index,
                "ff_waypoint_station": float(self.route.s[ff_waypoint_index]),
                "ff_preview_waypoints": self.ff_preview_waypoints,
                "ff_curvature_inv_m": k_ff,
                "speed_limit_mps": speed_limit_mps,
                "ff": v * self.ff_gain * k_ff, "p": v * k_p, "i": v * k_i,
                "d": v * k_d, "yaw": yaw_rate,
                "k_cmd": k_cmd, "k_ff": k_ff, "k_p": k_p, "k_d": k_d,
                "requested_w_rad_s": requested_w,
                "core_allowed_w_rad_s": allowable_w,
                "core_wheel_clipped": bool(abs(requested_w - w) > 1e-12),
                "v_for_steering_mps": v_for_steering,
                "steering_state": steering_state, "motion_policy": motion_policy,
                "pitch_deg": float(pitch_deg),
                "vl": vl, "vr": vr, "predicted_pose": predicted}

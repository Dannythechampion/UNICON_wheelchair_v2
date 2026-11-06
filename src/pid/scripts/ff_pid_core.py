#!/usr/bin/env python3
"""UNICON FF–PD 경로 추종 제어 계산 모듈(ROS 비의존).

경로 JSON은 재샘플링하거나 평활화하지 않고 그대로 사용합니다.
주행 전 경로와 연결된 공통 Drivable Mask는 ROS Node에서 검증합니다.
"""
import bisect
import json
import math

import numpy as np


# ======================= FF–PD 제어 파라미터 =======================
# 제어기 성능을 튜닝할 때는 이 부분만 수정합니다.
# Node는 여기서 설정한 값을 사용하고, 주행 명령 계산은 FFPD가 수행합니다.
# 아래 값은 초기 실험용이며 실차에서 검증·튜닝된 최종값이 아닙니다.
KP = 1.00                        # P 게인: 예상 CTE에 비례한 조향 보정
KD = 0.5                        # D 게인: 예상 CTE 변화율에 따른 조향 보정
KI = 0.00                        # I 게인: 초기 FF–PD 실험에서는 사용하지 않음
MAX_V_MPS = 0.80                 # 휠체어 최대 전진 속도 [m/s]
MAX_W_RAD_S = 0.60               # 휠체어 최대 각속도 [rad/s]
WHEEL_TRACK_M = 0.46             # 유효 바퀴 간격 [m]; NUC 구동부와 동일해야 함
MIN_WHEEL_MPS = 0.30             # 좌·우 각각의 바퀴 최소 구동 속도 [m/s]
MAX_WHEEL_MPS = 1.00             # 좌·우 각각의 바퀴 최대 구동 속도 [m/s]
LATERAL_ACCEL_MPS2 = 0.25        # 커브 감속에 사용할 횡가속도 제한 [m/s²]
CURVATURE_PREVIEW_M = 2.0       # 감속 판단 시 참조할 전방 경로 길이 [m]
CTE_DEADZONE_M = 0.20           # 작은 횡방향 오차를 무시하는 구간 [m]
D_DEADZONE_MPS = 0.02            # D항 입력(CTE 변화율) 데드존 [m/s]
D_CLAMP_RAD_S = 0.15             # D항 단독 조향 기여량 제한 [rad/s]

# 예측 시간과 구동 지연 가정은 서로 다른 개념입니다.
PREDICTION_HORIZON_S = 0.50      # 현재부터 예측할 미래 시간 [s]
PREDICTION_MEASURED_S = 0.10     # 예측 초기 측정 바퀴 속도를 사용하는 시간 [s]
PREDICTION_STEP_S = 0.02         # 예측기 내부 수치 적분 간격 [s]
PREDICTION_DELAY_S = 0.50        # 명령 반영 지연 가정 [s]; 실측 검증 필요
# ================================================================


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def deadzone(x, width):
    return math.copysign(max(0.0, abs(x) - width), x)


class Route:
    def __init__(self, filename):
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
                 max_v=MAX_V_MPS, max_w=MAX_W_RAD_S, wheel_track=WHEEL_TRACK_M,
                 min_wheel=MIN_WHEEL_MPS, max_wheel=MAX_WHEEL_MPS,
                 lateral_accel=LATERAL_ACCEL_MPS2, preview_m=CURVATURE_PREVIEW_M,
                 cte_deadzone=CTE_DEADZONE_M, d_deadzone=D_DEADZONE_MPS,
                 d_limit=D_CLAMP_RAD_S,
                 prediction_horizon_s=PREDICTION_HORIZON_S,
                 prediction_measured_s=PREDICTION_MEASURED_S,
                 prediction_step_s=PREDICTION_STEP_S,
                 prediction_delay_s=PREDICTION_DELAY_S):
        self.route = route
        self.kp, self.kd, self.ki = kp, kd, ki
        self.max_v, self.max_w = max_v, max_w
        self.wheel_track, self.min_wheel, self.max_wheel = wheel_track, min_wheel, max_wheel
        self.lateral_accel, self.preview_m = lateral_accel, preview_m
        self.cte_deadzone, self.d_deadzone, self.d_limit = cte_deadzone, d_deadzone, d_limit
        params = (kp, kd, ki, max_v, max_w, wheel_track, min_wheel,
                  max_wheel, lateral_accel, preview_m, cte_deadzone,
                  d_deadzone, d_limit)
        if not all(math.isfinite(float(p)) for p in params):
            raise ValueError("controller parameters must be finite")
        if not (0 <= min_wheel < max_wheel and 0 < max_v <= max_wheel
                and 0 < max_w and wheel_track > 0 and lateral_accel > 0
                and preview_m > 0 and cte_deadzone >= 0 and d_deadzone >= 0
                and d_limit >= 0 and kp >= 0 and kd >= 0 and ki >= 0):
            raise ValueError("invalid controller tuning parameters")
        self.predictor = SimplePredictor(
            horizon=prediction_horizon_s,
            measured_interval=prediction_measured_s,
            integration_dt=prediction_step_s,
            assumed_delay=prediction_delay_s)
        self.station = None
        self.prev_error = None
        self.integral = 0.0

    def reset(self):
        self.station = None
        self.prev_error = None
        self.integral = 0.0
        self.predictor.reset()

    def step(self, pose, measured, now, dt):
        """다음 선속도·각속도 명령과 제어 진단값을 계산합니다.

        물리적인 바퀴 속도 제약을 만족하지 못하면 feasible=False로 반환합니다.
        정지·종료·ROS 발행 여부는 Node에서 판단합니다.
        """
        if not (0 < dt < 0.5 and all(math.isfinite(z) for z in pose + measured)):
            raise ValueError("bad dt/pose/velocity")
        x, y, yaw = pose
        self.station, cte, distance, tangent = self.route.project(
            x, y, self.station)
        heading_error = wrap(yaw - tangent)

        predicted = self.predictor.predict(pose, measured, now)
        _, raw_error, predicted_distance, _ = self.route.project(
            predicted[0], predicted[1], self.station)
        error = deadzone(raw_error, self.cte_deadzone)
        de = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        self.integral = float(np.clip(self.integral + error * dt, -0.5, 0.5))

        # 커브 감속은 전방 곡률의 최댓값 대신 상위 20% 평균을 사용합니다.
        k_ff = self.route.kappa(self.station + 0.2)
        k_preview = self.route.top20_abs_curvature(self.station, self.preview_m)
        v = self.max_v
        if k_preview > 1e-4:
            v = min(v, math.sqrt(self.lateral_accel / k_preview),
                    self.max_w / k_preview)

        ff = v * k_ff
        p = -self.kp * error
        d = float(np.clip(-self.kd * deadzone(de, self.d_deadzone),
                          -self.d_limit, self.d_limit))
        i_term = -self.ki * self.integral
        wanted_w = float(np.clip(ff + p + i_term + d, -self.max_w, self.max_w))
        if (abs(measured[0]) < 0.15 and abs(k_ff) < 0.035 and
                abs(cte) < 0.08 and abs(heading_error) < 0.10):
            wanted_w = 0.0

        allowable_w = max(0.0, min(self.max_w,
                     2 * (v - self.min_wheel) / self.wheel_track,
                     2 * (self.max_wheel - v) / self.wheel_track))
        reason = None
        if v < self.min_wheel:
            reason = "CURVE_SPEED_BELOW_WHEEL_MIN"
        elif abs(wanted_w) > allowable_w + 0.02:
            reason = "WHEEL_STEERING_INFEASIBLE"

        if reason is None:
            w = float(np.clip(wanted_w, -allowable_w, allowable_w))
            vl, vr = v - self.wheel_track * w / 2, v + self.wheel_track * w / 2
        else:
            v, w, vl, vr = 0.0, 0.0, 0.0, 0.0

        return {"feasible": reason is None, "hold_reason": reason,
                "v": v, "w": w,
                "station": self.station, "cte": cte, "distance": distance,
                "heading_error": heading_error,
                "predicted_cte": raw_error,
                "predicted_distance": predicted_distance,
                "preview_curvature_top20": k_preview,
                "ff": ff, "p": p, "i": i_term, "d": d,
                "vl": vl, "vr": vr, "predicted_pose": predicted}

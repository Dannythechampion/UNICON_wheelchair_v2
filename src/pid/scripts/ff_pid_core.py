#!/usr/bin/env python3
"""UNICON FF-PD path tracking math (no rospy). All wheel constraints live here.

Route JSON is used AS IS; no resampling, smoothing or route modification.
The caller must validate its bound safety band / drivable mask before driving.
"""
import bisect
import json
import math

import numpy as np


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
        # Existing smoothed route files carry yaw_deg. Use it for curvature,
        # rather than resampling the path inside a controller.
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
        """Average the highest 20% of |curvature| in the forward route segment.

        Uses the ORIGINAL route's curvature samples; includes interpolated
        window boundaries only. No waypoint resampling or path smoothing.
        """
        start = float(np.clip(station, 0.0, self.length))
        end = min(start + max(0.0, ahead), self.length)
        inside = (self.s > start) & (self.s < end)
        values = np.abs(np.r_[self.kappa(start), self.curvature[inside],
                                self.kappa(end)])
        count = max(1, int(math.ceil(0.20 * len(values))))
        return float(np.mean(np.partition(values, -count)[-count:]))


class SimplePredictor:
    """Look 0.5s ahead: measured wheel twist for first 0.1s and
    past issued commands for the remaining 0.4s. Not a Smith model.

    The motion integral uses 0.02s substeps, independent of the 0.1s ROS
    controller period. assumed_delay=0.5s is an unvalidated alignment
    hypothesis; the 0.5s prediction horizon is a different concept.
    """

    def __init__(self, horizon=0.5, measured_interval=0.1,
                 integration_dt=0.02, assumed_delay=0.5):
        if not (0 < integration_dt <= measured_interval < horizon and
                assumed_delay >= 0):
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
            # Do not step across the 0.1s measured/command boundary.
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
    def __init__(self, route, kp=0.3, kd=0.02, ki=0.0,
                 max_v=0.5, max_w=0.5, wheel_track=0.54,
                 min_wheel=0.30, max_wheel=1.0,
                 lateral_accel=0.25, preview_m=2.0,
                 cte_deadzone=0.015, d_deadzone=0.0, d_limit=0.15):
        self.route = route
        self.kp, self.kd, self.ki = kp, kd, ki
        self.max_v, self.max_w = max_v, max_w
        self.wheel_track, self.min_wheel, self.max_wheel = wheel_track, min_wheel, max_wheel
        self.lateral_accel, self.preview_m = lateral_accel, preview_m
        self.cte_deadzone, self.d_deadzone, self.d_limit = cte_deadzone, d_deadzone, d_limit
        self.predictor = SimplePredictor()
        self.station = None
        self.prev_error = None
        self.integral = 0.0
        if not (0 <= min_wheel < max_wheel and 0 < max_v <= max_wheel and 0 < max_w):
            raise ValueError("invalid wheel/controller speed parameters")

    def reset(self):
        self.station = None
        self.prev_error = None
        self.integral = 0.0
        self.predictor.reset()

    def step(self, pose, measured, now, dt):
        """Compute candidate Twist and diagnostics, not ROS/safety decisions.

        Invalid or physically infeasible commands are reported as feasible=False.
        The ROS node decides whether to hold, finish, or publish commands.
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

        # Preview speed uses mean of the highest 20% of |curvature|, not max.
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

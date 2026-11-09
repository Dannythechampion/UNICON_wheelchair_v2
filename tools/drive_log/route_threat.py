"""Route-based obstacle threat for the drive logger (read-only analysis).

Mirrors the revised checklist (경로기준_위협판정_체크리스트_수정본_20261108.md):
  * every object point (profile slices stretched to the box's far face, or the
    box perimeter) is projected onto the route near the chair -> (s_ahead, d)
  * on-route threat: 0 <= s_ahead <= min(PLAN_AHEAD_M, route remaining) and
    |d| < half_width; distance = smallest s_ahead
  * gate-zone supplement: the straight 0.5 m strip inside the approximate stop
    radius + 0.6 m, reported separately (it must never create a bypass)
  * counts objects whose profile has a broken (non-finite) slice anywhere

It only uses cluster_guard primitives that exist in every deploy
(object_box, parsed_profile, BROKEN, slices_in, object_motion, Threat,
nearest_threat), so the logger can compute it whatever the chair runs.
"""
import math

import numpy as np

PLAN_AHEAD_M = 8.0
HALF_WIDTH_M = 0.45            # waypoint_follower.CORRIDOR_HALF_WIDTH
GATE_HALF_WIDTH_M = 0.5        # safety_gate.HALF_WIDTH_M
GATE_EXTRA_M = 0.6             # safety_gate.FORWARD_CHECK_EXTRA_M
SAMPLE_M = 0.2
MAX_SAMPLES = 600
WINDOW_BACK_M = 2.0
WINDOW_AHEAD_EXTRA_M = 2.0


class Route(object):
    def __init__(self, waypoints):
        self.xy = np.asarray(waypoints, dtype=float).reshape(-1, 2)
        seg = np.diff(self.xy, axis=0)
        self.seg_len = np.linalg.norm(seg, axis=1)
        self.seg_dir = seg / np.maximum(self.seg_len, 1e-9)[:, None]
        self.arc = np.concatenate([[0.0], np.cumsum(self.seg_len)])

    def project(self, points, seg_lo, seg_hi):
        """(s, d) of map points on segments [seg_lo, seg_hi)."""
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        a = self.xy[seg_lo:seg_hi]
        u = self.seg_dir[seg_lo:seg_hi]
        L = self.seg_len[seg_lo:seg_hi]
        rel = pts[:, None, :] - a[None, :, :]
        t = np.clip((rel * u[None]).sum(axis=2), 0.0, L[None, :])
        foot = a[None] + u[None] * t[:, :, None]
        off = pts[:, None, :] - foot
        dist = np.hypot(off[:, :, 0], off[:, :, 1])
        k = np.argmin(dist, axis=1)
        rows = np.arange(len(pts))
        s = self.arc[seg_lo:seg_hi][k] + t[rows, k]
        uk = u[k]
        d = uk[:, 0] * off[rows, k, 1] - uk[:, 1] * off[rows, k, 0]
        return s, d

    def chair_station(self, xy, nearest_index):
        lo = max(int(nearest_index) - 10, 0)
        hi = min(int(nearest_index) + 10, len(self.seg_len))
        s, d = self.project([xy], lo, max(hi, lo + 1))
        return float(s[0]), float(d[0])

    def window(self, chair_s):
        lo_s = chair_s - WINDOW_BACK_M
        hi_s = chair_s + PLAN_AHEAD_M + WINDOW_AHEAD_EXTRA_M
        lo = int(max(np.searchsorted(self.arc, lo_s) - 1, 0))
        hi = int(min(np.searchsorted(self.arc, hi_s) + 1, len(self.seg_len)))
        return lo, max(hi, lo + 1)


def object_points(CG, item):
    """Chair-frame (forward, left) samples, or CG.BROKEN."""
    box = CG.object_box(item)
    if box is None:
        return CG.BROKEN
    x, y, hx, hy = box
    far = x + hx
    prof = CG.parsed_profile(item)
    if prof is CG.BROKEN:
        return CG.BROKEN
    cols = []
    if prof is not None:
        try:
            cols = list(CG.slices_in(prof, -1e9, 1e9))
        except (TypeError, ValueError):
            return CG.BROKEN
    pts = []
    if cols:
        depth = max(max(far - near for _, near in cols), 0.0)
        rows = int(math.ceil(depth / SAMPLE_M)) + 1
        if rows * len(cols) > MAX_SAMPLES:
            rows = max(MAX_SAMPLES // len(cols), 2)
        for lat, near in cols:
            top = max(far, near)
            for k in range(rows):
                f = 0.0 if rows == 1 else k / float(rows - 1)
                pts.append((near + (top - near) * f, lat))
    else:
        near = max(0.0, x - hx)
        nx = max(int(math.ceil(2 * hx / SAMPLE_M)), 1)
        ny = max(int(math.ceil(2 * hy / SAMPLE_M)), 1)
        for k in range(ny + 1):
            yy = y - hy + 2 * hy * k / ny
            pts += [(near, yy), (far, yy)]
        for k in range(1, nx):
            xx = near + (far - near) * k / nx
            pts += [(xx, y - hy), (xx, y + hy)]
    return np.asarray(pts, dtype=float)


def has_broken_slice(CG, item):
    prof = CG.parsed_profile(item)
    if prof is None:
        return False
    if prof is CG.BROKEN:
        return True
    try:
        list(CG.slices_in(prof, -1e9, 1e9))
    except (TypeError, ValueError):
        return True
    return False


def approx_stop_radius(speed_mps):
    """0.9 m margin + v x ~1.4 s reaction + v^2 / (2 x 0.5 m/s^2)."""
    v = abs(float(speed_mps))
    return 0.9 + v * 1.4 + v * v / 1.0


def evaluate(CG, summary, route, pose_xy, pose_yaw, nearest_index,
             speed_mps=0.0):
    """dict with the on-route threat, the gate-zone supplement and counts."""
    out = {"route": None, "gate_zone": None, "broken": 0,
           "chair_s": None, "chair_d": None}
    if not summary.usable:
        out["route"] = CG.Threat(CG.BLOCKED, CG.MOVING, summary.status or
                                 "unusable", geometry_valid=False)
        return out
    chair_s, chair_d = route.chair_station(pose_xy, nearest_index)
    out["chair_s"], out["chair_d"] = chair_s, chair_d
    horizon = min(PLAN_AHEAD_M, route.arc[-1] - chair_s)
    lo, hi = route.window(chair_s)
    c, s = math.cos(pose_yaw), math.sin(pose_yaw)
    best = None
    for item in summary.objects:
        if has_broken_slice(CG, item):
            out["broken"] += 1
        pts = object_points(CG, item)
        motion = CG.object_motion(item)
        if pts is CG.BROKEN:
            cand = (CG.BLOCKED, CG.MOVING if CG.object_box(item) is None
                    else motion, item, None)
        else:
            mx = pose_xy[0] + c * pts[:, 0] - s * pts[:, 1]
            my = pose_xy[1] + s * pts[:, 0] + c * pts[:, 1]
            ps, pd = route.project(np.stack([mx, my], axis=1), lo, hi)
            ahead = ps - chair_s
            inside = (ahead >= 0.0) & (ahead <= horizon) & \
                (np.abs(pd) < HALF_WIDTH_M)
            if not inside.any():
                continue
            k = int(np.argmin(np.where(inside, ahead, np.inf)))
            cand = (max(0.0, float(ahead[k])), motion, item, float(pd[k]))
        if best is None or cand[0] < best[0]:
            best = cand
    if best is not None:
        dist, motion, item, d = best
        box = CG.object_box(item)
        tid = item.get("id")
        th = CG.Threat(dist, motion, str(item.get("class", "")),
                       lateral_m=None if box is None else float(box[1]),
                       track_id=tid if isinstance(tid, int) else None)
        th.route_d_m = d
        out["route"] = th
    straight = CG.nearest_threat(summary, GATE_HALF_WIDTH_M)
    zone = approx_stop_radius(speed_mps) + GATE_EXTRA_M
    if straight is not None and straight.distance_m < zone:
        straight.zone_m = zone
        out["gate_zone"] = straight
    return out


def would_go_round(threat):
    return threat is not None and not threat.is_person and threat.parked \
        and threat.distance_m < PLAN_AHEAD_M

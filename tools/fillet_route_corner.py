#!/usr/bin/env python3
"""Replace a kinked corner of a route with a circular fillet, and optionally
put a speed ceiling on a stretch of it.

Written for the J corner of the 2026-10-31 manual routes. Their picks put a
47-degree heading change into 1.2 m at A_to_D4 waypoints 451-457 (local radius
0.9-1.2 m against 4-6 m for the rest of the bend). Both 10-31 A -> D4 drives
braked there from 0.8-1.0 m/s and crawled 16-20 s at 0.1-0.3 m/s, swinging
-0.6/+0.4 m and +/-40 degrees off the line.

The span between two points on the route is rebuilt as: the route's own
tangent line out of the first point, an arc of --radius, and the route's own
tangent line into the second. The tangents are read off the route outside the
span, so the joins carry no heading step, and forward and reverse routes given
the same two points get the same geometry. Everything outside the span is kept
byte for byte; z and the band edges are recomputed only inside it.

Band edges inside the span are raycast against the drivable mask the band is
bound to - the same thing the manual band records (``manual_boundary_unverified``)
- with the band's own measured bias against that raycast taken off when it is
the generous direction.

--slow puts ``max_speed_mps`` on every band station between two points on the
route. The ceiling is read by mpc_speed.policy_speed through
SafetyBand.station_speed_cap, so horizon_speed brakes for it ahead of arrival.

The route, band and mask are bound by SHA-256, so both documents are rewritten
together and the binding is recomputed and then re-validated.

Usage:
  fillet_route_corner.py ROUTE BAND MASK_YAML --span X1,Y1 X2,Y2 --radius R
      [--slow X1,Y1 X2,Y2 SPEED] [--id-tag TAG]
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "src" / "static_livox_localization"
                       / "scripts"))

import mpc_speed                                        # noqa: E402
from route_assets import (mask_image_path, route_content_sha256,  # noqa: E402
                          sha256, validate_asset_binding)
from safety_band import SafetyBand                      # noqa: E402

# How far outside the span the tangent of the kept route is read from.
TANGENT_BASE_M = 1.2
# A point named on the command line has to be on the route, not near it.
ON_ROUTE_M = 0.5
RAY_STEP_M = 0.01
RAY_MAX_M = 8.0


def _write(path, text):
    """Always LF: these files are hashed byte for byte (see trim_route_tail)."""
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)


def _point(text):
    x, y = (float(v) for v in text.split(","))
    return np.array([x, y])


def _nearest(xy, point):
    d = np.linalg.norm(xy - point, axis=1)
    k = int(np.argmin(d))
    if d[k] > ON_ROUTE_M:
        raise SystemExit("(%.2f, %.2f) is %.2f m off the route"
                         % (point[0], point[1], d[k]))
    return k


def _arc_length(xy):
    return np.concatenate([[0.0], np.cumsum(
        np.linalg.norm(np.diff(xy, axis=0), axis=1))])


class Mask:
    def __init__(self, mask_yaml):
        meta = yaml.safe_load(Path(mask_yaml).read_text(encoding="utf-8"))
        self.res = float(meta["resolution"])
        self.origin = np.array(meta["origin"][:2], dtype=float)
        self.free = np.array(Image.open(mask_image_path(mask_yaml))) > 250

    def is_free(self, p):
        col = int((p[0] - self.origin[0]) / self.res)
        row = int(self.free.shape[0] - 1 - (p[1] - self.origin[1]) / self.res)
        return (0 <= row < self.free.shape[0] and 0 <= col < self.free.shape[1]
                and bool(self.free[row, col]))

    def ray(self, p, n):
        d = 0.0
        while d < RAY_MAX_M and self.is_free(p + n * d):
            d += RAY_STEP_M
        return round(d, 3)


def _normal(heading_deg):
    h = math.radians(heading_deg)
    return np.array([-math.sin(h), math.cos(h)])


def fillet(xy, ia, ib, radius, step):
    """Points strictly between ia and ib replacing the span."""
    base = int(round(TANGENT_BASE_M / step))
    if ia - base < 0 or ib + base >= len(xy):
        raise SystemExit("span too close to a route end to read tangents")
    d1 = xy[ia] - xy[ia - base]
    d1 /= np.linalg.norm(d1)
    d2 = xy[ib + base] - xy[ib]
    d2 /= np.linalg.norm(d2)
    turn = math.atan2(d1[0] * d2[1] - d1[1] * d2[0], float(d1 @ d2))
    s = np.linalg.solve(np.c_[d1, -d2], xy[ib] - xy[ia])
    vertex = xy[ia] + s[0] * d1
    t = radius * math.tan(abs(turn) / 2.0)
    if s[0] < t or -s[1] < t:
        raise SystemExit("radius %.2f m does not fit the span (tangent %.2f m,"
                         " legs %.2f / %.2f m)" % (radius, t, s[0], -s[1]))
    t1, t2 = vertex - d1 * t, vertex + d2 * t
    side = 1.0 if turn > 0 else -1.0
    centre = t1 + side * radius * np.array([-d1[1], d1[0]])
    a0 = math.atan2(*(t1 - centre)[::-1])
    arc_len = radius * abs(turn)
    dense = [xy[ia] + d1 * u for u in np.linspace(0.0, s[0] - t, 200)]
    dense += [centre + radius * np.array([math.cos(a), math.sin(a)])
              for a in a0 + side * np.linspace(0.0, arc_len / radius, 400)]
    dense += [t2 + d2 * u for u in np.linspace(0.0, -s[1] - t, 200)]
    dense = np.array(dense)
    arc = _arc_length(dense)
    n = max(2, int(round(arc[-1] / step)))
    at = np.linspace(0.0, arc[-1], n + 1)[1:-1]
    new = np.c_[np.interp(at, arc, dense[:, 0]), np.interp(at, arc, dense[:, 1])]
    return new, vertex, t1, t2, math.degrees(turn)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("route", type=Path)
    parser.add_argument("band", type=Path)
    parser.add_argument("mask_yaml", type=Path)
    parser.add_argument("--span", nargs=2, type=_point, required=True)
    parser.add_argument("--radius", type=float, required=True)
    parser.add_argument("--slow", nargs=3)
    parser.add_argument("--id-tag", default="")
    args = parser.parse_args()

    validate_asset_binding(args.route, args.band, args.mask_yaml)
    route = json.loads(args.route.read_text(encoding="utf-8"))
    band = json.loads(args.band.read_text(encoding="utf-8"))
    mask = Mask(args.mask_yaml)
    wps = route["waypoints"]
    xy = np.array([[w["x"], w["y"]] for w in wps])
    step = float(route["route_step_m"])

    ia, ib = sorted(_nearest(xy, p) for p in args.span)
    new_xy, vertex, t1, t2, turn = fillet(xy, ia, ib, args.radius, step)
    old_span = xy[ia:ib + 1]
    old_z = np.array([w["z"] for w in wps[ia:ib + 1]])
    middle = []
    for p in new_xy:
        d = np.linalg.norm(old_span - p, axis=1)
        k = np.argsort(d)[:2]
        wgt = 1.0 / np.maximum(d[k], 1e-3)
        middle.append({"x": round(float(p[0]), 4), "y": round(float(p[1]), 4),
                       "z": round(float(wgt @ old_z[k] / wgt.sum()), 4)})
    wps = wps[:ia + 1] + middle + wps[ib:]
    xy = np.array([[w["x"], w["y"]] for w in wps])
    # yaw_deg is the central-difference heading, as the source routes have it.
    grad = np.gradient(xy, axis=0)
    heading = np.degrees(np.arctan2(grad[:, 1], grad[:, 0]))
    for k in range(ia, ia + len(middle) + 2):
        wps[k]["yaw_deg"] = round(float(heading[k]), 3)

    # Band: drop the stations that fall inside the old span, lay new ones on
    # the new span at the band's own spacing.
    stations = band["stations"]
    sxy = np.array([[s["x"], s["y"]] for s in stations])
    owner = np.array([int(np.argmin(np.linalg.norm(old_span - p, axis=1)))
                      if np.linalg.norm(old_span - p, axis=1).min() < step
                      else -1 for p in sxy])
    inside = np.nonzero((owner > 0) & (owner < len(old_span) - 1))[0]
    if len(inside) == 0 or np.any(np.diff(inside) != 1):
        raise SystemExit("band stations inside the span are not one run")
    s0, s1 = int(inside[0]), int(inside[-1])
    bias = {}
    for side, sign in (("left", 1.0), ("right", -1.0)):
        gap = [s[side + "_m"] - mask.ray(np.array([s["x"], s["y"]]),
                                         sign * _normal(s["heading_deg"]))
               for s in stations[::5]]
        bias[side] = min(0.0, float(np.median(gap)))
    arc = _arc_length(xy)
    a_from = arc[_nearest(xy, sxy[s0 - 1])]
    a_to = arc[_nearest(xy, sxy[s1 + 1])]
    spacing = float(band["station_spacing_m"])
    count = max(1, int(round((a_to - a_from) / spacing)) - 1)
    template = stations[s0]
    fresh = []
    for a in np.linspace(a_from, a_to, count + 2)[1:-1]:
        p = np.array([np.interp(a, arc, xy[:, 0]), np.interp(a, arc, xy[:, 1])])
        k = min(int(np.searchsorted(arc, a)), len(xy) - 1)
        h = float(heading[k])
        st = dict(template)
        st.update({"x": round(float(p[0]), 3), "y": round(float(p[1]), 3),
                   "heading_deg": round(h, 3)})
        for side, sign in (("left", 1.0), ("right", -1.0)):
            st[side + "_m"] = round(max(0.0, mask.ray(p, sign * _normal(h))
                                        + bias[side]), 3)
        st.pop("max_speed_mps", None)
        fresh.append(st)
    stations = stations[:s0] + fresh + stations[s1 + 1:]
    band["stations"] = stations
    if "corridor" in band:
        band["corridor"]["stations_covered"] = len(stations)
        band["corridor"]["stations_total"] = len(stations)

    slow_note = ""
    if args.slow:
        p1, p2, speed = _point(args.slow[0]), _point(args.slow[1]), float(args.slow[2])
        if speed < mpc_speed.TURN_FLOOR_SPEED:
            raise SystemExit("a ceiling below TURN_FLOOR_SPEED %.2f is a stop"
                             % mpc_speed.TURN_FLOOR_SPEED)
        sxy = np.array([[s["x"], s["y"]] for s in stations])
        k1, k2 = sorted(_nearest(sxy, p) for p in (p1, p2))
        for s in stations[k1:k2 + 1]:
            s["max_speed_mps"] = speed
        slow_note = "stations %d-%d capped at %.2f m/s" % (k1, k2, speed)

    shift = len(wps) - route["count"]
    route["waypoints"] = wps
    route["count"] = len(wps)
    route["path_length_m"] = round(float(arc[-1]), 3)
    target = route.get("operator_target_waypoint_index")
    if target is not None and target >= ib:
        route["operator_target_waypoint_index"] = target + shift
    route["corner_fillets"] = route.get("corner_fillets", []) + [{
        "vertex_xy_m": [round(float(v), 3) for v in vertex],
        "tangent_points_xy_m": [[round(float(v), 3) for v in t1],
                                [round(float(v), 3) for v in t2]],
        "radius_m": args.radius,
        "turn_deg": round(turn, 1),
        "replaced_waypoints": [ia, ib],
    }]
    if args.id_tag:
        old_id = route["asset_binding"]["route_id"]
        if args.id_tag not in old_id:
            head, sep, rest = old_id.partition(":")
            route["asset_binding"]["route_id"] = head + args.id_tag + sep + rest
    band["route_id"] = route["asset_binding"]["route_id"]

    _write(args.band, json.dumps(band, indent=2) + "\n")
    route["asset_binding"]["safety_band_sha256"] = sha256(args.band)
    route["asset_binding"]["route_content_sha256"] = route_content_sha256(route)
    _write(args.route, json.dumps(route, indent=2) + "\n")
    validate_asset_binding(args.route, args.band, args.mask_yaml)

    print("%s: wp %d-%d -> %d points on R %.1f m (turn %.0f deg, vertex %.2f,%.2f)"
          % (args.route.name, ia, ib, len(middle), args.radius, turn, *vertex))
    print("  route %d wp, %.1f m; band %d stations (replaced %d-%d with %d, "
          "edge bias L %.3f R %.3f) %s"
          % (route["count"], route["path_length_m"], len(stations), s0, s1,
             len(fresh), bias["left"], bias["right"], slow_note))
    SafetyBand(str(args.band))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Set ``max_speed_mps`` on the band stations between two points of a route,
without touching the route geometry.

fillet_route_corner.py --slow does the same thing, but only together with a
fillet. This is for changing a speed zone on a route that is already filleted:
on 2026-11-07 the single 0.40 m/s J-corner + bump zone was split so the corner
runs at 0.60 (at 0.40 the chair was felt to slide down the cross slope while
turning) and only the run-up to the bump and the bump itself stay at 0.40.

Each --zone is applied in order, so a later zone overrides an earlier one
where they overlap. --clear X1,Y1 X2,Y2 removes the ceiling on that stretch.
The ceiling is read by mpc_speed.policy_speed through
SafetyBand.station_speed_cap, so horizon_speed brakes for it ahead of arrival.

The band is bound to its route by SHA-256, so the route's binding is
recomputed and the pair re-validated.

Usage:
  band_speed_zone.py ROUTE BAND MASK_YAML [--clear X1,Y1 X2,Y2]
      --zone X1,Y1 X2,Y2 SPEED [--zone ...]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "src" / "static_livox_localization"
                       / "scripts"))

import mpc_speed                                        # noqa: E402
from route_assets import (route_content_sha256, sha256,  # noqa: E402
                          validate_asset_binding)
from safety_band import SafetyBand                      # noqa: E402

# A point named on the command line has to be on the band, not near it.
ON_BAND_M = 0.5


def _write(path, text):
    """Always LF: these files are hashed byte for byte (see trim_route_tail)."""
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)


def _point(text):
    x, y = (float(v) for v in text.split(","))
    return np.array([x, y])


def _nearest(sxy, point):
    d = np.linalg.norm(sxy - point, axis=1)
    k = int(np.argmin(d))
    if d[k] > ON_BAND_M:
        raise SystemExit("(%.2f, %.2f) is %.2f m from the band, not on it"
                         % (point[0], point[1], d[k]))
    return k


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("route", type=Path)
    parser.add_argument("band", type=Path)
    parser.add_argument("mask_yaml", type=Path)
    parser.add_argument("--clear", nargs=2, type=_point)
    parser.add_argument("--zone", nargs=3, action="append", default=[])
    args = parser.parse_args()
    if not args.zone and args.clear is None:
        parser.error("nothing to do: give --zone and/or --clear")

    validate_asset_binding(args.route, args.band, args.mask_yaml)
    route = json.loads(args.route.read_text(encoding="utf-8"))
    band = json.loads(args.band.read_text(encoding="utf-8"))
    stations = band["stations"]
    sxy = np.array([[s["x"], s["y"]] for s in stations])

    notes = []
    if args.clear is not None:
        k1, k2 = sorted(_nearest(sxy, p) for p in args.clear)
        for s in stations[k1:k2 + 1]:
            s.pop("max_speed_mps", None)
        notes.append("cleared %d-%d" % (k1, k2))
    for p1, p2, speed in args.zone:
        speed = float(speed)
        if speed < mpc_speed.TURN_FLOOR_SPEED:
            raise SystemExit("a ceiling below TURN_FLOOR_SPEED %.2f is a stop"
                             % mpc_speed.TURN_FLOOR_SPEED)
        k1, k2 = sorted(_nearest(sxy, _point(p)) for p in (p1, p2))
        for s in stations[k1:k2 + 1]:
            s["max_speed_mps"] = speed
        notes.append("%d-%d at %.2f" % (k1, k2, speed))

    _write(args.band, json.dumps(band, indent=2) + "\n")
    route["asset_binding"]["safety_band_sha256"] = sha256(args.band)
    route["asset_binding"]["route_content_sha256"] = route_content_sha256(route)
    _write(args.route, json.dumps(route, indent=2) + "\n")
    validate_asset_binding(args.route, args.band, args.mask_yaml)
    SafetyBand(str(args.band))
    print("%s: %s" % (args.band.name, ", ".join(notes)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

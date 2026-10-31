#!/usr/bin/env python3
"""Drive a hash-bound route the other way, on the same ground.

The reversed route is the same centreline walked from its far end: the
waypoints in reverse order with their yaw turned 180 degrees, and the band's
stations in reverse order with left and right exchanged, because reversing
the heading flips which side of the line each limit is on. The drivable mask
is a region, not a direction, so the reversed pair is bound to the SAME mask
files and nothing about where the chair may go changes.

    python3 tools/reverse_route_assets.py \
        routes/20261028_route2_D5_to_A_waypoints.json \
        routes/20261028_route2_D5_to_A_safety_band.json \
        routes/route_2d_map_route2_D5_to_A.yaml \
        routes/20261030_route2_A_to_D5

writes <prefix>_waypoints.json and <prefix>_safety_band.json, then checks
the binding with route_assets.validate_asset_binding.
"""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "static_livox_localization" / "scripts"))

from route_assets import (  # noqa: E402
    mask_image_path, route_content_sha256, sha256, validate_asset_binding)

# Per-side station fields. Anything named left_* has a right_* twin.
SIDE_FIELDS = ("m", "drop_m", "kind", "rise_m", "corridor_m")


def turned(yaw_deg):
    return round((float(yaw_deg) + 180.0 + 180.0) % 360.0 - 180.0, 3)


def reverse_waypoints(waypoints):
    out = []
    for point in reversed(waypoints):
        point = dict(point)
        point["yaw_deg"] = turned(point["yaw_deg"])
        out.append(point)
    return out


def reverse_stations(stations):
    out = []
    for station in reversed(stations):
        station = dict(station)
        station["heading_deg"] = turned(station["heading_deg"])
        for field in SIDE_FIELDS:
            left, right = "left_" + field, "right_" + field
            if left in station or right in station:
                left_value = station.pop(left, None)
                right_value = station.pop(right, None)
                if right_value is not None:
                    station[left] = right_value
                if left_value is not None:
                    station[right] = left_value
        out.append(station)
    return out


def write_json(path, document):
    # LF and no trailing-newline dependence: route_assets.sha256 strips it,
    # and a CRLF checkout on Windows would change every hash.
    with open(path, "w", encoding="utf-8", newline="\n") as sink:
        json.dump(document, sink, indent=1, ensure_ascii=False)
        sink.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("route")
    parser.add_argument("band")
    parser.add_argument("mask_yaml")
    parser.add_argument("prefix")
    args = parser.parse_args()

    route = json.loads(Path(args.route).read_text(encoding="utf-8"))
    band = json.loads(Path(args.band).read_text(encoding="utf-8"))
    validate_asset_binding(args.route, args.band, args.mask_yaml)

    mask_sha = sha256(mask_image_path(args.mask_yaml))
    mask_yaml_sha = sha256(args.mask_yaml)
    reversed_from = Path(args.route).name

    new_route = dict(route)
    new_route.pop("asset_binding", None)
    new_route["waypoints"] = reverse_waypoints(route["waypoints"])
    new_route["count"] = len(new_route["waypoints"])
    new_route["reversed_from"] = reversed_from
    last = new_route["waypoints"][-1]
    if "operator_target_waypoint_index" in new_route:
        new_route["operator_target_waypoint_index"] = new_route["count"] - 1
    if "operator_target_xy_m" in new_route:
        new_route["operator_target_xy_m"] = [last["x"], last["y"]]
    content_sha = route_content_sha256(new_route)
    route_id = "%s:reversed:%s:%s" % (
        route["asset_binding"]["route_id"].split(":")[0],
        content_sha[:12], mask_sha[:12])

    new_band = dict(band)
    new_band["route_id"] = route_id
    new_band["stations"] = reverse_stations(band["stations"])
    new_band["reversed_from"] = Path(args.band).name

    prefix = Path(args.prefix)
    route_path = prefix.with_name(prefix.name + "_waypoints.json")
    band_path = prefix.with_name(prefix.name + "_safety_band.json")
    write_json(band_path, new_band)

    new_route["asset_binding"] = {
        "route_id": route_id,
        "drivable_mask_sha256": mask_sha,
        "drivable_mask_yaml_sha256": mask_yaml_sha,
        "safety_band_sha256": sha256(band_path),
        "route_content_sha256": content_sha,
    }
    write_json(route_path, new_route)

    validate_asset_binding(route_path, band_path, args.mask_yaml)
    first = new_route["waypoints"][0]
    print("%s: %d waypoints, start (%.2f, %.2f) yaw %.1f, end (%.2f, %.2f)"
          % (route_path.name, new_route["count"], first["x"], first["y"],
             first["yaw_deg"], last["x"], last["y"]))
    print("%s: %d stations, route_id %s"
          % (band_path.name, len(new_band["stations"]), route_id))


if __name__ == "__main__":
    main()

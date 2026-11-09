import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

ROOT = Path(__file__).parents[1]
SCRIPT = (
    ROOT / "src" / "static_livox_localization" / "scripts" / "route_assets.py"
)
SPEC = importlib.util.spec_from_file_location("route_assets", SCRIPT)
ASSETS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ASSETS)
SAFETY_SPEC = importlib.util.spec_from_file_location(
    "safety_band",
    ROOT / "src" / "static_livox_localization" / "scripts" / "safety_band.py",
)
SAFETY = importlib.util.module_from_spec(SAFETY_SPEC)
SAFETY_SPEC.loader.exec_module(SAFETY)
sys.path.insert(0, str(ROOT / "src" / "static_livox_localization" / "scripts"))
try:
    import mpc_core  # noqa: F401,E402  (mpc_speed imports it lazily)
    import mpc_speed as MPC_SPEED  # noqa: E402
finally:
    sys.path.pop(0)


def _station(band, point):
    return int(np.argmin(np.linalg.norm(band.xy - np.asarray(point), axis=1)))


def test_route2_a_to_d5_is_d5_to_a_driven_backwards_on_the_same_ground():
    routes = ROOT / "routes"
    mask = routes / "route_2d_map_route2_D5_to_A.yaml"
    forward = routes / "20261028_route2_D5_to_A_waypoints.json"
    forward_band = routes / "20261028_route2_D5_to_A_safety_band.json"
    backward = routes / "20261030_route2_A_to_D5_waypoints.json"
    backward_band = routes / "20261030_route2_A_to_D5_safety_band.json"
    binding = ASSETS.validate_asset_binding(backward, backward_band, mask)
    assert binding["drivable_mask_sha256"] == ASSETS.validate_asset_binding(
        forward, forward_band, mask)["drivable_mask_sha256"]

    there = json.loads(forward.read_text(encoding="utf-8"))["waypoints"]
    back = json.loads(backward.read_text(encoding="utf-8"))["waypoints"]
    assert [(p["x"], p["y"]) for p in back] == \
        [(p["x"], p["y"]) for p in reversed(there)]
    for a, b in zip(back, reversed(there)):
        turn = (a["yaw_deg"] - b["yaw_deg"]) % 360.0
        assert turn == pytest.approx(180.0, abs=1e-3)

    # Same limits either way, measured at every waypoint: reversing the
    # heading flips the lateral sign, which the left/right swap undoes.
    xy = [(p["x"], p["y"]) for p in back]
    one = SAFETY.SafetyBand(forward_band)
    two = SAFETY.SafetyBand(backward_band)
    lat_1, lo_1, hi_1 = one.margins_many(xy)
    lat_2, lo_2, hi_2 = two.margins_many(xy)
    assert lat_2 == pytest.approx(-lat_1, abs=1e-6)
    assert lo_2 == pytest.approx(-hi_1, abs=1e-6)
    assert hi_2 == pytest.approx(-lo_1, abs=1e-6)
    assert two.contains_many(xy).all()
    assert two.route_centre_chord_violations() == \
        sorted(len(one.xy) - 2 - i for i in one.route_centre_chord_violations())


ROUTE2_PAIRS = (("D5_to_A", "A_to_D5"), ("D5_to_D4", "D4_to_D5"),
                ("D4_to_A", "A_to_D4"))


def _route2(key):
    routes = ROOT / "routes"
    return (routes / ("20261031_route2_%s_waypoints.json" % key),
            routes / ("20261031_route2_%s_safety_band.json" % key))


def test_route2_segments_are_one_line_cut_at_d4_on_one_mask():
    mask = ROOT / "routes" / "route_2d_map_route2_intern_20261031.yaml"
    shas = set()
    for forward_key, backward_key in ROUTE2_PAIRS:
        forward, forward_band = _route2(forward_key)
        backward, backward_band = _route2(backward_key)
        shas.add(ASSETS.validate_asset_binding(
            forward, forward_band, mask)["drivable_mask_sha256"])
        shas.add(ASSETS.validate_asset_binding(
            backward, backward_band, mask)["drivable_mask_sha256"])
        there = json.loads(forward.read_text(encoding="utf-8"))["waypoints"]
        back = json.loads(backward.read_text(encoding="utf-8"))["waypoints"]
        assert [(p["x"], p["y"]) for p in back] == \
            [(p["x"], p["y"]) for p in reversed(there)]
        one = SAFETY.SafetyBand(forward_band)
        two = SAFETY.SafetyBand(backward_band)
        xy = [(p["x"], p["y"]) for p in back]
        lat_1, lo_1, hi_1 = one.margins_many(xy)
        lat_2, lo_2, hi_2 = two.margins_many(xy)
        assert lat_2 == pytest.approx(-lat_1, abs=1e-6)
        assert lo_2 == pytest.approx(-hi_1, abs=1e-6)
        assert two.contains_many(xy).all()
        assert one.route_centre_chord_violations() == []
        assert two.route_centre_chord_violations() == []
    assert len(shas) == 1

    # D5->D4 then D4->A is D5->A, joined at the one shared D4 waypoint.
    def xy(key):
        return [(p["x"], p["y"]) for p in json.loads(
            _route2(key)[0].read_text(encoding="utf-8"))["waypoints"]]
    whole, head, tail = xy("D5_to_A"), xy("D5_to_D4"), xy("D4_to_A")
    assert head[-1] == tail[0]
    assert head + tail[1:] == whole
    # D4 is the operator's 2026-10-31 choice, moved inwards by smoothing
    # the bend; anything further is not that point.
    assert math.dist(head[-1], (105.64, 60.00)) < 0.5


def test_route2_segment_waypoints_lie_inside_the_mask():
    image = np.array(Image.open(
        ROOT / "routes" / "route_2d_map_route2_intern_20261031.pgm")) > 250
    for key in ("D5_to_A",):
        pts = np.array(
            [[p["x"], p["y"]] for p in json.loads(
                _route2(key)[0].read_text(encoding="utf-8"))["waypoints"]])
        rows = np.rint(image.shape[0] - 1 - (pts[:, 1] + 108.8) / 0.1)
        cols = np.rint((pts[:, 0] + 66.9) / 0.1)
        assert image[rows.astype(int), cols.astype(int)].all()


def test_route2_manual_routes_are_bound_to_one_mask_and_meet_at_stations():
    # The keys route2_intern_env.sh hands out by default since 2026-10-31.
    manual = ROOT / "routes" / "route2_manual_20261031"
    mask = manual / "common_drivable_mask.yaml"
    shas, ends = set(), {}
    for forward_key, backward_key in ROUTE2_PAIRS:
        for key in (forward_key, backward_key):
            route = manual / ("%s_waypoints.json" % key)
            band = manual / ("%s_safety_band.json" % key)
            shas.add(ASSETS.validate_asset_binding(
                route, band, mask)["drivable_mask_sha256"])
            w = json.loads(route.read_text(encoding="utf-8"))["waypoints"]
            start, _, end = key.split("_")
            ends.setdefault(start, set()).add((w[0]["x"], w[0]["y"]))
            ends.setdefault(end, set()).add((w[-1]["x"], w[-1]["y"]))
    assert len(shas) == 1
    # Every route from or to a station starts or ends on the same point, so
    # the origin check in route2_intern_check.sh means one place per station.
    assert sorted(ends) == ["A", "D4", "D5"]
    for station, points in ends.items():
        assert len(points) == 1, (station, points)


def test_route2_manual_j_corner_is_filleted_and_slow_through_the_bump():
    # 2026-10-31: both A -> D4 drives crawled the J kink (R 0.9 m in the
    # picks) and run 1 wheelied to +35 degrees on the 6 cm bump at
    # (88.3, -12.3) with 0.8 m/s still commanded.
    manual = ROOT / "routes" / "route2_manual_20261031"
    corner, bump = np.array([87.4, -21.0]), np.array([88.3, -12.3])
    for key in ("A_to_D4", "A_to_D5", "D4_to_A", "D5_to_A"):
        w = json.loads((manual / ("%s_waypoints.json" % key))
                       .read_text(encoding="utf-8"))["waypoints"]
        xy = np.array([[p["x"], p["y"]] for p in w])
        step = np.diff(xy, axis=0)
        heading = np.unwrap(np.arctan2(step[:, 1], step[:, 0]))
        near = np.linalg.norm(xy[1:-1] - corner, axis=1) < 6.0
        kappa = np.abs(np.diff(heading)) / np.linalg.norm(step[1:], axis=1)
        assert 1.0 / kappa[near].max() > 2.4, key
        band = SAFETY.SafetyBand(manual / ("%s_safety_band.json" % key))
        # 2026-11-07: the corner runs at 0.60 (at 0.40 the chair slid down
        # the cross slope while turning); from 4 m before the bump on it
        # stays 0.40, and the chair is already at 0.40 when it gets there.
        assert band.station_speed_cap(corner) == pytest.approx(0.60), key
        assert band.station_speed_cap(bump) == pytest.approx(0.40), key
        assert band.station_speed_cap(np.array([88.3, -15.5])) == \
            pytest.approx(0.40), key
        uphill = key.startswith("A_")
        approach = band.xy[_station(band, (88.3, -14.3 if uphill else -10.3))]
        assert MPC_SPEED.horizon_speed(band, approach) <= 0.40 + 1e-9, key
        assert np.isinf(band.station_speed_cap(band.xy[0]))


def test_route2_d5_dip_runs_at_0p5_through_the_turn_into_the_climb():
    # 2026-11-09 D5 -> A: a short dip at map x 167.5-166.8 right before the
    # climb. At 0.8 m/s the chair rolled up to 0.86 m/s into it, pitched
    # +4.7 degrees, and at the dip -> climb transition (x 166.5-166.2) the
    # gate stopped it on the road ahead - the jolt the rider reported. The
    # zone covers the dip and holds 0.5 for ~2 s (1 m) past the transition,
    # per direction. The climb's kink at x ~151 runs at 0.6.
    manual = ROOT / "routes" / "route2_manual_20261031"
    transition = np.array([166.3, 53.5])
    kink = np.array([151.0, 57.7])
    for key, inside, outside in (
            ("D5_to_A", [168.5, 165.3], [170.5, 163.8]),
            ("D5_to_D4", [168.5, 165.3], [170.5, 163.8]),
            ("A_to_D5", [164.5, 167.3], [163.0, 168.5]),
            ("D4_to_D5", [164.5, 167.3], [163.0, 168.5])):
        band = SAFETY.SafetyBand(manual / ("%s_safety_band.json" % key))
        on = lambda x: band.xy[int(np.argmin(np.abs(band.xy[:, 0] - x)))]
        assert band.station_speed_cap(transition) == pytest.approx(0.50), key
        for x in inside:
            assert band.station_speed_cap(on(x)) == pytest.approx(0.50), \
                (key, x)
        for x in outside:
            assert np.isinf(band.station_speed_cap(on(x))), (key, x)
        assert band.station_speed_cap(kink) == pytest.approx(0.60), key
        assert np.isinf(band.station_speed_cap(np.array([158.0, 55.6]))), key
        # 1-1.5 m before the zone the policy already asks for 0.5, so the
        # chair arrives slowed rather than braking inside the dip
        before = on(outside[0])
        assert MPC_SPEED.horizon_speed(band, before) <= 0.50 + 1e-9, key


def test_shipped_v6_v8_assets_are_cryptographically_bound():
    binding = ASSETS.validate_asset_binding(
        ROOT / "routes" / "20260812_route_v6_v8_waypoints.json",
        ROOT / "routes" / "20260812_route_v6_v8_safety_band.json",
        ROOT / "routes" / "route_2d_map_v9.yaml",
    )
    assert binding["route_id"].startswith("v6-v8:")
    band = SAFETY.SafetyBand(
        ROOT / "routes" / "20260812_route_v6_v8_safety_band.json"
    )
    assert band.route_centre_clearance_violations() == []
    assert band.route_centre_chord_violations() == []


def test_algorithm_default_assets_are_cryptographically_bound_and_clear():
    # The shipped default since 2026-08-16. It replaced the algorithm
    # route, which has 13 corners below TURN_FLOOR_SPEED and ended the
    # 08-15 drive at station 395; this one clears the curvature gate at
    # zero blocked stations.
    route = ROOT / "routes" / "20260816_route_v9_clearance_waypoints.json"
    band_path = (ROOT / "routes"
                 / "20260816_route_v9_clearance_safety_band.json")
    mask = ROOT / "routes" / "route_2d_map_v9.yaml"
    binding = ASSETS.validate_asset_binding(route, band_path, mask)
    assert binding["route_id"].startswith("v6-v9")
    route_data = json.loads(route.read_text(encoding="utf-8"))
    band_data = json.loads(band_path.read_text(encoding="utf-8"))
    assert route_data["count"] == 1988
    assert route_data["reference_point"] == "chair_centre"
    assert band_data["route_id"] == binding["route_id"]
    band = SAFETY.SafetyBand(band_path)
    assert band.route_centre_clearance_violations() == []
    assert band.route_centre_chord_violations() == []


def test_band_mismatch_fails_closed(tmp_path):
    route = ROOT / "routes" / "20260812_route_v6_v8_waypoints.json"
    band = json.loads(
        (ROOT / "routes" / "20260812_route_v6_v8_safety_band.json")
        .read_text(encoding="utf-8")
    )
    band["route_id"] = "wrong"
    changed = tmp_path / "changed-band.json"
    changed.write_text(json.dumps(band), encoding="utf-8")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        ASSETS.validate_asset_binding(route, changed)


def test_route_waypoint_mutation_fails_closed(tmp_path):
    route = json.loads(
        (ROOT / "routes" / "20260812_route_v6_v8_waypoints.json")
        .read_text(encoding="utf-8")
    )
    route["waypoints"][100]["x"] += 1000.0
    changed = tmp_path / "changed-route.json"
    changed.write_text(json.dumps(route), encoding="utf-8")
    with pytest.raises(ValueError, match="route content SHA-256 mismatch"):
        ASSETS.validate_asset_binding(
            changed,
            ROOT / "routes" / "20260812_route_v6_v8_safety_band.json",
        )


def test_mask_geometry_metadata_mismatch_fails_closed(tmp_path):
    route = ROOT / "routes" / "20260812_route_v6_v8_waypoints.json"
    band = ROOT / "routes" / "20260812_route_v6_v8_safety_band.json"
    source = ROOT / "routes" / "route_2d_map_v9.yaml"
    changed = tmp_path / "route_2d_map_v8.yaml"
    changed.write_text(
        source.read_text(encoding="utf-8").replace(
            "resolution: 0.05", "resolution: 0.10"
        ),
        encoding="utf-8",
    )
    (tmp_path / "route_2d_map_v8.pgm").write_bytes(
        (ROOT / "routes" / "route_2d_map_v8.pgm").read_bytes()
    )
    with pytest.raises(ValueError, match="metadata SHA-256 mismatch"):
        ASSETS.validate_asset_binding(route, band, changed)


def test_provenance_binds_rebuild_inputs():
    provenance = json.loads(
        (ROOT / "routes" / "20260812_route_v6_v8_provenance.json")
        .read_text(encoding="utf-8")
    )
    expected = provenance["source_sha256"]
    paths = {
        "preferred_v6_pgm": ROOT / "routes" / "route_2d_map_v6.pgm",
        "drivable_v8_pgm": ROOT / "routes" / "route_2d_map_v8.pgm",
        "seed_v6_route": ROOT / "routes" / "20260812_route_v6_waypoints.json",
        "seed_v6_band": ROOT / "routes" / "20260812_route_v6_safety_band.json",
    }
    for name, path in paths.items():
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected[name]

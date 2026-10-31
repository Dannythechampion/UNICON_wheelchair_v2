import hashlib
import importlib.util
import json
import math
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

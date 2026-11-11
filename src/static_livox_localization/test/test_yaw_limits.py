"""B1 (curvature cap) and B2 (yaw window = ramp slope), dwa_core.yaw_range.

11-09 evening climb: 35-40 % of DWA rows at 0.30-0.45 m/s sat at |w| >= 0.4
and swung through full reversals; nothing at >= 0.65 m/s. These pin the
limits that take the low-speed gain down without touching cruise.
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
from pytest import approx

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location("dwa_core_yaw", SCRIPTS / "dwa_core.py")
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)


def test_off_is_the_full_range():
    assert core.yaw_range(0.35, 0.2, 0.1) == (-0.5, 0.5)


def test_b1_caps_by_the_commanded_speed():
    lo, hi = core.yaw_range(0.35, 0.0, 0.1, curvature_max=0.9)
    assert hi == approx(0.315) and lo == approx(-0.315)
    # Cruise: the 0.5 rad/s limit binds first, B1 does nothing.
    assert core.yaw_range(0.6, 0.0, 0.1, curvature_max=0.9) == (-0.5, 0.5)


def test_b1_keeps_the_tightest_route_bend_available():
    """J corner: 2.5 m radius (kappa 0.40), entered at 0.6 m/s or slower."""
    for v in (0.35, 0.45, 0.6):
        assert core.yaw_range(v, 0.0, 0.1, curvature_max=0.9)[1] >= v / 2.5


def test_b2_is_a_window_round_the_last_command():
    lo, hi = core.yaw_range(0.6, -0.30, 0.1, yaw_accel=0.8)
    assert (lo, hi) == (approx(-0.38), approx(-0.22))


def test_both_intersect():
    lo, hi = core.yaw_range(0.35, -0.30, 0.1, curvature_max=0.9,
                            yaw_accel=0.8)
    assert (lo, hi) == (approx(-0.315), approx(-0.22))


def test_a_cap_below_a_held_turn_cuts_at_once():
    """Speed fell under a held -0.45: reducing |w| is never rate limited."""
    lo, hi = core.yaw_range(0.35, -0.45, 0.1, curvature_max=0.9,
                            yaw_accel=0.8)
    assert lo == hi == approx(-0.315)


def test_unrestricted_pairs_are_the_old_set():
    old = [(v, w) for v in core.speed_samples(0.8, current=0.5) if v > 0.0
           for w in core.yaw_samples()]
    new = core.candidate_pairs(0.8, 0.5)
    assert [(a, approx(b)) for a, b in old] == new


def test_restricted_pairs_stay_inside_and_include_the_edges():
    pairs = core.candidate_pairs(
        0.8, 0.5, yaw_range_for=lambda v: core.yaw_range(
            v, -0.30, 0.1, curvature_max=0.9, yaw_accel=0.8))
    ws = sorted({w for _, w in pairs})
    assert min(ws) == approx(-0.38) and max(ws) == approx(-0.22)
    assert all(-0.38 - 1e-9 <= w <= -0.22 + 1e-9 for w in ws)
    assert len(ws) >= 3


def test_rejected_yaws_still_apply():
    pairs = core.candidate_pairs(0.8, 0.5, rejected=(-0.3,),
                                 yaw_range_for=lambda v: (-0.35, -0.25))
    assert all(abs(w + 0.3) > 1e-6 for _, w in pairs)

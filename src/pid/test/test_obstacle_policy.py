"""Unit tests for forward obstacle policy (1.5 m stop / 3.0 m slow) and
0.50-1.00 m near-object pre-slowdown."""
import sys
from pathlib import Path

import pytest


PID_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(PID_SCRIPTS))
import obstacle_policy as op


def test_forward_stop_and_slow_band():
    assert op.FRONT_STOP_M == pytest.approx(1.5)
    assert op.FRONT_SLOW_M == pytest.approx(3.0)

    # Beyond 3.0 m -> clear
    assert op.obstacle_restriction(3.5, "moving", False, 0.80, 0.40) == (0.80, False, "NONE")
    # At 3.0 m -> top of linear slowdown (0.80 m/s)
    cap, stop, reason = op.obstacle_restriction(3.0, "moving", False, 0.80, 0.40)
    assert not stop and cap == pytest.approx(0.80) and reason == "PATH_OBSTACLE"
    # Midpoint 2.25 m -> 0.60 m/s
    cap, stop, reason = op.obstacle_restriction(2.25, "moving", False, 0.80, 0.40)
    assert not stop and cap == pytest.approx(0.60) and reason == "PATH_OBSTACLE"
    # Just above 1.5 m -> ~0.40 m/s
    cap, stop, reason = op.obstacle_restriction(1.51, "moving", False, 0.80, 0.40)
    assert not stop and cap == pytest.approx(0.4027, abs=0.005)
    # At/below 1.5 m -> STOP
    assert op.obstacle_restriction(1.5, "moving", False, 0.80, 0.40) == (0.0, True, "PATH_OBSTACLE")


def test_near_object_linear_pre_slowdown_050_to_100():
    assert op.NEAR_STOP_GAP_M == pytest.approx(0.50)
    assert op.NEAR_SLOW_GAP_M == pytest.approx(1.00)

    assert op.near_object_speed_cap(None, 0.80, 0.40) == pytest.approx(0.80)
    assert op.near_object_speed_cap(1.20, 0.80, 0.40) == pytest.approx(0.80)
    assert op.near_object_speed_cap(1.00, 0.80, 0.40) == pytest.approx(0.80)
    assert op.near_object_speed_cap(0.75, 0.80, 0.40) == pytest.approx(0.60)
    assert op.near_object_speed_cap(0.50, 0.80, 0.40) == pytest.approx(0.40)
    assert op.near_object_speed_cap(0.49, 0.80, 0.40) == pytest.approx(0.0)


def test_receding_and_static_detour_with_new_3m_slow_band():
    # Receding object in 1.5-3.0 m is ignored; inside 1.5 m is slowed to 0.40
    assert op.obstacle_restriction(2.8, "moving", True, 0.80, 0.40, receding_enabled=True) == (0.80, False, "RECEDING_IGNORED")
    assert op.obstacle_restriction(1.2, "moving", True, 0.80, 0.40, receding_enabled=True) == (0.40, False, "RECEDING_NEAR_SLOW")
    # Static detour triggers at 3.0 m
    assert op.obstacle_restriction(2.9, "static", False, 0.80, 0.40, static_detour_enabled=True) == (0.0, True, "STATIC_PATH_OBSTACLE")

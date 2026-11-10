"""Launch from rest: through the deadband in one step, and straight until
both wheels turn.

The 11-09 climb restarts (straight_launch's docstring) are the cases these
pin: a ramp that crawled 0.02 -> 0.34 m/s with a yaw rate attached, while
one wheel came free before the other.
"""

import importlib.util
from pathlib import Path

from pytest import approx


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    spec = importlib.util.spec_from_file_location(
        name, SCRIPTS / ("%s.py" % name))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


sl = load("straight_launch")
tip = load("tip_guard_policy")


# ------------------------------------------------------------- D: the floor

def test_from_rest_the_command_goes_straight_to_the_floor():
    assert sl.launch_speed(0.0, 0.8) == approx(0.35)
    assert sl.launch_speed(0.0, 0.35) == approx(0.35)


def test_a_partly_ramped_command_is_lifted_to_the_floor():
    assert sl.launch_speed(0.12, 0.6) == approx(0.35)


def test_the_floor_never_applies_to_a_stop_or_above_it():
    assert sl.launch_speed(0.0, 0.0) is None
    assert sl.launch_speed(0.2, 0.0) is None
    assert sl.launch_speed(0.4, 0.8) is None
    assert sl.launch_speed(0.0, 0.3) is None


def test_floor_zero_is_the_old_ramp():
    assert sl.launch_speed(0.0, 0.8, floor=0.0) is None


def test_tip_guard_launches_at_the_floor_and_ramps_above_it():
    first = tip.next_linear_speed(0.0, 0.8, 0.30, 0.02, False,
                                  launch_floor=0.35)
    assert first == approx(0.35)
    second = tip.next_linear_speed(first, 0.8, 0.30, 0.02, False,
                                   launch_floor=0.35)
    assert second == approx(0.35 + 0.30 * 0.02)


def test_tip_guard_ramps_a_request_under_the_floor_as_before():
    assert tip.next_linear_speed(0.0, 0.2, 0.30, 0.1, False,
                                 launch_floor=0.35) == approx(0.03)


def test_tip_guard_floor_never_delays_a_stop():
    assert tip.next_linear_speed(0.5, 0.0, 0.30, 0.02, False,
                                 launch_floor=0.35) == 0.0
    assert tip.next_linear_speed(0.0, 0.8, 0.30, 0.02, True,
                                 launch_floor=0.35) == 0.0


def test_tip_guard_floor_does_not_brake_through_itself():
    """Slowing from 0.5 toward 0.4 is ordinary braking, untouched."""
    out = tip.next_linear_speed(0.5, 0.4, 0.30, 0.02, False,
                                launch_floor=0.35)
    assert out == approx(0.46)


def test_tip_guard_default_is_the_old_behaviour():
    assert tip.next_linear_speed(0.0, 1.0, 0.3, 0.1, False) == approx(0.03)


# ---------------------------------------------------- A: straight launch

def launch_at_rest():
    launch = sl.StraightLaunch()
    assert launch.eligible(0.0, 0.0, 0.0, True, True)
    return launch


def test_holds_yaw_until_both_wheels_turn():
    launch = launch_at_rest()
    assert launch.update(0.0, 0.0, 0.8, 0.0, 0.0, True, True)
    # 18:52:35: right free, left not - still held.
    assert launch.update(0.3, 0.35, 0.8, 0.00, 0.11, True, True)
    assert launch.update(0.6, 0.35, 0.8, 0.10, 0.30, True, True)
    assert not launch.update(0.9, 0.35, 0.8, 0.26, 0.31, True, True)
    assert launch.last_exit == "wheels turning"


def test_releases_after_the_timeout_even_if_a_wheel_never_turns():
    launch = launch_at_rest()
    assert launch.update(0.0, 0.0, 0.8, 0.0, 0.0, True, True)
    assert launch.update(1.9, 0.35, 0.8, 0.0, 0.2, True, True)
    assert not launch.update(2.0, 0.35, 0.8, 0.0, 0.2, True, True)
    assert launch.last_exit == "timeout"


def test_does_not_restart_once_released_while_still_moving():
    launch = launch_at_rest()
    launch.update(0.0, 0.0, 0.8, 0.0, 0.0, True, True)
    launch.update(0.5, 0.35, 0.8, 0.3, 0.3, True, True)
    assert not launch.eligible(0.40, 0.3, 0.3, True, True)


def test_not_while_going_round_something():
    launch = sl.StraightLaunch()
    assert not launch.eligible(0.0, 0.0, 0.0, True, False)
    assert not launch.update(0.0, 0.0, 0.8, 0.0, 0.0, True, False)


def test_going_round_mid_launch_hands_steering_back():
    launch = launch_at_rest()
    assert launch.update(0.0, 0.0, 0.8, 0.0, 0.0, True, True)
    assert not launch.update(0.2, 0.35, 0.8, 0.0, 0.0, True, False)


def test_not_without_a_fresh_wheel_report():
    launch = sl.StraightLaunch()
    assert not launch.eligible(0.0, 0.0, 0.0, False, True)


def test_not_while_the_wheels_are_still_rolling():
    """Resuming from a short gate stop rolls on; that is not a launch."""
    launch = sl.StraightLaunch()
    assert not launch.eligible(0.0, 0.4, 0.4, True, True)


def test_a_stop_target_ends_the_launch():
    launch = launch_at_rest()
    assert launch.update(0.0, 0.0, 0.8, 0.0, 0.0, True, True)
    assert not launch.update(0.1, 0.35, 0.0, 0.0, 0.0, True, True)
    assert not launch.active


def test_disabled_never_holds():
    launch = sl.StraightLaunch(enabled=False)
    assert not launch.eligible(0.0, 0.0, 0.0, True, True)
    assert not launch.update(0.0, 0.0, 0.8, 0.0, 0.0, True, True)

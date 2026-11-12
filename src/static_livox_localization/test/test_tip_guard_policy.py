import pytest
import importlib.util
from pathlib import Path


POLICY_PATH = Path(__file__).parents[1] / "scripts" / "tip_guard_policy.py"
SPEC = importlib.util.spec_from_file_location("tip_guard_policy", POLICY_PATH)
POLICY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(POLICY)


def test_zero_request_is_an_immediate_stop_authority():
    assert POLICY.next_linear_speed(1.2, 0.0, 0.3, 0.02, False) == 0.0
    assert POLICY.next_linear_speed(-0.2, 0.0, 0.3, 0.02, False) == 0.0


def test_internal_trip_or_staleness_stops_immediately():
    assert POLICY.next_linear_speed(1.2, 1.2, 0.3, 0.02, True) == 0.0


def test_output_cannot_exceed_the_final_stage_limit():
    limit = POLICY.ABSOLUTE_V_LIMIT
    assert POLICY.next_linear_speed(limit, 99.0, 99.0, 1.0, False) == limit
    assert POLICY.next_linear_speed(-limit, -99.0, 99.0, 1.0, False) == -limit


def test_nonzero_acceleration_remains_rate_limited():
    assert POLICY.next_linear_speed(0.0, 1.0, 0.3, 0.1, False) == 0.03


# ------------------------------------------- no-reverse yaw clamp (11-11)

def encode_inner(v, w, sep=0.54):
    """The base's encoding of each wheel (km/h counts, 87 = reverse)."""
    out = []
    for s in (v - w * sep / 2.0, v + w * sep / 2.0):
        kmh = s * 3.6
        out.append(67 if kmh > 0 else (87 if kmh < 0 else 83))
    return out


def test_ordinary_turns_pass_untouched():
    assert POLICY.no_reverse_yaw(0.8, 0.5, 0.54) == 0.5
    assert POLICY.no_reverse_yaw(0.35, -0.3, 0.54) == -0.3


def test_the_recorded_crest_command_loses_its_reverse_wheel():
    """18:45:51.6 sent 0.01 / +0.05: the left wheel encoded as reverse."""
    assert 87 in encode_inner(0.01, 0.05)
    limited = POLICY.no_reverse_yaw(0.01, 0.05, 0.54)
    assert 87 not in encode_inner(0.01, limited)
    assert 0.0 <= limited <= 0.05


# --------------------------------------- resume from the wheels (11-11)

def test_after_a_zero_the_ramp_resumes_from_the_rolling_wheels():
    """18:53:16.4: zero, then 0.64 requested with the wheels at 0.58; the
    old ramp put 0.006 m/s on the wire, a near-stop under a rolling chair."""
    current = POLICY.next_linear_speed(0.7, 0.0, 0.30, 0.02, False)
    assert current == 0.0
    current = POLICY.resume_speed(current, 0.64, 0.58)
    assert current == pytest.approx(0.58)
    out = POLICY.next_linear_speed(current, 0.64, 0.30, 0.02, False,
                                   launch_floor=0.35)
    assert out >= 0.58


def test_resume_never_exceeds_the_request():
    assert POLICY.resume_speed(0.0, 0.35, 0.6) == pytest.approx(0.35)


def test_resume_does_nothing_at_rest_stale_or_when_stopping():
    assert POLICY.resume_speed(0.0, 0.6, None) == 0.0
    assert POLICY.resume_speed(0.0, 0.0, 0.6) == 0.0
    assert POLICY.resume_speed(0.7, 0.8, 0.6) == 0.7


def test_a_stop_carries_no_yaw():
    assert POLICY.no_reverse_yaw(0.0, 0.3, 0.54) == 0.0
    assert POLICY.no_reverse_yaw(-0.1, 0.3, 0.54) == 0.0


def test_the_bound_is_exactly_the_inner_wheel_standing_still():
    w = POLICY.no_reverse_yaw(0.2, 5.0, 0.54)
    assert w == pytest.approx(2 * 0.2 / 0.54, rel=1e-5)
    assert 0.2 - w * 0.27 >= 0.0


def test_non_finite_input_is_no_yaw():
    assert POLICY.no_reverse_yaw(float("nan"), 0.2, 0.54) == 0.0
    assert POLICY.no_reverse_yaw(0.5, float("inf"), 0.54) == 0.0

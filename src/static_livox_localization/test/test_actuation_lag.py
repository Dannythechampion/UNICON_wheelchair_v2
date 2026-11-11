"""The Smith predictor that replaces the fixed LATENCY_S lead."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def load(name):
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            name, SCRIPTS / (name + ".py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


lag = load("actuation_lag")


def feed(predictor, start_s, end_s, speed, yaw_rate, hz=50.0):
    t = start_s
    while t < end_s - 1e-9:
        predictor.on_command(t, speed, yaw_rate)
        t += 1.0 / hz
    return t


def drive(predictor, start_s, end_s, command, measured):
    """Commands at 50 Hz and a prediction at 10 Hz, in time order, as the
    follower sees them. Returns the last prediction."""
    led, t = None, start_s
    while t < end_s - 1e-9:
        feed(predictor, t, t + 0.1, *command)
        t += 0.1
        led = predictor.predict([0.0, 0.0, 0.0] + list(measured),
                                measured[0], measured[1], t)
    return led


def test_a_held_command_leads_along_the_arc_it_is_on():
    predictor = lag.SmithPredictor()
    led = drive(predictor, 0.0, 5.0, (0.8, 0.0), (0.8, 0.0))

    assert led[0] == pytest.approx(0.8 * predictor.horizon_s, abs=0.02)
    assert led[1] == pytest.approx(0.0, abs=1e-6)
    assert led[2] == pytest.approx(0.0, abs=1e-6)
    # Speed and yaw rate stay as measured: the ramp resyncs from them.
    assert led[3:] == [0.8, 0.0]


def test_a_reversed_turn_is_not_projected_onward():
    """The lead extends the measured turn; the commands say it is ending.

    The chair is still turning left at 0.4 rad/s on what was sent 0.6 s
    ago, but for the last 0.3 s the command has been a right turn. The
    lead (0.55 s on the measured rate) projects another 0.22 rad left.
    """
    predictor = lag.SmithPredictor()
    drive(predictor, 0.0, 3.0, (0.8, 0.4), (0.8, 0.4))
    led = drive(predictor, 3.0, 3.3, (0.8, -0.4), (0.8, 0.4))

    lead_heading = 0.4 * 0.55
    assert led[2] < 0.5 * lead_heading
    assert led[2] < 0.4 * predictor.horizon_s


def test_by_default_the_model_error_is_not_carried():
    """A slope: nothing commanded, the base drifts at 0.1 rad/s anyway.

    Without the error term the prediction follows the commands alone; the
    drift shows up in the next cycle's measured pose instead.
    """
    predictor = lag.SmithPredictor()
    led = drive(predictor, 0.0, 6.0, (0.8, 0.0), (0.8, 0.1))

    assert predictor.residual == [0.0, 0.0]
    # 0.041 rad with the error term on; what is left is the cold start's
    # measured 0.1 rad/s decaying out of the model.
    assert led[2] == pytest.approx(0.0, abs=1e-6)


def test_the_model_error_is_carried_when_asked_for():
    predictor = lag.SmithPredictor(residual_tau_s=0.4)
    led = drive(predictor, 0.0, 6.0, (0.8, 0.0), (0.8, 0.1))

    assert predictor.residual[1] == pytest.approx(0.1, abs=0.01)
    assert led[2] == pytest.approx(0.1 * predictor.horizon_s, abs=0.005)


def test_a_cold_start_begins_from_the_measurement():
    predictor = lag.SmithPredictor()
    led = predictor.predict([1.0, 2.0, 0.5, 0.6, 0.2], 0.6, 0.2, 10.0)

    assert predictor.residual == [0.0, 0.0]
    # No commands recorded: the model decays toward zero from measured.
    assert 0.0 < led[2] - 0.5 < 0.2 * predictor.horizon_s
    assert math.hypot(led[0] - 1.0, led[1] - 2.0) < 0.6 * predictor.horizon_s


def test_a_long_gap_restarts_the_model():
    predictor = lag.SmithPredictor(residual_tau_s=0.4)
    drive(predictor, 0.0, 3.0, (0.8, 0.0), (0.8, 0.3))
    assert predictor.residual[1] > 0.2

    predictor.predict([0.0, 0.0, 0.0, 0.0, 0.0], 0.0, 0.0,
                      3.0 + lag.HISTORY_S + 1.0)

    assert predictor.residual == [0.0, 0.0]


def test_a_clock_that_runs_backwards_drops_the_history():
    predictor = lag.SmithPredictor()
    feed(predictor, 100.0, 101.0, 0.8, 0.3)
    predictor.on_command(5.0, 0.0, 0.0)

    assert predictor.stamps == [5.0]
    assert predictor.model_s is None


def test_history_is_bounded():
    predictor = lag.SmithPredictor()
    feed(predictor, 0.0, 30.0, 0.8, 0.0)

    assert predictor.stamps[-1] - predictor.stamps[0] <= lag.HISTORY_S


def test_residual_weight_scales_only_what_is_carried_over_the_horizon():
    """The follower applies the model error in full under the deadband and
    not at cruise; the error itself keeps being tracked either way."""
    full = lag.SmithPredictor(residual_tau_s=0.4)
    none = lag.SmithPredictor(residual_tau_s=0.4)
    led_full = drive(full, 0.0, 6.0, (0.3, 0.0), (0.3, 0.1))
    t = 6.0
    led_none = None
    t2 = 0.0
    while t2 < 6.0 - 1e-9:
        feed(none, t2, t2 + 0.1, 0.3, 0.0)
        t2 += 0.1
        led_none = none.predict([0.0, 0.0, 0.0, 0.3, 0.1], 0.3, 0.1, t2,
                                residual_weight=0.0)
    assert none.residual[1] == pytest.approx(full.residual[1], abs=1e-9)
    assert led_full[2] == pytest.approx(0.1 * full.horizon_s, abs=0.005)
    assert led_none[2] == pytest.approx(0.0, abs=1e-6)


def test_default_weight_is_the_old_prediction():
    a = lag.SmithPredictor(residual_tau_s=0.4)
    b = lag.SmithPredictor(residual_tau_s=0.4)
    la = drive(a, 0.0, 3.0, (0.8, 0.2), (0.8, 0.25))
    t = 0.0
    while t < 3.0 - 1e-9:
        feed(b, t, t + 0.1, 0.8, 0.2)
        t += 0.1
        lb = b.predict([0.0, 0.0, 0.0, 0.8, 0.25], 0.8, 0.25, t,
                       residual_weight=1.0)
    assert list(la) == pytest.approx(list(lb))

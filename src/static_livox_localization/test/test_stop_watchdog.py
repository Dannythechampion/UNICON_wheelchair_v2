"""A commanded stop that the wheels ignore has to be caught inside a second.

The fault it watches for ran 3.73 s on 2026-08-16 and 12.97 s on 08-19, the
second one spinning the chair about 2.9 times on the spot after it had
finished the route. Both were visible on /wheel_status the whole time and
nothing was looking.

The command stream runs at 50 Hz on the real chair, so these tests feed it
that way. A test that issues one frame and then jumps a second forward is
exercising the stale-command branch, which belongs to uart.py's watchdog.
"""

import importlib.util
from pathlib import Path


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    spec = importlib.util.spec_from_file_location(
        name, SCRIPTS / ("%s.py" % name))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


sw = load("stop_watchdog")
S, C = ord("S"), ord("C")
# 0x21 is the zero of the protocol; 0x3B decodes to 0.72 m/s, the speed
# the left wheel held for 12.97 s on 2026-08-19.
STILL, ROLLING = 0x21, 0x3B


def stop_frame():
    return [S, STILL, S, STILL, ord("O")]


def drive_frame():
    return [C, ROLLING, C, ROLLING, ord("O")]


def status(left_byte, right_byte, mode=sw.AUTO_MODE):
    return [72, mode, C, left_byte, C, right_byte, ord("O"), 88, 0, 13, 10]


def stream(check, command, left, right, start_s, duration_s,
           mode=sw.AUTO_MODE, hz=50.0):
    """Run the bus for a while and collect whatever the check reports."""
    reasons = []
    step = 1.0 / hz
    now = start_s
    while now <= start_s + duration_s + 1e-9:
        check.observe_command(command(), now)
        reason = check.observe_status(status(left, right, mode), now, mode)
        if reason:
            reasons.append((now, reason))
        now += step
    return reasons


def test_a_stop_the_wheels_obey_raises_nothing():
    check = sw.StopHonouredCheck()
    assert stream(check, stop_frame, STILL, STILL, 0.0, 5.0) == []


def test_one_wheel_turning_while_the_other_is_stopped_is_caught_at_once():
    """This fixture is a split, not a coast-down.

    It used to be read as one: the test asserted that 0.72 m/s on the left
    against a stopped right wheel was innocent for the first 0.4 s, on the
    grounds that wheels take time to stop. Wheels do - but not one of them
    while the other is already at rest. The chair is pivoting at 77 deg/s
    the whole time that is true, so the grace does not apply to it.
    """
    check = sw.StopHonouredCheck()
    caught = stream(check, stop_frame, ROLLING, STILL, 0.0, 0.5)
    assert len(caught) == 1
    assert caught[0][0] <= 0.2, "a pivot is not waited out"
    assert "one wheel ignored a stop" in caught[0][1]


def test_both_wheels_holding_past_the_envelope_are_a_fault():
    """No split here - both wheels hold the same speed, so the envelope is
    what has to notice, and it does."""
    check = sw.StopHonouredCheck()
    caught = stream(check, stop_frame, ROLLING, ROLLING, 0.0, 3.0)
    assert len(caught) == 1
    assert "not slowing" in caught[0][1]


def test_it_reports_once_and_not_every_frame():
    """An alarm at 100 Hz is not an alarm."""
    check = sw.StopHonouredCheck(grace_s=0.4)
    assert len(stream(check, stop_frame, ROLLING, STILL, 0.0, 3.0)) == 1


def test_it_arms_again_after_the_wheels_finally_stop():
    check = sw.StopHonouredCheck(grace_s=0.4)
    assert len(stream(check, stop_frame, ROLLING, STILL, 0.0, 1.0)) == 1
    assert stream(check, stop_frame, STILL, STILL, 1.0, 0.5) == []
    assert stream(check, drive_frame, ROLLING, ROLLING, 1.5, 0.5) == []
    assert len(stream(check, stop_frame, ROLLING, STILL, 2.0, 1.0)) == 1


def test_manual_mode_is_not_its_business():
    """In manual the base takes the joystick and ignores our frames, so a
    turning wheel under a stop command is the operator driving."""
    check = sw.StopHonouredCheck(grace_s=0.4)
    assert stream(check, stop_frame, ROLLING, ROLLING, 0.0, 3.0,
                  mode=sw.MANUAL_MODE) == []


def test_a_stale_command_stream_belongs_to_the_uart_watchdog():
    """Starved commands are a different fault with a different owner; this
    node must not also claim it, or one cause raises two alarms."""
    check = sw.StopHonouredCheck(grace_s=0.4, command_fresh_s=0.3)
    check.observe_command(stop_frame(), 0.0)
    assert check.observe_status(status(ROLLING, STILL), 1.0, sw.AUTO_MODE) is None


def test_a_drive_command_clears_the_arming():
    check = sw.StopHonouredCheck(grace_s=0.4)
    assert stream(check, drive_frame, ROLLING, ROLLING, 0.0, 3.0) == []


def test_the_08_19_numbers_are_caught_inside_a_second():
    """Left byte 0x37 is 0.72 m/s, right stopped - the frame pair that ran
    for 12.97 s. It has to be reported inside the grace plus one cycle."""
    check = sw.StopHonouredCheck()
    reasons = stream(check, stop_frame, ROLLING, STILL, 100.0, 13.0)
    assert len(reasons) == 1
    when, reason = reasons[0]
    assert when - 100.0 < sw.GRACE_S + 0.05
    assert "left 0.72" in reason


def test_speed_decoding_matches_the_base_protocol():
    assert abs(sw.wheel_speed(0x21) - 0.0) < 1e-9
    assert abs(sw.wheel_speed(0x3B) - 0.722) < 0.002
    assert abs(sw.wheel_speed(0x37) - 0.611) < 0.002
    assert sw.commanded_stop([S, STILL, S, STILL, ord("O")])
    assert not sw.commanded_stop([C, ROLLING, S, STILL, ord("O")])
    assert sw.reported_motion(status(ROLLING, STILL))[0] > 0.7
    assert sw.reported_motion([72, 65]) is None


# The stop that ended the 21:29 run, frame by frame from
# blackbox_20260823_212210 at 10 Hz. The profile saw a pedestrian, chose to
# wait, and commanded the stop; the wheels were shedding about 0.47 m/s^2,
# which is everything this chair has. The watchdog called it a fault at
# 0.40 s and took the chair off auto in front of the person.
PEDESTRIAN_STOP_MPS = [0.75, 0.69, 0.69, 0.67, 0.64, 0.53, 0.44, 0.39, 0.33,
                       0.25, 0.25, 0.22, 0.14, 0.14, 0.14, 0.14, 0.14, 0.00]

# 2026-08-19, the worst recorded fault: the left wheel held its last
# setpoint for 12.97 s while the right sat at zero.
IGNORED_STOP_MPS = [0.72] * 40

# 2026-08-16. Inertia cannot do this, which is why it needs no envelope to
# be recognised.
SPED_UP_AFTER_STOP_MPS = [0.78, 0.83, 0.90, 0.94, 0.94, 0.94]


def run_trace(speeds, step_s=0.1, mode=sw.AUTO_MODE, **kwargs):
    """Feed a stop command and a speed trace; collect what it says."""
    check = sw.StopHonouredCheck(**kwargs)
    said = []
    for index, speed in enumerate(speeds):
        now = index * step_s
        check.observe_command(stop_frame(), now)
        byte = int(round(speed * 36.0)) + 0x21
        reason = check.observe_status([72, mode, 83, byte, 83, byte],
                                      now, mode)
        if reason:
            said.append((now, reason))
    return said


def test_the_pedestrian_stop_is_not_a_fault():
    """The regression this rewrite exists for.

    A chair braking as hard as it can must never be taken off auto for it,
    least of all while it is stopping for someone standing in front of it.
    """
    assert run_trace(PEDESTRIAN_STOP_MPS) == []


def test_a_stop_the_base_ignores_is_still_a_fault():
    said = run_trace(IGNORED_STOP_MPS)
    assert len(said) == 1
    assert said[0][0] <= 1.5, "13 s of it should not take long to notice"
    assert "not slowing" in said[0][1]


def test_wheels_that_speed_up_after_a_stop_are_named_as_such():
    said = run_trace(SPED_UP_AFTER_STOP_MPS)
    assert len(said) == 1
    assert "sped up" in said[0][1]


def test_a_stop_from_cruise_has_room_that_a_stop_from_a_crawl_does_not():
    """The envelope is relative to the speed carried into the stop, so the
    same absolute reading is a fault at one entry speed and not at another.
    A flat grace cannot express that, which is how the old one went wrong."""
    from_crawl = run_trace([0.30] * 30)
    assert from_crawl, "0.30 m/s held for 3 s after a stop is a fault"
    braking = [max(0.0, 0.30 - 0.47 * k * 0.1) for k in range(20)]
    assert run_trace(braking) == []


def test_manual_mode_is_never_judged():
    assert run_trace(IGNORED_STOP_MPS, mode=sw.MANUAL_MODE) == []


def test_it_re_arms_once_the_wheels_actually_stop():
    check = sw.StopHonouredCheck()
    for index, speed in enumerate(IGNORED_STOP_MPS[:20]):
        check.observe_command(stop_frame(), index * 0.1)
        byte = int(round(speed * 36.0)) + 0x21
        check.observe_status([72, sw.AUTO_MODE, 83, byte, 83, byte],
                             index * 0.1, sw.AUTO_MODE)
    check.observe_command(stop_frame(), 2.0)
    still = check.observe_status([72, sw.AUTO_MODE, 83, 0x21, 83, 0x21],
                                 2.0, sw.AUTO_MODE)
    assert still is None
    assert check.latched is False, "a stop that was honoured re-arms it"


# The pivot alongside a pedestrian, 2026-08-23 21:36, read off
# blackbox_20260823_212210 at 10 Hz from the stop command onward.
# /wheel_cmd carried S0.00 / S0.00 for every one of these frames.
PIVOT_LEFT_RIGHT = [
    (0.72, 0.83), (0.75, 0.86), (0.75, 0.75), (0.72, 0.64), (0.72, 0.56),
    (0.67, 0.47), (0.67, 0.39), (0.64, 0.00), (0.64, 0.00), (0.67, 0.00),
    (0.64, 0.00), (0.67, 0.00), (0.69, 0.00), (0.64, 0.00), (0.56, 0.00),
]

# A stop taken out of a hard right turn: the wheels enter it 0.27 m/s
# apart and both shed speed together. Asymmetric, and not a fault.
TURNING_STOP_LEFT_RIGHT = [
    (0.66, 0.93), (0.61, 0.88), (0.52, 0.79), (0.43, 0.70), (0.33, 0.60),
    (0.24, 0.51), (0.14, 0.42), (0.05, 0.32), (0.00, 0.23), (0.00, 0.14),
    (0.00, 0.00),
]


def run_pairs(pairs, step_s=0.1, mode=sw.AUTO_MODE, **kwargs):
    check = sw.StopHonouredCheck(**kwargs)
    said = []
    for index, (left, right) in enumerate(pairs):
        now = index * step_s
        check.observe_command(stop_frame(), now)
        frame = [72, mode, 83, int(round(left * 36.0)) + 0x21,
                 83, int(round(right * 36.0)) + 0x21]
        reason = check.observe_status(frame, now, mode)
        if reason:
            said.append((now, reason))
    return said


def test_one_wheel_ignoring_a_stop_is_caught_while_it_is_still_pivoting():
    said = run_pairs(PIVOT_LEFT_RIGHT)
    assert len(said) == 1
    when, reason = said[0]
    assert "one wheel ignored a stop" in reason
    assert when <= 0.9, (
        "the right wheel was at rest from 0.7 s; waiting for the envelope "
        "cost another 0.4 s of turning (caught at %.2f s)" % when)


def test_the_pivot_report_says_how_fast_it_is_turning():
    """0.67 m/s across a 0.54 m track. An operator reading this needs to
    know it was a pivot and not merely a late stop."""
    reason = run_pairs(PIVOT_LEFT_RIGHT)[0][1]
    assert "deg/s of pivot" in reason
    assert "7" in reason.split("deg/s")[0][-4:]


def test_a_stop_out_of_a_turn_is_not_a_split():
    """Both wheels shedding speed together, 0.27 m/s apart because the
    chair was turning. Asymmetry alone must not be enough."""
    assert run_pairs(TURNING_STOP_LEFT_RIGHT) == []


def test_a_single_noisy_frame_is_not_a_split():
    trace = [(0.70, 0.70), (0.70, 0.00), (0.60, 0.55), (0.45, 0.42),
             (0.30, 0.28), (0.15, 0.14), (0.00, 0.00)]
    assert run_pairs(trace) == []


def test_the_pedestrian_stop_is_still_not_a_fault_under_the_split_test():
    assert run_pairs([(v, v) for v in PEDESTRIAN_STOP_MPS]) == []


def test_imu_cross_check_suppresses_brief_brake_stagger_false_alarm():
    """2026-11-19 false alarm shape: L=0.00, R=0.44 for 0.16 s while IMU is only 10 deg/s."""
    import math
    check = sw.StopHonouredCheck()
    low_imu = math.radians(10.0)
    said = []
    for k in range(25):  # 0.25 s at 100 Hz (< 0.50 s no-rot hold)
        t = k * 0.01
        check.observe_command(stop_frame(), t)
        check.observe_imu(low_imu, t)
        frame = [72, sw.AUTO_MODE, C, 0x21, C, int(round(0.44 * 36.0)) + 0x21]
        r = check.observe_status(frame, t, sw.AUTO_MODE)
        if r:
            said.append((t, r))
    assert said == []

    # If the same split persists past 0.50 s even with low IMU, it still trips.
    for k in range(25, 60):
        t = k * 0.01
        check.observe_command(stop_frame(), t)
        check.observe_imu(low_imu, t)
        frame = [72, sw.AUTO_MODE, C, 0x21, C, int(round(0.44 * 36.0)) + 0x21]
        r = check.observe_status(frame, t, sw.AUTO_MODE)
        if r:
            said.append((t, r))
    assert len(said) == 1
    assert said[0][0] >= 0.49


def test_imu_cross_check_catches_real_pivot_split_immediately():
    """2026-11-18 real spin: L=0.00, R=0.44 with IMU -140 deg/s catches inside 0.16 s."""
    import math
    check = sw.StopHonouredCheck()
    high_imu = math.radians(-140.0)
    said = []
    for k in range(30):
        t = k * 0.01
        check.observe_command(stop_frame(), t)
        check.observe_imu(high_imu, t)
        frame = [72, sw.AUTO_MODE, C, 0x21, C, int(round(0.44 * 36.0)) + 0x21]
        r = check.observe_status(frame, t, sw.AUTO_MODE)
        if r:
            said.append((t, r))
    assert len(said) == 1
    assert said[0][0] <= 0.17


def test_stationary_single_wheel_ghost_speed_is_not_an_alarm():
    """2026-11-18 10:55 ghost speed: L=0.00, R=0.31 for 3 s with IMU -0.6 deg/s."""
    import math
    check = sw.StopHonouredCheck()
    still_imu = math.radians(-0.6)
    said = []
    for k in range(300):
        t = k * 0.01
        check.observe_command(stop_frame(), t)
        check.observe_imu(still_imu, t)
        frame = [72, sw.AUTO_MODE, C, 0x21, C, int(round(0.31 * 36.0)) + 0x21]
        r = check.observe_status(frame, t, sw.AUTO_MODE)
        if r:
            said.append((t, r))
    assert said == []


def test_cmd_vel_zero_arms_stop_watch_before_wire_s():
    """Closing the 1.5 s ramp blind window: cmd_vel=0 with C 0.03 on wire still catches ignored stop."""
    check = sw.StopHonouredCheck()
    ramp_cmd = [C, 34, C, 34, ord("O")]
    said = []
    for k in range(150):  # 1.5 s
        t = k * 0.01
        check.observe_cmd_vel(0.0, 0.0, t)
        check.observe_command(ramp_cmd, t)
        r = check.observe_status(status(ROLLING, ROLLING), t, sw.AUTO_MODE)
        if r:
            said.append((t, r))
    assert len(said) == 1
    assert said[0][0] <= 1.05
    assert "not slowing" in said[0][1]


def test_imu_spin_check_catches_uncommanded_spin_in_auto():
    import math
    spin = sw.ImuSpinCheck()
    said = []
    for k in range(60):  # 0.30 s at 200 Hz
        t = k * 0.005
        spin.observe_cmd_vel(0.0, t)
        r = spin.observe_imu(math.radians(130.0), t, sw.AUTO_MODE)
        if r:
            said.append((t, r))
    assert len(said) == 1
    assert 0.195 <= said[0][0] <= 0.215
    assert "uncommanded spin" in said[0][1]


def test_imu_spin_check_ignores_brief_bump_transient():
    """Real Nov 12/16/19 road transients peaked at ~55-62 deg/s for ~0.15 s and must not trip."""
    import math
    spin = sw.ImuSpinCheck()
    for k in range(30):  # 0.15 s at 58 deg/s
        t = k * 0.005
        spin.observe_cmd_vel(0.0, t)
        assert spin.observe_imu(math.radians(58.0), t, sw.AUTO_MODE) is None


def test_imu_spin_check_ignores_single_stale_sample():
    import math
    spin = sw.ImuSpinCheck()
    # Same timestamp repeated must never trigger
    for _ in range(50):
        assert spin.observe_imu(math.radians(150.0), 1.0, sw.AUTO_MODE) is None


def test_imu_spin_check_ignores_normal_commanded_turn_and_manual():
    import math
    spin = sw.ImuSpinCheck()
    # Normal max AUTO turn (~34 deg/s commanded, 38 deg/s actual)
    for k in range(200):
        t = k * 0.005
        spin.observe_cmd_vel(math.radians(34.0), t)
        assert spin.observe_imu(math.radians(38.0), t, sw.AUTO_MODE) is None
    # Fast manual joystick spin (90 deg/s in MANUAL mode)
    for k in range(200, 400):
        t = k * 0.005
        assert spin.observe_imu(math.radians(90.0), t, sw.MANUAL_MODE) is None


def test_imu_spin_check_resets_after_dropout_gap():
    """Two 90 deg/s samples separated by a 0.35 s dropout gap must NOT trip."""
    import math
    spin = sw.ImuSpinCheck()
    assert spin.observe_imu(math.radians(90.0), 1.00, sw.AUTO_MODE) is None
    assert spin.observe_imu(math.radians(90.0), 1.35, sw.AUTO_MODE) is None


def test_both_wheels_rolling_at_0194_mps_is_still_caught():
    """Restoring MOVING_MPS=0.15 ensures both wheels rolling at 0.194 m/s is caught."""
    import math
    check = sw.StopHonouredCheck()
    said = []
    byte_0194 = int(round(0.194 * 36.0)) + 0x21
    for k in range(100):  # 10 s at 10 Hz
        t = k * 0.1
        check.observe_command(stop_frame(), t)
        check.observe_imu(math.radians(0.5), t)
        r = check.observe_status(status(byte_0194, byte_0194), t, sw.AUTO_MODE)
        if r:
            said.append((t, r))
    assert len(said) == 1
    assert "not slowing" in said[0][1] or "still rolling" in said[0][1]

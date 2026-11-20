"""Unit tests for the measured-speed two-stage FastStopRamp in wheel_cmd_tmp.py."""
import importlib.util
import math
import sys
import types
from pathlib import Path

import pytest


def find_runtime_wheel_cmd():
    tail = Path("docs") / "nuc_runtime_20261118" / "base_model" / "src" / "wheel_cmd_tmp.py"
    for parent in Path(__file__).resolve().parents:
        candidate = parent / tail
        if candidate.is_file():
            return candidate
    return None


TARGET = find_runtime_wheel_cmd()
if TARGET is None:
    pytest.skip("nuc_runtime_20261118 wheel_cmd_tmp.py not found", allow_module_level=True)


def load_module():
    for name, attrs in (
        ("rospy", ("Publisher", "Subscriber", "Time", "Timer", "Duration", "ROSInitException")),
        ("std_msgs", ()),
        ("std_msgs.msg", ("Int16MultiArray",)),
        ("geometry_msgs", ()),
        ("geometry_msgs.msg", ("Twist",)),
        ("sensor_msgs", ()),
        ("sensor_msgs.msg", ("Imu",)),
    ):
        module = types.ModuleType(name)
        for attr in attrs:
            setattr(module, attr, type(attr, (), {}))
        sys.modules.setdefault(name, module)
    sys.modules["rospy"].init_node = lambda *a, **k: None
    spec = importlib.util.spec_from_file_location("wheel_cmd_tmp_fast", TARGET)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


wheel = load_module()


def decode_cmd(cmd):
    if tuple(cmd) == wheel.STOP_COMMAND:
        return ("S", 0.0, "S", 0.0)
    l_mps = wheel.wheel_speed_mps(cmd[0], cmd[1])
    r_mps = wheel.wheel_speed_mps(cmd[2], cmd[3])
    return (chr(cmd[0]), l_mps, chr(cmd[2]), r_mps)


def test_one_wheel_stopped_immediately_sends_s():
    """L=0.00, R=0.44 must NEVER send C 0.03 to the stopped wheel."""
    ramp = wheel.FastStopRamp()
    assert tuple(ramp.command(10.0, 0.00, 0.44)) == wheel.STOP_COMMAND
    assert ramp.done is True

    ramp2 = wheel.FastStopRamp()
    assert tuple(ramp2.command(10.0, 0.44, 0.05)) == wheel.STOP_COMMAND
    assert ramp2.done is True


def test_reverse_or_rollback_immediately_sends_s():
    ramp = wheel.FastStopRamp()
    assert tuple(ramp.command(10.0, -0.10, 0.40)) == wheel.STOP_COMMAND
    ramp2 = wheel.FastStopRamp()
    assert tuple(ramp2.command(10.0, -0.20, -0.20)) == wheel.STOP_COMMAND


def test_already_slow_immediately_sends_s():
    ramp = wheel.FastStopRamp()
    assert tuple(ramp.command(10.0, 0.15, 0.14)) == wheel.STOP_COMMAND
    assert ramp.done is True


def test_straight_stop_goes_immediately_to_final_003_then_s():
    """Straight stop (|L-R| <= 0.08) skips 0.20 pause and commands C 0.03 from tick 1."""
    ramp = wheel.FastStopRamp()
    c1 = ramp.command(0.0, 0.80, 0.78)
    assert ramp.stage == "final"
    assert c1[0] == 67 and c1[2] == 67
    assert c1[1] == 34 and c1[3] == 34  # C 0.03

    # Still > 0.15 on faster wheel -> stays on C 0.03
    c2 = ramp.command(0.5, 0.22, 0.14)
    assert tuple(c2) != wheel.STOP_COMMAND
    assert c2[1] == 34 and c2[3] == 34

    # Both <= 0.15 -> S
    c3 = ramp.command(0.9, 0.15, 0.12)
    assert tuple(c3) == wheel.STOP_COMMAND
    assert ramp.done is True


def test_corner_stop_aligns_at_020_then_003_then_s():
    """Turning stop (0.80, 0.50): stage 1 commands ~0.20 until |L-R|<=0.08, then 0.03, then S<=0.15."""
    ramp = wheel.FastStopRamp()

    # Stage 1: L=0.80, R=0.50 (diff 0.30 > 0.08) -> both commanded ~0.20
    c1 = ramp.command(0.0, 0.80, 0.50)
    dL, vL, dR, vR = decode_cmd(c1)
    assert dL == "C" and dR == "C"
    assert vL == pytest.approx(vR)
    assert vL == pytest.approx(0.20, abs=0.02)
    assert ramp.stage == "align"

    # R reaches 0.20 while L is still 0.45 (diff 0.25 > 0.08) -> stays in align, holds ~0.20
    c2 = ramp.command(0.40, 0.45, 0.20)
    _, vL2, _, vR2 = decode_cmd(c2)
    assert vL2 == pytest.approx(0.20, abs=0.02)
    assert vR2 == pytest.approx(0.20, abs=0.02)
    assert ramp.stage == "align"

    # Once |L-R| <= 0.08 (even if max is 0.28 > 0.25) -> immediately transitions to final (C 0.03)
    c3 = ramp.command(0.75, 0.28, 0.20)
    dL3, vL3, dR3, vR3 = decode_cmd(c3)
    assert ramp.stage == "final"
    assert dL3 == "C" and dR3 == "C"
    assert c3[1] == 34 and c3[3] == 34  # 0.1 km/h = 0.0278 m/s
    assert vL3 == pytest.approx(0.0278, abs=0.005)

    # Even if speeds slightly diverge again, never reverts to align
    c4 = ramp.command(0.80, 0.26, 0.16)
    assert ramp.stage == "final"
    assert c4[1] == 34 and c4[3] == 34

    # Still one wheel above 0.15 -> stays on C 0.03 (max rule, NOT min rule)
    c5 = ramp.command(0.85, 0.18, 0.12)
    assert tuple(c5) != wheel.STOP_COMMAND
    assert c5[1] == 34 and c5[3] == 34

    # Both wheels <= 0.15 -> sends S and latches done
    c6 = ramp.command(0.90, 0.14, 0.12)
    assert tuple(c6) == wheel.STOP_COMMAND
    assert ramp.done is True


def test_never_accelerates_wheel_slower_than_align():
    """If R=0.14 < 0.20 while L=0.35, command must NOT exceed 0.14."""
    ramp = wheel.FastStopRamp()
    cmd = ramp.command(0.0, 0.35, 0.14)
    _, vL, _, vR = decode_cmd(cmd)
    assert vL <= 0.14 + 0.015
    assert vR <= 0.14 + 0.015
    assert ramp.stage == "align"


def test_align_timeout_and_overall_timeout_force_s():
    """Ghost wheel speed stuck above 0.15 must advance to final at 1.0s and force S at 1.5s."""
    ramp = wheel.FastStopRamp()
    assert ramp.command(0.0, 0.50, 0.22)[1] == 40  # ~0.20 m/s
    assert ramp.stage == "align"

    # At 1.0s -> switches to final (C 0.03)
    c_final = ramp.command(1.0, 0.35, 0.22)
    assert ramp.stage == "final"
    assert c_final[1] == 34

    # At 1.5s -> forces S even though measured speed is still stuck
    c_stop = ramp.command(1.5, 0.30, 0.22)
    assert tuple(c_stop) == wheel.STOP_COMMAND
    assert ramp.done is True


def test_imu_spin_aborts_immediately_even_when_both_wheels_forward():
    """If both wheels report C>0.06 during a spin, IMU >= 50 deg/s for 0.10s aborts to S."""
    ramp = wheel.FastStopRamp()
    high_wz = math.radians(60.0)

    # Brief spike (< 0.10s) does not abort
    c1 = ramp.command(0.00, 0.60, 0.50, imu_wz_rad_s=high_wz)
    assert tuple(c1) != wheel.STOP_COMMAND
    c2 = ramp.command(0.06, 0.58, 0.48, imu_wz_rad_s=high_wz)
    assert tuple(c2) != wheel.STOP_COMMAND
    # Drop back below threshold resets timer
    c3 = ramp.command(0.08, 0.56, 0.46, imu_wz_rad_s=math.radians(10.0))
    assert tuple(c3) != wheel.STOP_COMMAND

    # Sustained >= 0.10s with 50Hz distinct IMU timestamps aborts straight to S
    aborted = False
    for k in range(6):  # 0.10, 0.12, 0.14, 0.16, 0.18, 0.20
        ts = 0.10 + k * 0.02
        cmd = ramp.command(ts, 0.54, 0.44, imu_wz_rad_s=high_wz, imu_stamp=ts)
        if tuple(cmd) == wheel.STOP_COMMAND:
            aborted = True
            assert ts >= 0.199
            break
    assert aborted and ramp.done is True


def test_single_stale_imu_sample_does_not_abort():
    """One 60 deg/s IMU frame reused across 50Hz timer ticks must NEVER count as 0.10s sustained."""
    ramp = wheel.FastStopRamp()
    high_wz = math.radians(60.0)
    for k in range(8):  # 0.00s .. 0.14s (> 0.10s) with the SAME imu_stamp=0.00
        cmd = ramp.command(k * 0.02, 0.60, 0.50, imu_wz_rad_s=high_wz, imu_stamp=0.00)
        assert tuple(cmd) != wheel.STOP_COMMAND
        assert ramp.done is False


def test_reset_allows_clean_next_stop():
    ramp = wheel.FastStopRamp()
    assert tuple(ramp.command(0.0, 0.10, 0.10)) == wheel.STOP_COMMAND
    assert ramp.done is True
    ramp.reset()
    assert ramp.done is False
    assert ramp.stage == "align"
    c = ramp.command(5.0, 0.70, 0.70)
    assert tuple(c) != wheel.STOP_COMMAND

"""Unit tests for FF-PID NO_TRACKING recovery, OBSTACLE_STOP hold/auto-resume,
manual-intervention reporting during hold, and 0.20 s wheel settle guard."""
import importlib.util
import json
import sys
import threading
import types
from pathlib import Path

import pytest


PID_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(PID_SCRIPTS))
import ff_pid_recovery as rec


# --- 1. Pure helper tests ---

def test_choice_a_degraded_tracking_classification():
    # <= 1.5s passes straight through without stopping
    assert rec.classify_tracking("DEGRADED", 0.2, 5.0, 0.0) == ("PASS", None)
    assert rec.classify_tracking("DEGRADED", 0.2, 5.0, 1.5) == ("PASS", None)
    # 1.5s < age <= 5.0s enters TRACKING_HOLD (auto-resumable)
    assert rec.classify_tracking("DEGRADED", 0.2, 5.0, 1.6) == ("HOLD", "TRACKING_HOLD")
    assert rec.classify_tracking("DEGRADED", 0.2, 5.0, 5.0) == ("HOLD", "TRACKING_HOLD")
    # > 5.0s escalates to permanent NO_TRACKING
    assert rec.classify_tracking("DEGRADED", 0.2, 5.0, 5.1) == ("FATAL", "NO_TRACKING")
    # Start/resume strictly requires TRACKING
    assert rec.classify_tracking("DEGRADED", 0.2, 5.0, 0.1, for_start_or_resume=True) == ("FATAL", "NO_TRACKING")
    assert rec.classify_tracking("TRACKING", 0.2, 5.0, None, for_start_or_resume=True) == ("OK", None)
    # LOST / WAITING_INITIALIZATION / stale diag are always FATAL
    assert rec.classify_tracking("LOST", 0.1, 5.0, 0.0) == ("FATAL", "NO_TRACKING")
    assert rec.classify_tracking("WAITING_INITIALIZATION", 0.1, 5.0, 0.0) == ("FATAL", "NO_TRACKING")
    assert rec.classify_tracking("TRACKING", 5.5, 5.0, None) == ("FATAL", "NO_TRACKING")


def test_policy_stop_classification():
    for r in ("PATH_OBSTACLE", "VERY_CLOSE_OBJECT", "STATIC_REPLAN_EARLY",
              "RELEASE_CONFIRM", "REPLAN_HOLD"):
        assert rec.classify_policy_stop(r) == ("OBSTACLE", "OBSTACLE_STOP")
    for r in ("PERCEPTION_STALE", "PERCEPTION_NOT_OK",
              "PERCEPTION_SOURCE_STALE", "POSE_MISSING"):
        assert rec.classify_policy_stop(r) == ("TRANSIENT", "SENSOR_HOLD")
    for r in ("MASK_BOUNDARY", "BAD_OBJECT", "BAD_OBJECT_PROFILE", "SAFETY_EXCEPTION"):
        kind, label = rec.classify_policy_stop(r)
        assert kind == "FATAL"
        assert label == "SAFETY_STOP: " + r


def test_wheel_settle_guard_020s_and_ghost_timeout():
    guard = rec.WheelSettleGuard(threshold_mps=0.06, settle_s=0.20, timeout_s=2.50)

    # Residual 0.17 m/s on right wheel blocks immediate resume
    guard.observe_wheels(0.00, 0.17, 1.00)
    assert guard.can_resume(1.00) == (False, False)
    guard.observe_wheels(0.00, 0.14, 1.20)
    assert guard.can_resume(1.20) == (False, False)

    # Drops <= 0.06 m/s: must wait full 0.20 s continuously
    guard.observe_wheels(0.00, 0.00, 1.30)
    assert guard.can_resume(1.30) == (False, False)
    guard.observe_wheels(0.00, 0.03, 1.45)
    assert guard.can_resume(1.45) == (False, False)
    guard.observe_wheels(0.00, 0.00, 1.50)
    assert guard.can_resume(1.50) == (True, False)

    # Stuck single-wheel ghost (0.00 / 0.17 m/s forever) times out after 2.50 s
    guard2 = rec.WheelSettleGuard(threshold_mps=0.06, settle_s=0.20, timeout_s=2.50)
    for k in range(25):
        t = 10.0 + k * 0.10
        guard2.observe_wheels(0.00, 0.17, t)
        assert guard2.can_resume(t) == (False, False)
    guard2.observe_wheels(0.00, 0.17, 12.50)
    assert guard2.can_resume(12.50) == (True, True)

    # Both wheels rolling (0.20 / 0.20) or high single-wheel speed (0.00 / 0.50)
    # must NEVER bypass settle via the ghost timeout!
    for pair in ((0.20, 0.20), (0.00, 0.50)):
        guard3 = rec.WheelSettleGuard(threshold_mps=0.06, settle_s=0.20, timeout_s=2.50)
        for k in range(40):
            t = 20.0 + k * 0.10
            guard3.observe_wheels(pair[0], pair[1], t)
            assert guard3.can_resume(t) == (False, False)


# --- 2. FFNode state machine tests (with minimal fake clock & ROS stubs) ---

class _FakeClock:
    def __init__(self, t=100.0):
        self.t = t

    def now(self):
        return types.SimpleNamespace(to_sec=lambda: self.t)


def _make_fake_ff_node():
    clock = _FakeClock(100.0)
    # Stub ROS modules only if missing so we can import FFNode methods cleanly
    rospy_mod = types.ModuleType("rospy")
    rospy_mod.Time = clock
    rospy_mod.logwarn_throttle = lambda *a, **k: None
    rospy_mod.logerr_throttle = lambda *a, **k: None
    rospy_mod.logerr = lambda *a, **k: None
    rospy_mod.loginfo = lambda *a, **k: None
    rospy_mod.ROSInitException = RuntimeError
    rospy_mod.ROSInterruptException = RuntimeError
    sys.modules["rospy"] = rospy_mod
    for name in ("rosgraph", "rospkg", "tf", "tf.transformations",
                 "diagnostic_msgs", "diagnostic_msgs.msg",
                 "geometry_msgs", "geometry_msgs.msg",
                 "nav_msgs", "nav_msgs.msg",
                 "std_msgs", "std_msgs.msg",
                 "std_srvs", "std_srvs.srv"):
        sys.modules.setdefault(name, types.ModuleType(name))

    sys.modules["rospkg"].RosPack = lambda: types.SimpleNamespace(
        get_path=lambda pkg: str(Path(__file__).resolve().parents[2] / pkg)
    )
    sys.modules["diagnostic_msgs.msg"].DiagnosticArray = object
    sys.modules["geometry_msgs.msg"].PoseWithCovarianceStamped = object
    sys.modules["geometry_msgs.msg"].Twist = lambda: types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.0), angular=types.SimpleNamespace(z=0.0)
    )
    sys.modules["nav_msgs.msg"].Odometry = object
    sys.modules["std_msgs.msg"].Bool = object
    sys.modules["std_msgs.msg"].Int16MultiArray = object
    sys.modules["std_msgs.msg"].String = lambda data="": types.SimpleNamespace(data=data)
    sys.modules["std_srvs.srv"].SetBool = object
    sys.modules["std_srvs.srv"].SetBoolResponse = lambda ok, msg: (ok, msg)

    spec = importlib.util.spec_from_file_location("ff_pid_node_mod", PID_SCRIPTS / "ff_pid_node.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    node = object.__new__(mod.FFNode)
    node.lock = threading.RLock()
    node.route_name = "A_to_D5"
    node.route_version = "original"
    node.replan_resume_pending = False
    node.core = types.SimpleNamespace(max_v=0.8, wheel_track=0.46, reset=lambda: None, station=0.0)
    node.mask = types.SimpleNamespace(contains=lambda pt: True)
    node.enabled = True
    node.finishing = False
    node.external_hold = False
    node.stopped_reports = 0
    node.competing_sources = []
    node.stopped_wheel_threshold = 0.06
    node.max_pose_age = 0.50
    node.max_wheel_age = 0.35
    node.max_diag_age = 5.0
    node.degraded_pass_s = 1.5
    node.degraded_max_hold_s = 5.0
    node.transient_hold_max_s = 3.0
    node.wheel_settle = rec.WheelSettleGuard(threshold_mps=0.06, settle_s=0.20, timeout_s=2.50)
    node.motion_policy = "CLEAR"
    node.policy_reason = ""
    node.speed_limit_mps = 0.8
    node.policy_received = clock.t
    node.obstacle_stopped = False
    node.resume_after_obstacle = False
    node.auto_hold_kind = None
    node.auto_hold_since = None
    node.auto_hold_detail = None
    node.wheel = (0.0, 0.0)
    node.wheel_pair = (0.0, 0.0)
    node.wheel_received = clock.t
    node.mode = mod.AUTO_MODE
    node.tracking = "TRACKING"
    node.diag_received = clock.t
    node.degraded_since = None
    node.reason = "RUNNING"
    node.last_tick = None
    node.last_command_v = 0.0
    node.last_command_w = 0.0
    node.current_pose = lambda now: (0.0, 0.0, 0.0)
    node.statuses = []
    node.pub = types.SimpleNamespace(publish=lambda msg: None)
    node.status = types.SimpleNamespace(
        publish=lambda msg: node.statuses.append(json.loads(msg.data)["state"])
    )
    return mod, node, clock


def _policy_msg(state, cap, reason, stamp):
    return types.SimpleNamespace(data=json.dumps({
        "state": state, "speed_limit_mps": cap, "reason": reason,
        "route": "A_to_D5", "route_version": "original", "stamp": stamp,
    }))


def _wheel_msg(mode, l_mps, r_mps):
    def enc(v):
        if v == 0.0:
            return 83, 33
        return (67 if v > 0 else 87), int(round(abs(v) * 36.0)) + 33
    dl, ml = enc(l_mps)
    dr, mr = enc(r_mps)
    return types.SimpleNamespace(data=[72, mode, dl, ml, dr, mr, 79, 88, 0, 13, 10])


def _diag_msg(state):
    return types.SimpleNamespace(status=[
        types.SimpleNamespace(name="fast_lio_icp", message=state)
    ])


def test_manual_during_obstacle_stop_publishes_manual_intervention_and_never_resumes():
    mod, node, clock = _make_fake_ff_node()
    node.on_motion_policy(_policy_msg("STOP", 0.0, "PATH_OBSTACLE", clock.t))
    assert node.enabled is False
    assert node.statuses[-1] == "OBSTACLE_STOP"

    # Switch to MANUAL while waiting in OBSTACLE_STOP
    clock.t += 0.2
    node.on_wheel(_wheel_msg(77, 0.0, 0.0))
    assert node.statuses[-1] == "MANUAL_INTERVENTION"
    assert node.resume_after_obstacle is False

    # Even if mode returns to AUTO and obstacle clears, MUST NOT auto-resume!
    clock.t += 0.5
    node.on_wheel(_wheel_msg(65, 0.0, 0.0))
    node.on_motion_policy(_policy_msg("CLEAR", 0.8, "CLEAR", clock.t))
    assert node.enabled is False


def test_transient_perception_blip_during_obstacle_stop_does_not_kill_auto_resume():
    mod, node, clock = _make_fake_ff_node()
    node.on_wheel(_wheel_msg(65, 0.0, 0.0))
    node.on_motion_policy(_policy_msg("STOP", 0.0, "PATH_OBSTACLE", clock.t))
    assert node.statuses[-1] == "OBSTACLE_STOP"

    # Brief PERCEPTION_SOURCE_STALE blip during OBSTACLE_STOP
    clock.t += 0.3
    node.on_wheel(_wheel_msg(65, 0.0, 0.0))
    node.on_motion_policy(_policy_msg("STOP", 0.0, "PERCEPTION_SOURCE_STALE", clock.t))
    assert node.resume_after_obstacle is True

    # Clears back to CLEAR -> wheels already settled >= 0.20s -> resumes RUNNING!
    clock.t += 0.1
    node.on_wheel(_wheel_msg(65, 0.0, 0.0))
    node.on_motion_policy(_policy_msg("CLEAR", 0.8, "CLEAR", clock.t))
    assert node.enabled is True
    assert node.statuses[-1] == "RUNNING"


def test_mask_boundary_permanently_stops_even_during_obstacle_hold():
    mod, node, clock = _make_fake_ff_node()
    node.on_motion_policy(_policy_msg("STOP", 0.0, "PATH_OBSTACLE", clock.t))
    clock.t += 0.2
    node.on_motion_policy(_policy_msg("STOP", 0.0, "MASK_BOUNDARY", clock.t))
    assert node.statuses[-1] == "SAFETY_STOP: MASK_BOUNDARY"
    assert node.resume_after_obstacle is False

    clock.t += 0.5
    node.on_wheel(_wheel_msg(65, 0.0, 0.0))
    node.on_motion_policy(_policy_msg("CLEAR", 0.8, "CLEAR", clock.t))
    assert node.enabled is False


def test_degraded_choice_a_pass_hold_and_auto_resume():
    mod, node, clock = _make_fake_ff_node()
    node.on_wheel(_wheel_msg(65, 0.0, 0.0))

    # DEGRADED starts at t=100.0 -> at t=101.0 (1.0s <= 1.5s) readiness is STILL None (PASS!)
    node.on_diag(_diag_msg("DEGRADED"))
    clock.t += 1.0
    node.on_wheel(_wheel_msg(65, 0.0, 0.0))
    node.policy_received = clock.t
    assert node.readiness(clock.t, for_start_or_resume=False) is None

    # At t=102.0 (2.0s > 1.5s, <= 5.0s) -> enters TRACKING_HOLD
    clock.t += 1.0
    node.on_wheel(_wheel_msg(65, 0.0, 0.0))
    node.policy_received = clock.t
    assert node.readiness(clock.t, for_start_or_resume=False) == "TRACKING_HOLD"
    node.enter_hold("TRACKING", "TRACKING_HOLD", clock.t, detail="NO_TRACKING")
    assert node.enabled is False
    assert node.statuses[-1] == "TRACKING_HOLD"

    # At t=103.0 TRACKING returns (with continuous wheel status during hold) -> auto-resumes RUNNING!
    for step in (0.25, 0.50, 0.75, 1.00):
        clock.t = 102.0 + step
        node.on_wheel(_wheel_msg(65, 0.0, 0.0))
    node.policy_received = clock.t
    node.on_diag(_diag_msg("TRACKING"))
    assert node.try_auto_resume(clock.t) is True
    assert node.enabled is True
    assert node.statuses[-1] == "RUNNING"


def test_residual_wheel_speed_delays_auto_resume_by_020s_after_settling():
    mod, node, clock = _make_fake_ff_node()
    node.on_wheel(_wheel_msg(65, 0.0, 0.17))
    node.on_motion_policy(_policy_msg("STOP", 0.0, "PATH_OBSTACLE", clock.t))
    assert node.enabled is False

    # Obstacle clears immediately while R wheel still has 0.17 m/s residual
    clock.t += 0.1
    node.on_wheel(_wheel_msg(65, 0.0, 0.17))
    node.on_motion_policy(_policy_msg("CLEAR", 0.8, "CLEAR", clock.t))
    assert node.enabled is False  # Blocked by wheel settle guard!

    # Wheel settles to 0.00 at t=100.2 -> still needs 0.20 s
    clock.t += 0.1
    node.on_wheel(_wheel_msg(65, 0.0, 0.0))
    assert node.try_auto_resume(clock.t) is False

    # At t=100.35 (0.15 s settled) -> still waiting
    clock.t += 0.15
    node.on_wheel(_wheel_msg(65, 0.0, 0.0))
    assert node.try_auto_resume(clock.t) is False

    # At t=100.40 (0.20 s settled) -> resumes RUNNING!
    clock.t += 0.05
    node.on_wheel(_wheel_msg(65, 0.0, 0.0))
    assert node.try_auto_resume(clock.t) is True
    assert node.enabled is True
    assert node.statuses[-1] == "RUNNING"

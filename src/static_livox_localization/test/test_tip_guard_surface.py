from pathlib import Path


ROOT = Path(__file__).parents[1]


def guard_text():
    return (ROOT / "scripts" / "tip_guard.py").read_text(encoding="utf-8")


def test_guard_sits_between_gated_command_and_final_cmd_vel():
    text = guard_text()
    assert '"/cmd_vel_gated"' in text
    assert '"/cmd_vel"' in text


def test_guard_uses_the_shared_rate_limiter_and_stops_on_shutdown():
    text = guard_text()
    assert "next_linear_speed(" in text
    assert "stale)" in text
    assert "rospy.on_shutdown(lambda: self.pub.publish(Twist()))" in text


def test_stale_input_forces_a_stop():
    text = guard_text()
    assert "stale = (now - self.raw_stamp).to_sec() > INPUT_STALE_S" in text


def test_tip_detection_was_deliberately_removed_not_forgotten():
    """Guards against silently reintroducing pitch-rate/trip logic without
    a matching, deliberate design decision - the removal reason must stay
    documented at the top of the file."""
    text = guard_text()
    assert "Tip-over detection/prevention" in text
    assert "was removed" in text
    assert "should_trip" not in text
    assert "self.tripped" not in text


def test_the_inner_wheel_clamp_is_applied_to_what_is_published():
    text = guard_text()
    assert "no_reverse_yaw(out.linear.x, out.angular.z" in text
    assert '"~no_reverse", True' in text


def test_the_ramp_resumes_from_the_wheel_report():
    text = guard_text()
    assert "resume_speed(self.current_speed" in text
    assert '"/wheel_status"' in text
    assert '"~resume_from_wheels", True' in text

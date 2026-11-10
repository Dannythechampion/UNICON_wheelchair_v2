"""Launch from rest straight, and at the speed the wheels turn for.

ROS-free so the policy can be tested at a desk; dwa_follower owns the clock
and the wheel report and asks this module what to put on the wire.

WHY
---
The loaded wheels do not turn under roughly 0.30 m/s, and on a climb they do
not come free together. On 2026-11-09 the A->J climb was walked by a stream
of pedestrians; every gate stop for them ended in a launch from rest, and
the launch is where the steering went wrong:

  17:05:07  w +0.25..+0.35 commanded for 3 s while both wheels read 0.00,
            then both came free together - recovered, heading -35 -> +11 deg.
  18:52:33  w +0.15..+0.20 commanded while the ramp crawled 0.02 -> 0.34;
            the right wheel came free first (left 0.00 / right 0.11 m/s),
            the chair turned harder than asked, heading -23 -> +40 deg, DWA
            saturated at -0.45 for 2 s and overshot to -50 deg.

Two parts, both only from rest:

  D  the launch command goes straight to LAUNCH_SPEED_MPS (the turn floor)
     instead of ramping through the deadband. Above it the normal ramp runs.
  A  until each wheel reports RELEASE_MPS, the yaw rate is held at zero, so
     a wheel that comes free first has no differential to act on. Steering
     is handed back as soon as both wheels turn, or after TIMEOUT_S, so a
     wheel that never comes free cannot hold the chair straight forever.

A is not used while going round something (GO_ROUND / PERSON_BYPASS): there
the first metre is the manoeuvre, and holding it straight drives at the
object. D still applies there.
"""

LAUNCH_SPEED_MPS = 0.35
# Each wheel, not the mean. 9 counts of the 0.1 km/h report; under the 0.30
# deadband edge because the report lags the wheel by 0.16-0.28 s.
RELEASE_MPS = 0.25
TIMEOUT_S = 2.0
# One count of the report away from zero is rest (dwa_follower uses the same).
AT_REST_MPS = 0.05
# The command the ramp holds when it is effectively stopped.
COMMAND_AT_REST_MPS = 0.02


def launch_speed(current_command, target, floor=LAUNCH_SPEED_MPS):
    """D: the speed to command when ramping toward target from current.

    Returns floor when the command is below it and the target is at or above
    it, else None (the normal ramp decides). Never used to slow down.
    """
    if floor <= 0.0:
        return None
    if 0.0 <= float(current_command) < floor <= float(target):
        return float(floor)
    return None


class StraightLaunch:
    """A: hold the yaw rate at zero from rest until both wheels turn."""

    def __init__(self, release_mps=RELEASE_MPS, timeout_s=TIMEOUT_S,
                 enabled=True):
        self.release_mps = float(release_mps)
        self.timeout_s = float(timeout_s)
        self.enabled = bool(enabled)
        self.active = False
        self.since_s = None
        self.last_exit = ""

    def reset(self, reason="reset"):
        if self.active:
            self.last_exit = reason
        self.active = False
        self.since_s = None

    def eligible(self, current_command, left_mps, right_mps, wheels_fresh,
                 allowed):
        """Whether a launch would start (or continue) this cycle.

        Asked before the planner runs, so candidate safety judges the
        command that will actually be sent.
        """
        if not self.enabled or not allowed or not wheels_fresh:
            return False
        if self.active:
            return True
        return (float(current_command) <= COMMAND_AT_REST_MPS
                and abs(float(left_mps)) <= AT_REST_MPS
                and abs(float(right_mps)) <= AT_REST_MPS)

    def update(self, now_s, current_command, target_v, left_mps, right_mps,
               wheels_fresh, allowed, floor=LAUNCH_SPEED_MPS):
        """Advance the state after the planner chose target_v.

        Returns True while the yaw rate must be held at zero.
        """
        now_s = float(now_s)
        if not self.eligible(current_command, left_mps, right_mps,
                             wheels_fresh, allowed):
            self.reset("not eligible" if self.active else self.last_exit)
            return False
        if float(target_v) < floor:
            # Planner wants to stop (or cannot move): no launch in progress.
            self.reset("stopped")
            return False
        if not self.active:
            self.active = True
            self.since_s = now_s
            self.last_exit = ""
        if float(left_mps) >= self.release_mps and \
                float(right_mps) >= self.release_mps:
            self.reset("wheels turning")
            return False
        if now_s - self.since_s >= self.timeout_s:
            self.reset("timeout")
            return False
        return True

"""ROS-free final-stage linear-speed policy for the field tip guard."""

import math

ABSOLUTE_V_LIMIT = 1.6
NORMAL_DECEL_MPS2 = 2.0


def resume_speed(current, requested, rolling):
    """Where the ramp restarts after a zero, when the wheels still turn.

    A zero request resets this guard to 0 (next_linear_speed), and the next
    non-zero request used to ramp up from there at ACCEL - while the chair
    was still rolling at 0.6-0.7 m/s. At the 11-09 crest that put 0.01-0.16
    m/s on the wire for half a second (18:45:51.6, 18:53:16.4-17.1): a
    near-stop that is not the 0/0 the base's own stop ramp runs on, so each
    wheel braked hard on its own and the chair spun. The DWA follower
    already resumes from the wheels (dwa_follower.rolling_wheel_speed); this
    is the same at the last stage. Never above what was requested, never
    used to speed up a chair that is slower than the command.

    rolling is the measured forward speed, or None when stale / at rest.
    """
    if rolling is None or requested <= 0.0 or current >= rolling:
        return current
    return max(current, min(float(rolling), float(requested)))


def next_linear_speed(current, requested, accel_limit, dt, stop_required,
                      launch_floor=0.0):
    """Apply the governor without delaying an upstream stop.

    A zero request is the common stop authority used by obstacle, localization,
    route-band, tilt, and manual-mode holds. The final stage must never turn
    that zero into residual motion through its own slew limiter.

    launch_floor is the speed the loaded wheels start turning at. Ramping up
    to it at accel_limit publishes about a second of 0.0x-0.3 m/s that the
    wheels ignore - or, on the 11-09 climb, that one wheel answers and the
    other does not (18:52:35, left 0.00 / right 0.11 m/s). So a request at or
    above the floor, from below it, goes straight to the floor; the governor
    applies from there. A request under the floor is ramped as before, and
    braking is untouched. 0.0 (the default) is the old behaviour exactly.
    """
    if stop_required or requested == 0.0:
        return 0.0

    desired = max(-ABSOLUTE_V_LIMIT, min(ABSOLUTE_V_LIMIT, requested))
    floor = max(0.0, float(launch_floor))
    if floor > 0.0 and 0.0 <= current < floor <= desired:
        return floor
    if desired > current:
        step = min(desired - current, max(0.0, accel_limit) * max(0.0, dt))
    else:
        step = max(desired - current, -NORMAL_DECEL_MPS2 * max(0.0, dt))
    speed = current + step
    return max(-ABSOLUTE_V_LIMIT, min(ABSOLUTE_V_LIMIT, speed))


def no_reverse_yaw(linear_speed, yaw_rate, wheel_separation_m):
    """The yaw rate, limited so the inner wheel never runs backwards.

    The base encodes each wheel as (v -/+ w * separation / 2); a negative
    one goes out as a reverse byte (87), and on this base a reverse byte at
    a few counts is a hard brake on that wheel alone. That is what spun the
    chair at the A->J crest on 2026-11-09 (18:45:51, 18:53:16): terrain_guard
    zeroed the command mid left turn, this guard then ramped v up from zero
    at ACCEL while passing w through untouched - 0.01 m/s with +0.05 rad/s,
    0.03 with +0.10 - so the left wheel was braked while the right one kept
    rolling at 0.6 m/s, and the chair turned 50-60 deg. The base's own
    stop ramp, which slows both wheels together, only runs on an exact 0/0;
    the 14:59:37 stop from 0.8 m/s going straight spun nothing.

    So |w| <= 2 v / separation: at worst the inner wheel stands still, never
    reverses, and a near-zero v carries a near-zero w. Nothing is added to
    v (unlike the base's turn boost), so a stop stays a stop.
    """
    v = float(linear_speed)
    w = float(yaw_rate)
    if not (math.isfinite(v) and math.isfinite(w)) or v <= 0.0:
        return 0.0
    # A hair inside the bound, so float rounding cannot make the inner wheel
    # -1e-17 and encode it as a reverse byte.
    cap = 2.0 * v / float(wheel_separation_m) * (1.0 - 1e-6)
    return max(-cap, min(cap, w))

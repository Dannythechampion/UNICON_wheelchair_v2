"""ROS-free final-stage linear-speed policy for the field tip guard."""

ABSOLUTE_V_LIMIT = 1.6
NORMAL_DECEL_MPS2 = 2.0


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

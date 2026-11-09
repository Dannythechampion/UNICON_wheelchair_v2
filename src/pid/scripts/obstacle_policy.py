"""ROS-independent obstacle-specific forward-path policy.

A static obstacle stops at the beginning of the existing 2 m slow corridor.
A *confirmed receding* moving object is not a path-speed restriction in the
1-2 m band, and causes a SLOW (not CLEAR) inside 1 m. A collision-near object
is never exempt from the independent footprint emergency stop.
"""
FRONT_STOP_M = 1.0
FRONT_SLOW_M = 2.0


def obstacle_restriction(distance, motion, confirmed_receding, max_speed,
                         slow_min_speed, static_detour_enabled=False,
                         receding_enabled=False):
    """Return (speed_ceiling, must_stop, reason) for one path-relative object."""
    if distance is None or distance > FRONT_SLOW_M:
        return max_speed, False, "NONE"
    if motion == "static" and static_detour_enabled:
        return 0.0, True, "STATIC_PATH_OBSTACLE"
    if motion == "moving" and confirmed_receding and receding_enabled:
        if distance > FRONT_STOP_M:
            return max_speed, False, "RECEDING_IGNORED"
        return min(slow_min_speed, max_speed), False, "RECEDING_NEAR_SLOW"
    if distance <= FRONT_STOP_M:
        return 0.0, True, "PATH_OBSTACLE"
    fraction = (distance - FRONT_STOP_M) / (FRONT_SLOW_M - FRONT_STOP_M)
    cap = slow_min_speed + fraction * (max_speed - slow_min_speed)
    return cap, False, "PATH_OBSTACLE"

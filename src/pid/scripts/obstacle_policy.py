"""ROS-independent obstacle-specific forward-path and near-footprint policy.

A static obstacle stops at the beginning of the 3.0 m slow corridor when
static detour replanning is enabled.
A *confirmed receding* moving object is not a path-speed restriction in the
1.5-3.0 m band, and causes a SLOW (not CLEAR) inside 1.5 m. A collision-near
object (< 0.50 m) is never exempt from the independent footprint emergency stop,
and front/side objects in the 0.50-1.00 m band apply a linear speed ceiling.
"""
FRONT_STOP_M = 1.5
FRONT_SLOW_M = 3.0
NEAR_STOP_GAP_M = 0.50
NEAR_SLOW_GAP_M = 1.00


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


def near_object_speed_cap(near_gap, max_speed, slow_min_speed,
                          stop_gap_m=NEAR_STOP_GAP_M,
                          slow_gap_m=NEAR_SLOW_GAP_M):
    """Linear pre-slowdown cap for front/side objects approaching VERY_CLOSE_OBJECT."""
    if near_gap is None or near_gap >= slow_gap_m:
        return max_speed
    if near_gap < stop_gap_m:
        return 0.0
    fraction = (near_gap - stop_gap_m) / (slow_gap_m - stop_gap_m)
    return slow_min_speed + fraction * (max_speed - slow_min_speed)

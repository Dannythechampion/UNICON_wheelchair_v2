"""Pure-Python tracking, hold/auto-resume, and wheel-settle helpers for FFNode.

Kept free of ROS imports so unit tests and offline bag replay can exercise
the exact production state-transition logic without a running ROS master.
"""

import math


DEGRADED_PASS_S = 1.5          # Choice A: brief 1-frame DEGRADED passes through
DEGRADED_MAX_HOLD_S = 5.0      # Longer DEGRADED escalates to permanent NO_TRACKING
TRANSIENT_HOLD_MAX_S = 3.0     # Max hold for transient perception/pose/wheel hiccups
RESUME_WHEEL_SETTLE_S = 0.20   # Both wheels <= 0.06 m/s continuously before auto-resume
RESUME_WHEEL_TIMEOUT_S = 2.50  # Avoid infinite lockup on a stuck stationary ghost speed
WHEEL_SETTLE_FRESH_S = 0.35
ROLLING_RESUME_MIN_MPS = 0.15       # Both wheels clearly rolling forward -> immediate resume
ROLLING_RESUME_MAX_DIFF_MPS = 0.10  # Max |L-R| allowed for rolling resume

OBSTACLE_STOP_REASONS = frozenset((
    "PATH_OBSTACLE",
    "VERY_CLOSE_OBJECT",
    "STATIC_REPLAN_EARLY",
    "RELEASE_CONFIRM",
    "REPLAN_HOLD",
))

TRANSIENT_POLICY_STOP_REASONS = frozenset((
    "PERCEPTION_STALE",
    "PERCEPTION_NOT_OK",
    "PERCEPTION_SOURCE_STALE",
    "POSE_MISSING",
))

TRANSIENT_READINESS_REASONS = frozenset((
    "POSE_STALE",
    "WHEEL_STALE",
    "SAFETY_POLICY_STALE",
))


def classify_tracking(tracking, diag_age, max_diag_age, degraded_age,
                      pass_s=DEGRADED_PASS_S,
                      max_hold_s=DEGRADED_MAX_HOLD_S,
                      for_start_or_resume=False):
    """Classify GICP tracking state into (action, status_label).

    Actions:
      - "OK"    : healthy TRACKING
      - "PASS"  : brief DEGRADED (<= pass_s) while already running; keep driving
      - "HOLD"  : medium DEGRADED (pass_s < age <= max_hold_s); pause & auto-resume
      - "FATAL" : LOST / WAITING_INITIALIZATION / stale diag / DEGRADED > max_hold_s
    """
    if diag_age is None or not math.isfinite(diag_age) or diag_age > max_diag_age:
        return "FATAL", "NO_TRACKING"
    if tracking == "TRACKING":
        return "OK", None
    if for_start_or_resume:
        return "FATAL", "NO_TRACKING"
    if tracking == "DEGRADED":
        if degraded_age is None or not math.isfinite(degraded_age) or degraded_age < 0.0:
            degraded_age = 0.0
        if degraded_age <= pass_s:
            return "PASS", None
        if degraded_age <= max_hold_s:
            return "HOLD", "TRACKING_HOLD"
        return "FATAL", "NO_TRACKING"
    return "FATAL", "NO_TRACKING"


def classify_policy_stop(reason):
    """Classify a /ff_pid/motion_policy STOP reason.

    Returns (kind, status_label) where kind is "OBSTACLE", "TRANSIENT", or "FATAL".
    """
    text = str(reason or "UNKNOWN")
    if text in OBSTACLE_STOP_REASONS:
        return "OBSTACLE", "OBSTACLE_STOP"
    if text in TRANSIENT_POLICY_STOP_REASONS:
        return "TRANSIENT", "SENSOR_HOLD"
    return "FATAL", "SAFETY_STOP: " + text


def classify_readiness_fault(reason):
    """Classify a non-tracking readiness fault into ("TRANSIENT"|"FATAL", status)."""
    text = str(reason or "UNKNOWN")
    if text in TRANSIENT_READINESS_REASONS:
        return "TRANSIENT", "SENSOR_HOLD"
    return "FATAL", "SAFETY_STOP: " + text


class WheelSettleGuard:
    """Requires both measured wheels <= threshold_mps for settle_s before
    auto-resuming from a stop, with a bounded timeout_s cap ONLY for a
    single-wheel stationary ghost speed (min <= threshold, max <= max_ghost_mps).
    """

    def __init__(self, threshold_mps=0.06, settle_s=RESUME_WHEEL_SETTLE_S,
                 timeout_s=RESUME_WHEEL_TIMEOUT_S,
                 fresh_s=WHEEL_SETTLE_FRESH_S,
                 max_ghost_mps=0.35,
                 rolling_min_mps=ROLLING_RESUME_MIN_MPS,
                 rolling_max_diff_mps=ROLLING_RESUME_MAX_DIFF_MPS):
        self.threshold_mps = float(threshold_mps)
        self.settle_s = float(settle_s)
        self.timeout_s = float(timeout_s)
        self.fresh_s = float(fresh_s)
        self.max_ghost_mps = float(max_ghost_mps)
        self.rolling_min_mps = float(rolling_min_mps)
        self.rolling_max_diff_mps = float(rolling_max_diff_mps)
        self.settled_since = None
        self.last_wheel_t = None
        self.last_pair = None
        self.last_signed_pair = None
        self.wait_since = None

    def reset_all(self):
        self.settled_since = None
        self.last_wheel_t = None
        self.last_pair = None
        self.last_signed_pair = None
        self.wait_since = None

    def reset_wait(self):
        self.wait_since = None

    def observe_wheels(self, left_mps, right_mps, now_s):
        try:
            left = float(left_mps)
            right = float(right_mps)
            now = float(now_s)
        except (TypeError, ValueError):
            return
        if not (math.isfinite(left) and math.isfinite(right) and math.isfinite(now)):
            return
        if self.last_wheel_t is not None and now - self.last_wheel_t > self.fresh_s:
            self.settled_since = None
        self.last_wheel_t = now
        self.last_pair = (abs(left), abs(right))
        self.last_signed_pair = (left, right)
        if abs(left) <= self.threshold_mps and abs(right) <= self.threshold_mps:
            if self.settled_since is None:
                self.settled_since = now
        else:
            self.settled_since = None

    def is_clean_forward_rolling(self):
        """True when both wheels are clearly rolling forward together (no S cutoff,
        no single-wheel residual or reversal)."""
        if self.last_signed_pair is None:
            return False
        left, right = self.last_signed_pair
        return (left > self.rolling_min_mps and
                right > self.rolling_min_mps and
                abs(left - right) <= self.rolling_max_diff_mps + 1e-9)

    def _is_single_wheel_ghost(self):
        if self.last_pair is None:
            return False
        lo, hi = min(self.last_pair), max(self.last_pair)
        return lo <= self.threshold_mps and hi <= self.max_ghost_mps

    def can_resume(self, now_s):
        """Return (can_resume_now, timed_out_on_residual)."""
        now = float(now_s)
        fresh = (self.last_wheel_t is not None and
                 0.0 <= now - self.last_wheel_t <= self.fresh_s)
        if (fresh and self.settled_since is not None and
                now - self.settled_since + 1e-9 >= self.settle_s):
            self.wait_since = None
            return True, False
        if fresh and self.is_clean_forward_rolling():
            self.wait_since = None
            return True, False
        if not fresh or not self._is_single_wheel_ghost():
            # Never bypass wheel settle when unbalanced, reversing, or speed > max_ghost_mps.
            self.wait_since = None
            return False, False
        if self.wait_since is None:
            self.wait_since = now
        if now - self.wait_since + 1e-9 >= self.timeout_s:
            self.wait_since = None
            return True, True
        return False, False

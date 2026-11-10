"""ROS-independent temporal static-vote counter for PID A* entry.

Only distinct 5 Hz geometric observation timestamps count. "unknown" and
"moving" do not count as static. Confirmation triggers STOP/inspection, NOT
permission to drive around a moving obstacle.
"""
from collections import deque
import math

STATIC_WATCH_M = 5.0
STATIC_WINDOW_S = 2.0
STATIC_MIN_SAMPLES = 8
STATIC_FRACTION = 0.50
STATIC_MAX_LAST_SEEN_S = 0.45


class StaticVoteWindow:
    def __init__(self, window_s=STATIC_WINDOW_S, min_samples=STATIC_MIN_SAMPLES,
                 fraction=STATIC_FRACTION):
        self.window_s = float(window_s)
        self.min_samples = int(min_samples)
        self.fraction = float(fraction)
        self.history = {}
        self.last_stamp = None
        self.latched_id = None
        self.latch_last_seen = None

    def reset(self):
        self.history.clear()
        self.last_stamp = None
        self.latched_id = None
        self.latch_last_seen = None

    def add(self, stamp, observations):
        """observations: [(object_id, motion, route_gap_m), ...].

        Called at 10 Hz safely, but adds votes only for a NEW measurement stamp.
        Objects not on the forward 5 m route do not contribute any votes.
        """
        stamp = float(stamp)
        if not math.isfinite(stamp) or stamp <= 0:
            return False
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        self.last_stamp = stamp
        visible_ids = set()
        for object_id, motion, gap in observations:
            if gap is None or not math.isfinite(float(gap)) or not 0 <= gap <= STATIC_WATCH_M:
                continue
            key = int(object_id)
            visible_ids.add(key)
            if key not in self.history:
                self.history[key] = deque()
            self.history[key].append((stamp, str(motion)))
        if self.latched_id is not None:
            if self.latched_id in visible_ids:
                self.latch_last_seen = stamp
            elif self.latch_last_seen is None or stamp - self.latch_last_seen > 0.6:
                self.latched_id = None
                self.latch_last_seen = None
        for key, samples in list(self.history.items()):
            while samples and stamp - samples[0][0] > self.window_s:
                samples.popleft()
            if not samples:
                del self.history[key]
        return True

    def confirmed(self):
        if self.latched_id is not None:
            return self.latched_id
        if self.last_stamp is None:
            return None
        matches = []
        for object_id, samples in self.history.items():
            if (len(samples) < self.min_samples or
                    self.last_stamp - samples[-1][0] > STATIC_MAX_LAST_SEEN_S):
                continue
            static_count = sum(1 for _, motion in samples if motion == "static")
            fraction = static_count / len(samples)
            if fraction >= self.fraction:
                matches.append((fraction, len(samples), object_id))
        if not matches:
            return None
        self.latched_id = max(matches)[2]
        self.latch_last_seen = self.last_stamp
        return self.latched_id

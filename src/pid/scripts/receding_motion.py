"""Conservative relative-motion evidence for moving obstacles.

Only a sustained increase in chair-relative range AND forward separation
qualifies an observed 'moving' track for relaxed *path* speed control.
Unknown/static tracks, stale samples, id changes, bad timestamps, and mixed
motion immediately lose the exemption. This is not a collision prediction.
"""
import math
from collections import deque

MIN_HISTORY_S = 0.60
MIN_RANGE_INCREASE_M = 0.18
MIN_FORWARD_INCREASE_M = 0.12
MIN_SEPARATION_RATE_MPS = 0.20
MAX_SAMPLE_GAP_S = 0.35
MAX_OBSERVATION_AGE_S = 0.35
MAX_RELATIVE_RATE_MPS = 2.50
RANGE_BACKTRACK_TOLERANCE_M = 0.07
HISTORY_WINDOW_S = 1.5


class RecedingTracks:
    def __init__(self):
        self.tracks = {}

    def update(self, summary):
        if (not isinstance(summary, dict) or summary.get("status") != "OK"
                or summary.get("frame") != "chair_centre"):
            self.tracks.clear()
            return
        stamp = summary.get("stamp")
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp <= 0:
            self.tracks.clear()
            return
        objects = summary.get("objects")
        if not isinstance(objects, list):
            self.tracks.clear()
            return
        seen = set()
        for item in objects:
            if not isinstance(item, dict):
                continue
            object_id = item.get("id")
            if not isinstance(object_id, int) or isinstance(object_id, bool):
                continue
            seen.add(object_id)
            if item.get("motion") != "moving":
                self.tracks.pop(object_id, None)
                continue
            try:
                x, y = float(item["x"]), float(item["y"])
            except (TypeError, ValueError, KeyError):
                self.tracks.pop(object_id, None)
                continue
            if not math.isfinite(x) or not math.isfinite(y) or x <= 0:
                self.tracks.pop(object_id, None)
                continue
            history = self.tracks.setdefault(object_id, deque())
            if history and stamp <= history[-1][0]:
                # Fusion may republish the same geometric sample at 10Hz.
                if stamp < history[-1][0]:
                    self.tracks.pop(object_id, None)
                continue
            if history:
                age = stamp - history[-1][0]
                delta = math.hypot(x - history[-1][1], y - history[-1][2])
                previous_range = math.hypot(history[-1][1], history[-1][2])
                new_range = math.hypot(x, y)
                if (age > MAX_SAMPLE_GAP_S or
                        delta / age > MAX_RELATIVE_RATE_MPS or
                        new_range + RANGE_BACKTRACK_TOLERANCE_M < previous_range):
                    history.clear()
            history.append((stamp, x, y))
            while history and stamp - history[0][0] > HISTORY_WINDOW_S:
                history.popleft()
        for object_id in list(self.tracks):
            history = self.tracks[object_id]
            if object_id not in seen or (history and stamp - history[-1][0] > MAX_SAMPLE_GAP_S):
                self.tracks.pop(object_id, None)

    def receding(self, object_id, source_stamp, now=None):
        if not isinstance(object_id, int) or isinstance(object_id, bool):
            return False
        history = self.tracks.get(object_id)
        if not history or len(history) < 4:
            return False
        if (not isinstance(source_stamp, (int, float)) or
                not math.isfinite(source_stamp) or
                source_stamp - history[-1][0] > MAX_OBSERVATION_AGE_S or
                source_stamp < history[-1][0]):
            return False
        if now is not None:
            if (not isinstance(now, (int, float)) or not math.isfinite(now)
                    or now < source_stamp or now - source_stamp > MAX_OBSERVATION_AGE_S):
                return False
        start, end = history[0], history[-1]
        elapsed = end[0] - start[0]
        if elapsed < MIN_HISTORY_S:
            return False
        delta_range = math.hypot(end[1], end[2]) - math.hypot(start[1], start[2])
        delta_forward = end[1] - start[1]
        # Net departure alone is insufficient: a reversing object could
        # retain a large positive net displacement while now approaching.
        recent = list(history)[-3:]
        ranges = [math.hypot(h[1], h[2]) for h in recent]
        if any(new < old - 1e-3 for old, new in zip(ranges[:-1], ranges[1:])):
            return False
        # An object can recede in range while moving sideways into our lane.
        # Refuse the exemption if lateral separation is shrinking materially.
        lateral = [abs(h[2]) for h in history]
        if min(lateral) < lateral[0] - 0.08 or lateral[-1] < lateral[0] - 0.05:
            return False
        return (delta_range >= MIN_RANGE_INCREASE_M and
                delta_forward >= MIN_FORWARD_INCREASE_M and
                delta_range / elapsed >= MIN_SEPARATION_RATE_MPS)

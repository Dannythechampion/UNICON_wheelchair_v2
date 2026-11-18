"""Obstacle avoidance from classified, tracked clusters instead of raw returns.

obstacle_clusters.py was written as an observer - it draws boxes for RViz and
the black box, and nothing in the motion chain read it. This makes it a
control input, which changes what its output has to be trusted for.

Why clusters rather than the raw corridor check the follower already has:
that check stops on any five returns in the corridor above 0.18 m, so it
fires on sensor noise, on ground returns near the height clip, and on
whatever the rider's coat is doing. The cluster node requires eight points
across at least two occupied 0.20 m cells, excludes the rider by an explicit
box, and classifies what is left by footprint and height. Same sensor, far
fewer reasons to stop for nothing.

More importantly it carries a motion verdict, which is what lets the two
situations be answered differently. Something confirmed parked is gone
around; anything moving - or not yet watched long enough to say - is waited
out where it stands. A raw return cannot support that distinction at all,
because it has no identity from one scan to the next.

The geometry uses each object's EXTENT, not its centre. A vehicle whose
centre sits 3 m to the side can still have a corner in the corridor, and a
guard comparing centres would drive into it.

Extent means the object's own returns, sliced laterally, wherever the
producer publishes them. An axis-aligned box was the first answer and is
still the fallback, but its near face is a corner of the bounding volume
rather than anything the sensor saw.

For a wall crossing the scan diagonally the two answers are far apart, and
on 2026-07-31 the gap cost a 16-minute hold at waypoint 349. The box put
the wall 0.69 m dead ahead; its nearest return inside the corridor was
2.13 m. 0.69 m is inside the 0.9 m floor on the stop radius, so the chair
stopped - and a stopped chair's stopping envelope stays at that floor, so
the phantom near face stayed inside it and no amount of waiting could
change anything. The tracker had it right the whole time: static, and
therefore never going to move out of the way.

Sidestepping was not the missing answer and is not the fix. That wall
really does span the corridor, and the route curves past it; what the chair
needed was to not be hard-stopped by a face with no returns behind it.
Measured from the returns it reads 1.72 m - outside the stop radius, inside
the slow radius - so the chair creeps and the route does the rest. See
test/test_object_profile.py, which reconstructs the wall from the recorded
box and pins both numbers.

Everything unreadable fails closed, and fails closed as MOVING. A malformed
object is reported at zero distance rather than skipped, because skipping it
means not seeing an obstacle; and it is never reported as parked, because
the one thing that must not follow from a producer bug is a manoeuvre around
something whose position is not actually known.
"""

import json
import math

import numpy as np

from cluster_tracking import MOVING, STATIC, UNKNOWN

# Must match obstacle_clusters.WINDOW_S; test_cluster_guard pins the pair.
# The consumer needs it to size the stopping envelope: an object's reported
# position is already this old before the message is even published.
ACCUMULATION_S = 0.6
# The producer runs at 5 Hz and publishes every cycle, including when it has
# no cloud, so silence for this long means the node itself is gone.
STALE_S = 1.5

BLOCKED = 0.0

# A profile that is present but unreadable. Distinct from None, which is an
# object that never carried one, because the two have opposite answers: no
# profile falls back to the box, a broken profile blocks.
BROKEN = object()

# Spacing at which a box's near face is sampled when the producer published
# no profile. obstacle_clusters.PROFILE_BIN_M, so the fallback has the same
# resolution as the measurement it stands in for - test_object_profile pins
# the pair, and a fallback finer than the real thing would only be inventing
# detail the box never had.
BOX_SAMPLE_M = 0.2

# Objects handed to a planner as geometry, nearest first. A corridor this
# width does not hold many, and the rollout scoring is O(candidates x steps
# x points) inside a 0.1 s control period - the producer's forty-cluster
# ceiling arriving whole would be a different node.
MAX_OBSTACLE_OBJECTS = 4
# The producer's label for a person. Compared rather than enumerated,
# because a label this code does not recognise must not silently become
# something it is willing to drive around.
PERSON_LABEL = "person"
# The lidar sees a torso slice rather than a complete person. On 2026-08-25
# the earlier bypass measured its berth from those narrow boxes, clipped the
# operator's side and went for their feet. Model at least a 0.70 m standing
# footprint, while believing any larger observation.
PERSON_MIN_HALF_EXTENT_M = 0.35
# The chair is under 0.60 m wide. Keep its centre 0.50 m from obstacle
# returns: 0.30 m physical half-width plus 0.20 m lateral reserve.
PERSON_BYPASS_CLEARANCE_M = 0.50
PERSON_BYPASS_SPEED_MPS = 0.35


class Threat(object):
    """The nearest thing in the corridor, and whether it is going to move."""

    def __init__(self, distance_m, motion, label="", lateral_m=None,
                 track_id=None, observed_stamp_s=None,
                 directly_observed=True, geometry_valid=True,
                 centre_forward_m=None, half_forward_m=None,
                 half_lateral_m=None, route_s_m=None, route_d_m=None,
                 on_route=True):
        self.distance_m = distance_m
        # Route coordinates of the box centre when the threat was judged
        # against a RouteFrame: station ahead of the chair along the route
        # and signed offset from it (left positive). None otherwise.
        self.route_s_m = route_s_m
        self.route_d_m = route_d_m
        # False only for a threat that is NOT on the route but sits in the
        # chair's straight gate zone (waypoint_follower.cluster_threat).
        # Such a threat may slow or stop the chair; it must never authorise
        # a bypass, because a bypass side-commit would forbid the very turn
        # that takes the chair back to the route (J corner, 11-04 / 11-07).
        self.on_route = on_route
        self.motion = motion
        self.label = label
        self.track_id = track_id
        self.observed_stamp_s = observed_stamp_s
        self.directly_observed = directly_observed
        self.geometry_valid = geometry_valid
        self.centre_forward_m = centre_forward_m
        self.half_forward_m = half_forward_m
        self.half_lateral_m = half_lateral_m
        # Signed offset from the corridor centreline, chair frame, left
        # positive. None when the producer did not give a parseable box.
        #
        # Kept because dropping it makes every threat frontal. A planner
        # given only a distance can do nothing but place the object dead
        # ahead, so a wall 0.70 m away on the inside of a bend became a
        # phantom in the middle of the corridor and killed every candidate
        # rollout - 211 consecutive DWA_OBSTACLE holds on 2026-08-09 at a
        # station 0.3 m wide, where passing it on the far side was the whole
        # manoeuvre available.
        self.lateral_m = lateral_m

    @property
    def parked(self):
        """True only when the producer has watched it stand still.

        UNKNOWN is deliberately not parked: it is what a track looks like
        before it has been seen long enough, and before that the honest
        answer is that this could be someone about to step out.
        """
        return self.motion == STATIC

    @property
    def is_person(self):
        """A person, however still they are standing.

        The label is the producer's, and anything it cannot name is not
        given the benefit of the doubt in the other direction either: this
        only ever adds caution, never removes it.
        """
        return str(self.label).strip().lower() == PERSON_LABEL


class Summary(object):
    """One cycle of the producer's output, already validated."""

    def __init__(self, stamp_s, status, objects):
        self.stamp_s = stamp_s
        self.status = status
        self.objects = objects

    @property
    def usable(self):
        return self.status == "OK"


def parse_summary(payload):
    """Parse /perception/objects_summary, or raise ValueError.

    Raising is deliberate: the caller has to distinguish "no objects" from
    "no idea", and a parser returning an empty list for both would make a
    crashed producer look like clear road.
    """
    try:
        data = json.loads(payload)
    except (TypeError, ValueError) as error:
        raise ValueError("objects_summary is not JSON: %s" % error)
    if not isinstance(data, dict):
        raise ValueError("objects_summary is not an object")
    stamp = data.get("stamp")
    if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        raise ValueError("objects_summary has no usable stamp")
    objects = data.get("objects")
    if not isinstance(objects, list):
        raise ValueError("objects_summary has no object list")
    return Summary(float(stamp), str(data.get("status", "")), objects)


def object_box(item):
    """(x, y, half_x, half_y) for one object, or None if it does not parse."""
    try:
        x = float(item["x"])
        y = float(item["y"])
        size = item["size"]
        half_x = abs(float(size[0])) / 2.0
        half_y = abs(float(size[1])) / 2.0
    except (KeyError, IndexError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (x, y, half_x, half_y)):
        return None
    if str(item.get("class", "")).strip().lower() == PERSON_LABEL:
        half_x = max(half_x, PERSON_MIN_HALF_EXTENT_M)
        half_y = max(half_y, PERSON_MIN_HALF_EXTENT_M)
    return x, y, half_x, half_y


def object_motion(item):
    motion = item.get("motion", UNKNOWN)
    return motion if motion in (STATIC, MOVING, UNKNOWN) else MOVING


def object_reach(item, lateral_shift_m):
    """(lateral gap, forward distance, motion) for one object's box.

    The lateral gap is how far the near side of the box sits from the
    chair's centre line; negative means it straddles the line. Forward
    distance is the near face, clamped at zero - an object whose box already
    contains the chair is not at a negative distance, it is here.

    This is the fallback. The near face of an AXIS-ALIGNED box is a corner
    of the bounding volume, and for anything not aligned with the chair
    there are no returns there: a wall crossing the scan diagonally reported
    0.69 m on 2026-07-31 with its nearest return in the corridor at 2.13 m.
    Where the producer supplies a lateral profile, corridor_reach measures
    from that instead.
    """
    box = object_box(item)
    if box is None:
        return -1.0, BLOCKED, MOVING
    x, y, half_x, half_y = box
    return (abs(y - lateral_shift_m) - half_y, max(0.0, x - half_x),
            object_motion(item))


def parsed_profile(item):
    """(bin_m, y0, slices) for a present, structurally sound profile.

    None when the object carries no profile at all - that is the fallback
    to its box. BROKEN when a profile is present but its envelope does not
    parse, which is NOT a fallback: a producer emitting a broken profile is
    one whose box is no more trustworthy.

    Individual slice values are left unread here on purpose. profile_reach
    never looked at a slice outside the corridor it was asked about, so a
    broken value out there has never blocked, and parsing eagerly to share
    this would quietly make it start.
    """
    profile = item.get("profile")
    if profile is None:
        return None
    if not isinstance(profile, dict):
        return BROKEN
    try:
        bin_m = float(profile["bin_m"])
        y0 = float(profile["y0"])
        slices = profile["min_x"]
    except (KeyError, TypeError, ValueError):
        return BROKEN
    if not isinstance(slices, list) or not slices or \
            not math.isfinite(bin_m) or bin_m <= 0.0 or not math.isfinite(y0):
        return BROKEN
    return bin_m, y0, slices


def object_geometry_valid(item):
    """Whether current box/profile geometry can authorize a maneuver."""
    if object_box(item) is None:
        return False
    profile = parsed_profile(item)
    if profile is BROKEN:
        return False
    if profile is None:
        return True
    try:
        return all(
            value is None or math.isfinite(float(value))
            for value in profile[2])
    except (TypeError, ValueError):
        return False


def slices_in(profile, low, high):
    """(lateral centre, forward distance) per slice overlapping [low, high].

    Raises ValueError for a slice the producer wrote unreadably, which every
    caller turns into a block at zero. A slice counts when it overlaps the
    window at all, so a return sitting just inside a slice that straddles
    the corridor edge is never missed.
    """
    bin_m, y0, slices = profile
    for index, value in enumerate(slices):
        if value is None:
            continue
        if y0 + (index + 1) * bin_m <= low or y0 + index * bin_m >= high:
            continue
        x = float(value)
        if not math.isfinite(x):
            raise ValueError("profile slice %d is not finite" % index)
        yield y0 + (index + 0.5) * bin_m, max(0.0, x)


def profile_reach(item, lateral_shift_m, half_width_m):
    """(blocks, distance) from the object's own returns, or None.

    None means this object carries no profile and the caller must fall back
    to its box. A profile that is PRESENT but unreadable is not a fallback:
    it blocks at zero, because a producer emitting a broken profile is one
    whose box is no more trustworthy, and the fallback would quietly restore
    the very over-approximation this exists to remove.

    A slice counts when it overlaps the corridor at all, so a return sitting
    just inside a slice that straddles the corridor edge is never missed.
    """
    profile = parsed_profile(item)
    if profile is None or profile is BROKEN:
        return None if profile is None else (True, BLOCKED)
    try:
        reach = [x for _, x in slices_in(
            profile, lateral_shift_m - half_width_m,
            lateral_shift_m + half_width_m)]
    except (TypeError, ValueError):
        return True, BLOCKED
    if not reach:
        return False, None
    return True, min(reach)


def object_points(item, lateral_shift_m, half_width_m):
    """(blocks, points) for one object: where its returns actually are.

    points are ``(forward, left)`` in the chair frame, one per lateral slice
    the object occupies - the same measurement profile_reach takes the
    minimum of, handed over whole instead. Absolute chair-frame offsets, not
    the corridor-relative ``Threat.lateral_m`` beside them, because a
    consumer placing them in the map needs where they are and not where they
    are relative to a lane.

    A planner given one distance can only put the object dead ahead, and a
    planner given one point can only put it at one place. Neither is what a
    wall is. On 2026-07-31 a wall crossing the scan diagonally spanned the
    corridor for metres while its box reported a single near face 0.69 m
    away that had no returns behind it; measuring from the returns fixed the
    distance, and this fixes the shape, which is the half that was left. An
    arc scored against one point can pass 0.41 m from it and be admitted
    while driving through the rest of the wall.

    Fails closed exactly as corridor_reach does: an object whose box does not
    parse, or whose profile is present but unreadable, blocks at zero range
    on the centreline - a point no candidate can clear - rather than being
    skipped. Where there is no profile the box's near face is sampled across
    its own width, which is everything the box actually asserts.
    """
    box = object_box(item)
    if box is None:
        return True, [(BLOCKED, lateral_shift_m)]
    low = lateral_shift_m - half_width_m
    high = lateral_shift_m + half_width_m
    profile = parsed_profile(item)
    if profile is BROKEN:
        return True, [(BLOCKED, lateral_shift_m)]
    if profile is not None:
        try:
            points = [(x, y) for y, x in slices_in(profile, low, high)]
        except (TypeError, ValueError):
            return True, [(BLOCKED, lateral_shift_m)]
        return (True, points) if points else (False, [])
    x, y, half_x, half_y = box
    if abs(y - lateral_shift_m) - half_y > half_width_m:
        return False, []
    near = max(0.0, x - half_x)
    edges = (max(y - half_y, low), min(y + half_y, high))
    span = edges[1] - edges[0]
    count = max(int(math.ceil(span / BOX_SAMPLE_M)), 1)
    return True, [(near, edges[0] + span * (k + 0.5) / count)
                  for k in range(count)]


def corridor_reach(item, lateral_shift_m, half_width_m):
    """(blocks, distance, motion) for one object against one corridor.

    The single place the question "is this in the way, and how far" is
    answered. Measures from the object's returns where they are published
    and from its box otherwise; anything that does not parse blocks at zero
    and reads as MOVING, so a producer bug can neither hide an obstacle nor
    make one look parked enough to drive around.
    """
    if object_box(item) is None:
        return True, BLOCKED, MOVING
    motion = object_motion(item)
    measured = profile_reach(item, lateral_shift_m, half_width_m)
    if measured is not None:
        blocks, distance = measured
        if not blocks:
            return False, None, motion
        return True, BLOCKED if distance is None else distance, motion
    gap, distance, _ = object_reach(item, lateral_shift_m)
    if gap > half_width_m:
        return False, None, motion
    return True, distance, motion


# Route-frame threat geometry (2026-11-08).
#
# The straight strip above answers "is it in front of the chair". In a bend
# that is the wrong question: at the J corner of route2 the outer wall, a
# permanent structure 0.5-0.9 m outside the band, stood dead ahead while the
# route turned left, was judged a parked obstacle 7-8 m away and armed a
# static-object bypass whose side-commit then forbade the left turn (11-04,
# 11-07 drives 2 and 3, reproduced by bag replay). A RouteFrame answers "is it
# on the route ahead": every return is projected onto the route near the
# chair and judged by (station ahead along the route, offset from it).
ROUTE_FRAME_BACK_M = 2.0
ROUTE_FRAME_EXTRA_M = 2.0
ROUTE_SAMPLE_M = 0.2
ROUTE_MAX_SAMPLES_PER_OBJECT = 600


class RouteFrame(object):
    """The route around the chair, for judging chair-frame points against.

    Only the segments from ROUTE_FRAME_BACK_M behind the chair's station to
    ROUTE_FRAME_EXTRA_M past the horizon are projected onto, so a route that
    passes near itself (an out-and-back, a U-turn) cannot claim a point for
    the wrong leg. The horizon stops at the route's end: nothing past the
    goal is somewhere the chair will drive (11-07, 88 s DWA_WAIT at D4 for
    objects 6 m beyond the end).
    """

    def __init__(self, route_xy, arc_m, pose_xy, pose_yaw, chair_s_m,
                 chair_d_m, horizon_m):
        self.route = route_xy
        self.arc = arc_m
        self.pose_xy = (float(pose_xy[0]), float(pose_xy[1]))
        self.pose_yaw = float(pose_yaw)
        self.chair_s_m = float(chair_s_m)
        self.chair_d_m = float(chair_d_m)
        self.horizon_m = max(0.0, min(float(horizon_m),
                                      float(arc_m[-1]) - self.chair_s_m))
        # Set by the caller while a bypass is active: the line the chair is
        # actually driving, parallel to the route at chair_d_m, then counts
        # as part of the corridor too. Off by default - with it on, a chair
        # pushed 0.9 m outside the J corner would find the outer wall on
        # "its" line and re-arm the bypass that pushed it there.
        self.follow_chair_line = False
        delta = np.diff(route_xy, axis=0)
        self._length = np.linalg.norm(delta, axis=1)
        self._direction = delta / np.maximum(self._length, 1e-9)[:, None]
        lo_s = self.chair_s_m - ROUTE_FRAME_BACK_M
        hi_s = self.chair_s_m + self.horizon_m + ROUTE_FRAME_EXTRA_M
        self._lo = int(max(np.searchsorted(arc_m, lo_s) - 1, 0))
        self._hi = int(min(np.searchsorted(arc_m, hi_s) + 1, len(delta)))
        self._hi = max(self._hi, self._lo + 1)

    @classmethod
    def build(cls, route_xy, arc_m, pose_xy, pose_yaw, nearest_index,
              horizon_m, search_segments=10):
        """A frame for this pose, or None when there is nothing to follow."""
        try:
            route = np.asarray(route_xy, dtype=float).reshape(-1, 2)
            arc = np.asarray(arc_m, dtype=float).reshape(-1)
            px, py = float(pose_xy[0]), float(pose_xy[1])
            yaw = float(pose_yaw)
            index = int(nearest_index)
        except (TypeError, ValueError, IndexError):
            return None
        if len(route) < 2 or len(arc) != len(route) or \
                not all(math.isfinite(v) for v in (px, py, yaw)):
            return None
        segments = len(route) - 1
        index = min(max(index, 0), segments)
        lo = max(index - int(search_segments), 0)
        hi = min(index + int(search_segments), segments)
        hi = max(hi, lo + 1)
        s, d = _project(route, arc, np.array([[px, py]]), lo, hi)
        return cls(route, arc, (px, py), yaw, s[0], d[0], horizon_m)

    def to_map(self, points):
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        c, s = math.cos(self.pose_yaw), math.sin(self.pose_yaw)
        return np.stack([self.pose_xy[0] + c * pts[:, 0] - s * pts[:, 1],
                         self.pose_xy[1] + s * pts[:, 0] + c * pts[:, 1]],
                        axis=1)

    def project(self, chair_points):
        """(station ahead of the chair, offset) for chair-frame points."""
        s, d = _project(self.route, self.arc, self.to_map(chair_points),
                        self._lo, self._hi)
        return s - self.chair_s_m, d

    def in_corridor(self, ahead, offset, half_width_m, lateral_shift_m=0.0,
                    max_ahead_m=None):
        reach = self.horizon_m if max_ahead_m is None else min(
            self.horizon_m, float(max_ahead_m))
        inside = np.abs(offset - float(lateral_shift_m)) < half_width_m
        if self.follow_chair_line:
            inside |= np.abs(offset - self.chair_d_m) < half_width_m
        return inside & (ahead >= 0.0) & (ahead <= reach)


def _project(route, arc, points, lo, hi):
    """(arc station, signed left offset) of map points on segments [lo, hi).

    Each point takes the segment it is nearest to, so the station is
    continuous rather than snapping vertex to vertex.
    """
    a = route[lo:hi]
    delta = route[lo + 1:hi + 1] - a
    length = np.linalg.norm(delta, axis=1)
    direction = delta / np.maximum(length, 1e-9)[:, None]
    rel = points[:, None, :] - a[None, :, :]
    along = np.clip((rel * direction[None]).sum(axis=2), 0.0, length[None])
    foot = a[None] + direction[None] * along[:, :, None]
    off = points[:, None, :] - foot
    gap = np.hypot(off[:, :, 0], off[:, :, 1])
    k = np.argmin(gap, axis=1)
    rows = np.arange(len(points))
    station = arc[lo:hi][k] + along[rows, k]
    # Past either END of the route a point is not alongside it: carry its
    # station on beyond the end (or before the start) instead of clamping it
    # onto the last vertex, so the horizon test can exclude it.
    raw = (rel[rows, k] * direction[k]).sum(axis=1)
    last = (lo + k) == len(route) - 2
    first = (lo + k) == 0
    station = np.where(last & (raw > length[k]), arc[lo:hi][k] + raw, station)
    station = np.where(first & (raw < 0.0), arc[lo:hi][k] + raw, station)
    u = direction[k]
    offset = u[:, 0] * off[rows, k, 1] - u[:, 1] * off[rows, k, 0]
    return station, offset


def object_chair_points(item):
    """Chair-frame (forward, left) samples of where an object is, or BROKEN.

    With a profile, each lateral slice runs from its nearest return to the
    box's far face: the profile keeps only the nearest return per slice, and
    a route bending round an object can cross it well behind that near face.
    Without one, the box perimeter - a corridor cannot enter the box without
    crossing it, and the spacing is finer than any corridor half-width.

    A profile slice that does not read is BROKEN wherever it lies. That is
    stricter than corridor_reach, which never reads a slice outside its
    strip, but a route corridor can run through any of them.
    """
    box = object_box(item)
    if box is None:
        return BROKEN
    x, y, half_x, half_y = box
    far = x + half_x
    profile = parsed_profile(item)
    if profile is BROKEN:
        return BROKEN
    columns = []
    if profile is not None:
        try:
            columns = [(lateral, near) for lateral, near in
                       slices_in(profile, -math.inf, math.inf)]
        except (TypeError, ValueError):
            return BROKEN
    points = []
    if columns:
        depth = max(max(far - near for _, near in columns), 0.0)
        rows = int(math.ceil(depth / ROUTE_SAMPLE_M)) + 1
        if rows * len(columns) > ROUTE_MAX_SAMPLES_PER_OBJECT:
            rows = max(ROUTE_MAX_SAMPLES_PER_OBJECT // len(columns), 2)
        for lateral, near in columns:
            top = max(far, near)
            for k in range(rows):
                fraction = 0.0 if rows == 1 else k / float(rows - 1)
                points.append((near + (top - near) * fraction, lateral))
    else:
        near = max(0.0, x - half_x)
        across = max(int(math.ceil(2.0 * half_y / ROUTE_SAMPLE_M)), 1)
        along = max(int(math.ceil((far - near) / ROUTE_SAMPLE_M)), 1)
        for k in range(across + 1):
            lateral = y - half_y + 2.0 * half_y * k / across
            points += [(near, lateral), (far, lateral)]
        for k in range(1, along):
            forward = near + (far - near) * k / along
            points += [(forward, y - half_y), (forward, y + half_y)]
    return np.asarray(points, dtype=float)


def route_reach(item, frame, half_width_m, lateral_shift_m=0.0):
    """(blocks, distance, motion) against a RouteFrame corridor.

    The route counterpart of corridor_reach, with the same contract and the
    same failures: a box that does not parse blocks at zero as MOVING, a
    profile that does not read blocks at zero keeping its motion (which is
    what corridor_reach does; geometry_valid already stops it authorising a
    manoeuvre). distance is travel along the route to the nearest return in
    the corridor.
    """
    if object_box(item) is None:
        return True, BLOCKED, MOVING
    motion = object_motion(item)
    points = object_chair_points(item)
    if points is BROKEN:
        return True, BLOCKED, motion
    ahead, offset = frame.project(points)
    inside = frame.in_corridor(ahead, offset, half_width_m, lateral_shift_m)
    if not inside.any():
        return False, None, motion
    return True, max(0.0, float(ahead[inside].min())), motion


def matching_threats(summary, half_width_m, lateral_shift_m=0.0,
                     labels=None, route_frame=None):
    """Every matching object overlapping the corridor, nearest first.

    With route_frame the corridor is the route's (route_reach); without it,
    or for a shifted corridor, the original straight strip, unchanged.
    """
    if not summary.usable:
        return [Threat(BLOCKED, MOVING, summary.status or "unusable",
                       directly_observed=False, geometry_valid=False)]
    wanted = None if labels is None else {
        str(label).strip().lower() for label in labels}
    use_route = route_frame is not None and abs(float(lateral_shift_m)) < 1e-9
    found = []
    for item in summary.objects:
        label = str(item.get("class", "")).strip().lower()
        if wanted is not None and label not in wanted:
            continue
        if use_route:
            blocks, distance, motion = route_reach(
                item, route_frame, half_width_m)
        else:
            blocks, distance, motion = corridor_reach(
                item, lateral_shift_m, half_width_m)
        if not blocks:
            continue
        box = object_box(item)
        lateral = None if box is None else float(box[1]) - lateral_shift_m
        track_id = item.get("id")
        if not isinstance(track_id, int) or isinstance(track_id, bool):
            track_id = None
        route_s = route_d = None
        if use_route and box is not None:
            ahead, offset = route_frame.project([(box[0], box[1])])
            route_s, route_d = float(ahead[0]), float(offset[0])
        found.append(Threat(
            distance, motion, str(item.get("class", "")),
            lateral_m=lateral, track_id=track_id,
            observed_stamp_s=summary.stamp_s,
            geometry_valid=object_geometry_valid(item),
            centre_forward_m=None if box is None else float(box[0]),
            half_forward_m=None if box is None else float(box[2]),
            half_lateral_m=None if box is None else float(box[3]),
            route_s_m=route_s, route_d_m=route_d))
    return sorted(found, key=lambda threat: threat.distance_m)


def nearest_threat(summary, half_width_m, lateral_shift_m=0.0,
                   labels=None, route_frame=None):
    """The nearest matching object overlapping the corridor, or None.

    None means nothing is in the way. It never means "could not tell": an
    unusable summary reports a blocking threat, so a caller treating None
    as clear road is right to. When labels is provided, only objects whose
    normalized class is in it are considered.
    """
    threats = matching_threats(
        summary, half_width_m, lateral_shift_m, labels,
        route_frame=route_frame)
    return threats[0] if threats else None


def corridor_obstacle_points(summary, half_width_m, lateral_shift_m=0.0,
                             max_distance_m=None,
                             max_objects=MAX_OBSTACLE_OBJECTS,
                             exclude_track_ids=(), route_frame=None):
    """(blocks, points) for everything in the way, as geometry.

    The planner-facing companion to nearest_threat. That one answers "is
    there something and how far", which is what a stopping radius needs;
    this answers "and what shape is it", which is what a planner choosing
    between arcs around it needs, and the two are not the same question.

    Several objects, not just the nearest. The single-object argument holds
    for a distance - a second object behind the first constrains no
    candidate the first already kills - and stops holding for a shape: an
    object off to one side kills the arcs on that side only, and what
    forbids the other side is a different object the nearest one never
    covered.

    Never more than max_objects of them, nearest first. Scoring is O(
    candidates x steps x points) in a 0.1 s period, and the producer will
    publish up to forty clusters.

    An unusable summary blocks at zero on the centreline, so a caller that
    treats an empty point list as clear road is right to.

    With route_frame, an object's samples within half_width_m of the route
    ahead, or of the line the chair is on (parallel to the route at its
    offset), are added to the straight strip's. A planner going round
    something in a bend has to see what stands round the bend, and the
    straight strip alone runs off the outside of it.
    """
    if not summary.usable:
        return True, [(BLOCKED, lateral_shift_m)]
    excluded = {
        value for value in exclude_track_ids
        if isinstance(value, int) and not isinstance(value, bool)}
    found = []
    for item in summary.objects:
        if item.get("id") in excluded:
            continue
        blocks, points = object_points(item, lateral_shift_m, half_width_m)
        if route_frame is not None and object_box(item) is not None:
            samples = object_chair_points(item)
            if samples is BROKEN:
                blocks, points = True, [(BLOCKED, lateral_shift_m)]
            else:
                ahead, offset = route_frame.project(samples)
                inside = (np.abs(offset) < half_width_m) | \
                    (np.abs(offset - route_frame.chair_d_m) < half_width_m)
                inside &= ahead >= 0.0
                if max_distance_m is not None:
                    inside &= ahead <= float(max_distance_m)
                extra = [(float(fx), float(fy))
                         for fx, fy in samples[inside]]
                if extra:
                    blocks, points = True, list(points or []) + extra
        if not blocks or not points:
            continue
        nearest = min(x for x, _ in points)
        if max_distance_m is not None and nearest > float(max_distance_m):
            continue
        found.append((nearest, points))
    found.sort(key=lambda entry: entry[0])
    if max_objects is not None:
        found = found[:int(max_objects)]
    if not found:
        return False, []
    return True, [point for _, points in found for point in points]


GO_ROUND = "go_round"
PERSON_BYPASS = "person_bypass"
WAIT = "wait"
CLEAR = "clear"


def avoidance_decision(threat, blocking, blocked_for_s, plan_ahead_m,
                       bypass_after_s, person_bypass_ready=False):
    """What to do about the nearest thing in the corridor.

    GO_ROUND for something the tracker has watched stand still, and taken
    while it is still plan_ahead_m away so the chair drifts past rather than
    driving up to it and stopping first.

    WAIT for anything moving, or not yet watched long enough to say.
    Stepping around a person is a manoeuvre into where they are about to be.
    Nothing here resumes the chair explicitly: once they leave the corridor
    the threat is gone, the answer becomes CLEAR, and it drives on.

    A person is waited out by default. The caller may supply a separate
    person_bypass_ready authorization only after longer, direct same-track
    STATIC evidence; that produces a distinct result so only a controller
    with current geometry and hard-mask rollout checks can execute it.
    Standing still for the tracker's CONFIRM_S alone is never enough.

    blocked_for_s is the fallback for sources that carry no identity. A
    raw-scan return is UNKNOWN forever, so standing in the way is the only
    evidence of parkedness it can ever offer - but it never overrules a
    tracker that says the thing is moving.
    """
    if threat is None:
        return CLEAR
    if threat.is_person:
        if threat.parked and threat.distance_m < plan_ahead_m:
            return PERSON_BYPASS if person_bypass_ready else WAIT
        return WAIT if blocking else CLEAR
    if threat.parked and threat.distance_m < plan_ahead_m:
        return GO_ROUND
    if blocking and blocked_for_s is not None and \
            blocked_for_s > bypass_after_s and threat.motion != MOVING:
        return GO_ROUND
    return WAIT if blocking else CLEAR


def is_stale(summary_stamp_s, now_s, stale_s=STALE_S):
    """True when the producer has gone quiet - or has never spoken.

    A never-seen producer is stale, not clear: the node not being launched
    must not read as an empty road.
    """
    if summary_stamp_s is None:
        return True
    if not math.isfinite(now_s) or not math.isfinite(summary_stamp_s):
        return True
    return (now_s - summary_stamp_s) > stale_s


# Preference order for a step around a parked object. Sizes are what the
# corridor is asked for; bypass_offsets_for_room decides what it can give.
BYPASS_OFFSETS = (0.6, -0.6, 1.0, -1.0)
# The largest step this will ever take, and the room it keeps between the
# offset chair's outer edge and the band edge.
#
# The ladder above is a preference order, not a set of achievable offsets.
# On 2026-08-16 the chair took a 0.60 m step in a corridor whose band was
# 1.70 m wide - 0.85 m either side of the line - and the operator stopped
# it. The step was legal: band.contains() tests the centre point, and an
# edge with no drop behind it only insets EDGE_MARGIN, by design, because
# that is what buys room to pass obstacles at all. But the chair is 0.70 m
# wide, so 0.60 m off the line put its outer edge 0.275 m past the band.
# Legal and drivable are not the same number, and the ladder could not tell
# the difference because it never asked how much room was there.
BYPASS_OFFSET_MAX_M = 1.0
BYPASS_EDGE_KEEP_M = 0.25
# Below this the chair has not gone round anything, it has only drifted,
# and it still pays the whole cost of leaving the line.
BYPASS_OFFSET_MIN_M = 0.30
# Where the step is measured and checked. One list so the offset that gets
# sized and the offset that gets approved are talking about the same ground.
BYPASS_PROBE_AHEAD_M = (0.5, 1.5, 2.5, 3.5)


def bypass_offsets_for_room(left_room, right_room):
    """Offsets worth trying, given the room each side actually has.

    BYPASS_OFFSETS is the preference order; this is what the corridor can
    give. An entry survives at its own size when it fits, and is cut down
    to the side's room when it does not, so a narrow corridor is offered a
    step sized to it instead of a 0.60 m step admitted because the centre
    point happened to land inside the band. A side with less room than
    BYPASS_OFFSET_MIN_M is dropped: there is no step there worth the cost
    of leaving the line.

    ROS-free so it can be tested without a running follower.
    """
    room = {1.0: min(left_room, BYPASS_OFFSET_MAX_M),
            -1.0: min(right_room, BYPASS_OFFSET_MAX_M)}
    offsets = []
    for wanted in BYPASS_OFFSETS:
        side = 1.0 if wanted > 0 else -1.0
        allowed = room[side]
        if allowed < BYPASS_OFFSET_MIN_M:
            continue
        value = side * min(abs(wanted), allowed)
        if not any(abs(value - seen) < 1e-6 for seen in offsets):
            offsets.append(value)
    return offsets

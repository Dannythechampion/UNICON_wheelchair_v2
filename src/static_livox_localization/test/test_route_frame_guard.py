"""Route-frame obstacle judgement (2026-11-08).

The J corner of route2 bends left by ~90 degrees (radius 2.5 m). Its outer
wall - a permanent structure 0.5-0.9 m outside the band - stood dead ahead
of the chair while the route turned, so the straight chair-frame strip read
it as a parked obstacle on the route 7-8 m out. That armed
STATIC_OBJECT_BYPASS; the bypass committed RIGHT (the wall looked LEFT in
the chair frame though it was right of the route) and the side veto then
forbade every left turn, including the route's own. Bag replay of 11-04 run 1
and 11-07 drives 2 and 3 reproduces the first trigger at the logged time.

What changed, and what these tests pin:
  A  ordinary objects are judged against a RouteFrame (station ahead along
     the route, offset from it); a straight gate-zone supplement reports
     off-route returns the gate would stop for, but never as a bypass;
  B  the bypass side is chosen from the target's ROUTE offset;
  C  the side veto bounds turns relative to the route's own turn rate;
  D  the planner also sees what stands round the bend;
  E  a candidate's obstacle tail passes if its straight OR route-following
     continuation clears.
"""
import importlib.util
import json
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).parent
SCRIPTS = HERE.parent / "scripts"
sys.path.insert(0, str(SCRIPTS))

import cluster_guard as cg  # noqa: E402
import cluster_tracking as ct  # noqa: E402
import dwa_core  # noqa: E402


def _policy_loader():
    spec = importlib.util.spec_from_file_location(
        "route_frame_policy_loader", HERE / "test_dwa_policy.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.load_follower


load_follower = _policy_loader()

HALF_WIDTH = 0.45          # waypoint_follower.CORRIDOR_HALF_WIDTH
PLAN_AHEAD = 8.0           # waypoint_follower.PLAN_AHEAD_M
STEP = 0.2


def l_route():
    """East 10 m, then left round a 2.5 m radius, then north."""
    pts = [(k * STEP, 0.0) for k in range(int(7.5 / STEP) + 1)]
    for k in range(1, 21):
        a = k * (math.pi / 2) / 20
        pts.append((7.5 + 2.5 * math.sin(a), 2.5 - 2.5 * math.cos(a)))
    pts += [(10.0, 2.5 + k * STEP) for k in range(1, 51)]
    return np.array(pts)


def arc_of(route):
    return np.concatenate([[0.0], np.cumsum(
        np.linalg.norm(np.diff(route, axis=0), axis=1))])


def frame_at(x, y=0.0, yaw=0.0, route=None, horizon=PLAN_AHEAD):
    route = l_route() if route is None else route
    nearest = int(np.argmin(np.linalg.norm(route - (x, y), axis=1)))
    return cg.RouteFrame.build(route, arc_of(route), (x, y), yaw, nearest,
                               horizon)


def obj(x, y, size=(0.4, 0.4, 1.2), motion=ct.STATIC, cls="obstacle",
        track_id=7, profile=None):
    item = {"id": track_id, "class": cls, "x": x, "y": y, "z": 0.5,
            "size": list(size), "motion": motion, "points": 40}
    if profile is not None:
        item["profile"] = profile
    return item


def summary(objects, status="OK"):
    return cg.parse_summary(json.dumps(
        {"stamp": 100.0, "status": status, "objects": objects}))


# Chair 4 m before the bend, heading east. The outer wall stands across the
# east leg 2 m past where the bend starts: dead ahead in the chair frame,
# ~1.5 m outside the route once it has turned north.
CHAIR_X = 4.0
WALL = obj(7.6, 0.0, size=(0.3, 2.0, 1.5))


# ------------------------------------------------------------------ A


def test_the_straight_strip_reads_the_outer_corner_wall_as_on_the_route():
    threat = cg.nearest_threat(summary([WALL]), HALF_WIDTH)
    assert threat is not None and threat.parked
    assert threat.distance_m == pytest.approx(7.45, abs=0.01)


def test_the_route_frame_does_not():
    assert cg.nearest_threat(summary([WALL]), HALF_WIDTH,
                             route_frame=frame_at(CHAIR_X)) is None


def test_what_stands_on_the_route_round_the_bend_is_seen_along_it():
    # On the north leg 0.3 m past the bend: (10.0, 2.8) in the map.
    after = obj(6.0, 2.8)
    assert cg.nearest_threat(summary([after]), HALF_WIDTH) is None
    threat = cg.nearest_threat(summary([after]), HALF_WIDTH,
                               route_frame=frame_at(CHAIR_X))
    assert threat is not None
    # 3.5 m of east leg + a quarter circle (3.93 m) + 0.3 m - half the box.
    assert threat.distance_m == pytest.approx(3.5 + 3.93 + 0.3 - 0.2,
                                              abs=0.15)
    assert threat.route_d_m == pytest.approx(0.0, abs=0.05)
    assert threat.lateral_m == pytest.approx(2.8)   # chair frame, unchanged


def test_on_a_straight_route_the_route_frame_agrees_with_the_strip():
    route = np.array([(k * STEP, 0.0) for k in range(101)])
    frame = frame_at(2.0, route=route)
    for x, y in ((3.0, 0.0), (5.0, 0.5), (5.0, 0.7), (6.0, -0.62),
                 (7.5, 0.0)):
        item = obj(x, y)
        straight = cg.nearest_threat(summary([item]), HALF_WIDTH)
        routed = cg.nearest_threat(summary([item]), HALF_WIDTH,
                                   route_frame=frame)
        assert (straight is None) == (routed is None), (x, y)
        if straight is not None:
            assert routed.distance_m == pytest.approx(straight.distance_m,
                                                      abs=0.11)


def test_the_chair_line_counts_only_while_bypassing():
    route = np.array([(k * STEP, 0.0) for k in range(101)])
    frame = frame_at(2.0, 0.9, route=route)        # stepped 0.9 m left
    ahead = obj(3.0, 0.0)                          # on the chair's line
    assert cg.nearest_threat(summary([ahead]), HALF_WIDTH,
                             route_frame=frame) is None
    frame.follow_chair_line = True
    threat = cg.nearest_threat(summary([ahead]), HALF_WIDTH,
                               route_frame=frame)
    assert threat is not None
    assert threat.distance_m == pytest.approx(2.8, abs=0.11)


def test_a_route_passing_near_itself_is_not_projected_onto_the_far_leg():
    """Out 15 m along y=0, back along y=-0.8. A return 0.42 m right of the
    out leg is 0.38 m from the back leg - the nearer one - but the chair
    reaches the back leg only ~25 m on."""
    out = [(k * STEP, 0.0) for k in range(76)]
    back = [(15.0 - k * STEP, -0.8) for k in range(76)]
    route = np.array(out + back)
    frame = frame_at(1.0, route=route)
    threat = cg.nearest_threat(summary([obj(5.0, -0.42, size=(0.2, 0.02, 1))]),
                               HALF_WIDTH, route_frame=frame)
    assert threat is not None
    assert threat.distance_m == pytest.approx(4.9, abs=0.11)


def test_nothing_past_the_route_end_is_in_the_way():
    route = np.array([(k * STEP, 0.0) for k in range(31)])    # ends at 6 m
    beyond = obj(6.5, 0.0)                                    # 7.5 m map x
    assert cg.nearest_threat(summary([beyond]), HALF_WIDTH) is not None
    assert cg.nearest_threat(summary([beyond]), HALF_WIDTH,
                             route_frame=frame_at(1.0, route=route)) is None


def test_a_long_object_is_seen_where_the_bend_crosses_its_far_part():
    """The profile keeps only the nearest return per slice. A van parked
    along the east leg's left side, its near end 1 m left of the route
    3 m ahead, reaches back 5 m; the route turns north through its far
    part, which no near-face return describes."""
    profile = {"bin_m": 0.2, "y0": 2.7, "min_x": [3.0, 3.0]}
    van = obj(5.6, 2.8, size=(5.2, 0.4, 1.8), profile=profile)
    profile_only = dict(van, size=[0.02, 0.4, 1.8], x=3.01)
    assert cg.nearest_threat(summary([profile_only]), HALF_WIDTH,
                             route_frame=frame_at(CHAIR_X)) is None
    threat = cg.nearest_threat(summary([van]), HALF_WIDTH,
                               route_frame=frame_at(CHAIR_X))
    assert threat is not None


def test_a_broken_slice_blocks_wherever_it_lies():
    profile = {"bin_m": 0.2, "y0": -1.0, "min_x": [7.5, "x", 7.5]}
    threat = cg.nearest_threat(summary([obj(7.6, -0.8, profile=profile)]),
                               HALF_WIDTH, route_frame=frame_at(CHAIR_X))
    assert threat.distance_m == cg.BLOCKED
    assert not threat.geometry_valid


def test_an_unparsable_box_blocks_as_moving():
    threat = cg.nearest_threat(summary([{"class": "obstacle", "x": 3.0,
                                         "y": 9.0}]),
                               HALF_WIDTH, route_frame=frame_at(CHAIR_X))
    assert threat.distance_m == cg.BLOCKED and threat.motion == ct.MOVING


def test_an_unusable_summary_still_blocks():
    threat = cg.nearest_threat(summary([], status="NO_CLOUD"), HALF_WIDTH,
                               route_frame=frame_at(CHAIR_X))
    assert threat.distance_m == cg.BLOCKED


def test_a_shifted_corridor_stays_straight():
    assert cg.nearest_threat(summary([obj(7.6, 0.6)]), HALF_WIDTH, 0.6,
                             route_frame=frame_at(CHAIR_X)) is not None


def test_no_frame_without_a_route():
    assert cg.RouteFrame.build(np.zeros((1, 2)), [0.0], (0, 0), 0.0, 0,
                               PLAN_AHEAD) is None


# ------------------------------------------------------------------ D


def test_the_planner_sees_what_stands_round_the_bend():
    after = obj(6.0, 2.8)
    blocks, points = cg.corridor_obstacle_points(
        summary([after]), 1.0, max_distance_m=PLAN_AHEAD)
    assert not blocks
    blocks, points = cg.corridor_obstacle_points(
        summary([after]), 1.0, max_distance_m=PLAN_AHEAD,
        route_frame=frame_at(CHAIR_X))
    assert blocks and points


# ---------------------------------------------- A: follower wiring


def follower_on_l_route(objects, x=CHAIR_X, y=0.0, yaw=0.0,
                        tracking="TRACKING", mode="route",
                        name="waypoint_follower", cls="WaypointFollower"):
    module, _Stamp = load_follower(name)
    follower = getattr(module, cls).__new__(getattr(module, cls))
    route = l_route()
    follower.waypoints = route
    follower.route_arc = arc_of(route)
    follower.threat_corridor = mode
    follower.threat_route_frame = None
    follower.pose_xy = np.array([x, y])
    follower.pose_yaw = yaw
    follower.route_locked = True
    follower.nearest_index = int(np.argmin(np.linalg.norm(route - (x, y),
                                                          axis=1)))
    follower.tracking_state = tracking
    follower.clusters_enabled = True
    follower.cluster_summary = summary(objects)
    follower.person_memory = None
    follower.direct_person_threats = ()
    follower.person_bypass_committed_track_id = None
    follower.blocked_since = None
    follower.cluster_stop_radius = lambda: 2.0       # gate zone 2.6 m
    return module, follower


def test_follower_ignores_the_outer_wall_in_route_mode():
    _module, follower = follower_on_l_route([WALL])
    assert follower.cluster_threat() is None
    assert follower.threat_route_frame is not None


@pytest.mark.parametrize("tracking,mode", [("DEGRADED", "route"),
                                           ("TRACKING", "straight")])
def test_follower_falls_back_to_the_straight_strip(tracking, mode):
    _module, follower = follower_on_l_route([WALL], tracking=tracking,
                                            mode=mode)
    threat = follower.cluster_threat()
    assert threat is not None and threat.parked


def test_an_off_route_return_in_the_gate_zone_slows_but_never_goes_round():
    # The chair has swung 60 degrees left of the route at x=4; a parked
    # object 2 m dead ahead is 1.7 m off the route, inside the gate zone.
    module, follower = follower_on_l_route(
        [obj(2.2, 0.0)], yaw=math.radians(60))
    threat = follower.cluster_threat()
    assert threat is not None and threat.parked
    assert threat.on_route is False
    assert threat.distance_m == pytest.approx(2.0, abs=0.01)
    now = module.rospy.Time.now()
    assert follower.avoidance_for(now, threat, False) == module.CLEAR
    assert follower.avoidance_for(now, threat, True) == module.WAIT


def test_the_same_object_on_the_route_keeps_its_bypass():
    module, follower = follower_on_l_route([obj(2.2, 0.0)])
    threat = follower.cluster_threat()
    assert threat.on_route is True
    now = module.rospy.Time.now()
    assert follower.avoidance_for(now, threat, False) == module.GO_ROUND


def test_beyond_the_gate_zone_an_off_route_object_is_not_reported():
    _module, follower = follower_on_l_route([obj(4.0, 0.0)],
                                            yaw=math.radians(60))
    assert follower.cluster_threat() is None


def test_a_person_off_the_route_in_the_straight_strip_is_still_a_threat():
    person = obj(5.0, 0.0, cls="person", motion=ct.MOVING)
    _module, follower = follower_on_l_route([person], yaw=math.radians(60))
    threat = follower.cluster_threat()
    assert threat is not None and threat.is_person


def test_gate_zone_constants_match_safety_gate():
    spec = importlib.util.spec_from_file_location(
        "rf_safety_gate_src", SCRIPTS / "safety_gate.py")
    text = Path(spec.origin).read_text(encoding="utf-8")
    module, _ = load_follower("waypoint_follower")
    assert "HALF_WIDTH_M = %s" % module.GATE_ZONE_HALF_WIDTH_M in text
    assert "FORWARD_CHECK_EXTRA_M = %s" % module.GATE_ZONE_EXTRA_M in text


# ------------------------------------------------------------------ B, C


def bypass_follower(x=CHAIR_X, y=0.0, yaw=0.0, speed=0.6):
    module, follower = follower_on_l_route(
        [], x=x, y=y, yaw=yaw, name="person_bypass_dwa_follower",
        cls="PersonBypassDwaFollower")
    planner = dwa_core.DwaPlanner.__new__(dwa_core.DwaPlanner)
    dwa_core.DwaPlanner.__init__(planner, None, l_route())
    follower.planner = planner
    follower.current_speed = speed
    follower.threat_route_frame = follower.make_route_frame()
    return module, follower


def test_the_veto_margin_is_one_yaw_sample():
    module, _ = load_follower("person_bypass_dwa_follower")
    step = 2 * dwa_core.MAX_YAW_RATE / (dwa_core.YAW_SAMPLES - 1)
    assert module.PersonBypassDwaFollower.SIDE_VETO_MARGIN_RPS == \
        pytest.approx(step)


def test_on_a_straight_the_side_veto_is_the_old_rule():
    _module, follower = bypass_follower(x=2.0)
    assert follower.side_veto_bound(0.6, -1) == 0.0
    assert follower.side_veto_bound(0.6, 1) == 0.0


def test_in_the_bend_a_right_commit_may_still_follow_the_left_turn():
    # On the arc: 0.6 m/s on a 2.5 m radius is +0.24 rad/s.
    _module, follower = bypass_follower(
        x=7.5 + 2.5 * math.sin(0.3), y=2.5 - 2.5 * math.cos(0.3), yaw=0.3)
    route_w = follower.route_turn_rate(0.6)
    assert route_w == pytest.approx(0.24, abs=0.03)
    bound = follower.side_veto_bound(0.6, -1)
    assert bound == pytest.approx(route_w + 0.05)
    # The planner's own +0.25 sample survives a RIGHT commit; +0.35 does not.
    assert (0.25 - bound) * -1 >= -1e-6
    assert (0.35 - bound) * -1 < -1e-6
    # A LEFT commit may not turn less than the bend.
    assert follower.side_veto_bound(0.6, 1) == pytest.approx(route_w - 0.05)


def test_the_side_is_chosen_from_the_route_offset_not_the_chair_frame():
    """At the bend a target right of the route reads LEFT from a chair
    still facing east - the 11-07 J case, which committed RIGHT."""
    _module, follower = bypass_follower(
        x=7.5 + 2.5 * math.sin(0.3), y=2.5 - 2.5 * math.cos(0.3), yaw=0.3)
    # Map point 0.7 m right (east) of the north leg, 1 m up it.
    target_map = np.array([10.7, 3.5])
    c, s = math.cos(0.3), math.sin(0.3)
    rel = target_map - follower.pose_xy
    tx, ty = c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1]
    assert ty > 0.0                                   # looks left
    permit = types.SimpleNamespace(target_x_m=tx, target_y_m=ty,
                                   min_clearance_m=0.5)
    offset = follower.permit_route_offset(permit)
    assert offset == pytest.approx(-0.7, abs=0.05)    # right of the route


# ------------------------------------------------------------------ E


@pytest.fixture
def planner():
    p = dwa_core.DwaPlanner.__new__(dwa_core.DwaPlanner)
    dwa_core.DwaPlanner.__init__(p, None, l_route())
    return p


def cpu_query(points):
    from scipy.spatial import cKDTree
    tree = cKDTree(points)
    return lambda flat, _p: tree.query(flat)[0]


def test_either_is_the_default_extension():
    assert dwa_core.OBSTACLE_EXTENSION == "either"
    assert set(dwa_core.OBSTACLE_EXTENSION_MODES) == {"either", "straight"}


def test_a_bend_candidate_is_not_killed_by_the_outer_wall(planner):
    # An arc ending in the bend, 0.36 m outside it and turned only 10
    # degrees; the outer wall stands 1.5 m east of the north leg.
    wall = np.array([(11.5, y) for y in np.arange(-1.0, 1.01, 0.1)])
    paths = np.array([[[8.8, 0.0, 0.17]]])
    planner.obstacle_extension = "straight"
    straight = planner._obstacle_clearance(paths, 0.1, wall, cpu_query(wall))
    planner.obstacle_extension = "either"
    either = planner._obstacle_clearance(paths, 0.1, wall, cpu_query(wall))
    assert straight[0] < dwa_core.OBSTACLE_FLOOR_M
    assert either[0] > dwa_core.OBSTACLE_FLOOR_M


def test_a_real_way_round_an_object_on_the_route_is_kept(planner):
    """Route-only tails cannot show a chair that keeps stepping aside: an
    arc ending 0.25 m left heading 25 degrees left still runs into a box
    on the route at the end offset. Its straight tail clears it."""
    box = np.array([(x, y) for x in np.arange(3.0, 3.41, 0.1)
                    for y in np.arange(-0.3, 0.31, 0.1)])
    paths = np.array([[[1.6, 0.25, 0.44]]])
    routed = planner._route_obstacle_paths(paths, 0.1,
                                           dwa_core.OBSTACLE_PREVIEW_M)
    from scipy.spatial import cKDTree
    route_only = cKDTree(box).query(routed[0, :, :2])[0].min()
    either = planner._obstacle_clearance(paths, 0.1, box, cpu_query(box))
    assert route_only < dwa_core.OBSTACLE_FLOOR_M
    assert either[0] >= dwa_core.OBSTACLE_FLOOR_M


def test_the_route_tail_stops_at_the_route_end(planner):
    end = l_route()[-1]
    paths = np.array([[[end[0], end[1] - 0.5, math.pi / 2]]])
    grown = planner._route_obstacle_paths(paths, 0.1,
                                          dwa_core.OBSTACLE_PREVIEW_M)
    assert np.max(grown[0, :, 1]) <= end[1] + 1e-6


def test_route_curvature(planner):
    assert planner.route_curvature((2.0, 0.0), 1.2) == pytest.approx(
        0.0, abs=1e-6)
    on_arc = (7.5 + 2.5 * math.sin(0.4), 2.5 - 2.5 * math.cos(0.4))
    assert planner.route_curvature(on_arc, 1.2) == pytest.approx(
        0.4, abs=0.03)


def test_the_gpu_planner_uses_the_same_rule():
    text = (SCRIPTS / "gpu_dwa_backend.py").read_text(encoding="utf-8")
    assert "self._obstacle_clearance(" in text
    assert "self._obstacle_paths(" not in text

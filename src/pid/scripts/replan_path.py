#!/usr/bin/env python3
"""ROS-independent static-obstacle A* detour generation.

Only stationary objects enter the search. Runtime Safety continues to own
moving/unknown obstacle avoidance and all motion authorization.
Original on-disk routes are never modified.
"""
import heapq
import math

import numpy as np
from scipy.interpolate import splprep, splev

# Initial conservative planning values, NOT validated wheelchair dimensions.
MASK_RESERVE_M = 0.30
OBSTACLE_RADIUS_M = (0.50, 0.60, 0.70)
GRID_MARGIN_M = 8.0
MAX_SEARCH_CELLS = 250000
MAX_EXPANSIONS = 120000
JOIN_OFFSETS_M = (3.0, 5.0, 8.0, 12.0, 16.0, 24.0)
NEIGHBORS = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0),
             (0, 1, 1.0), (-1, -1, 2 ** .5), (-1, 1, 2 ** .5),
             (1, -1, 2 ** .5), (1, 1, 2 ** .5))


def static_boxes(objects):
    """Validate all object geometry; never use moving/unknown objects for planning."""
    if not isinstance(objects, list) or len(objects) > 200:
        raise ValueError("invalid object list")
    boxes = []
    for item in objects:
        if not isinstance(item, dict):
            raise ValueError("invalid object")
        if item.get("motion") != "static" or item.get("band_relation") == "outside":
            continue
        size = item.get("size")
        try:
            x, y, dx, dy = map(float, (item["x"], item["y"], size[0], size[1]))
        except (KeyError, ValueError, TypeError, IndexError) as exc:
            raise ValueError("invalid static object box") from exc
        if not all(math.isfinite(v) for v in (x, y, dx, dy)) or dx <= 0 or dy <= 0:
            raise ValueError("invalid static object dimensions")
        boxes.append((x, y, dx, dy))
    if not boxes:
        raise ValueError("no confirmed static obstacles")
    return boxes


def footprint_free(xy, pose, boxes, radius):
    """Distance from chair-centre samples to every chair-frame obstacle AABB."""
    points = np.asarray(xy, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("invalid path points")
    x, y, theta = map(float, pose)
    c, s = math.cos(theta), math.sin(theta)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    local_x, local_y = c * dx + s * dy, -s * dx + c * dy
    clear = np.ones(len(points), dtype=bool)
    for bx, by, bw, bh in boxes:
        gx = np.maximum(np.abs(local_x - bx) - bw / 2., 0.)
        gy = np.maximum(np.abs(local_y - by) - bh / 2., 0.)
        clear &= gx * gx + gy * gy >= radius * radius
    return clear


def sample_line(a, b, interval):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    count = max(2, int(math.ceil(np.linalg.norm(b - a) / interval)) + 1)
    t = np.linspace(0., 1., count)
    return a[None, :] * (1. - t[:, None]) + b[None, :] * t[:, None]


def checked_path(mask, path, pose, boxes, radius, reserve=MASK_RESERVE_M):
    """Check interpolated curves, not just the original grid cell centres."""
    pts = np.asarray(path, dtype=float)
    if len(pts) < 2 or not np.isfinite(pts).all():
        return False
    sampled = np.concatenate([
        sample_line(a, b, min(mask.resolution * .4, .04))[:-1]
        for a, b in zip(pts[:-1], pts[1:])
    ] + [pts[-1:]], axis=0)
    return (bool(np.all(mask.clearance_many(sampled) >= reserve)) and
            bool(np.all(footprint_free(sampled, pose, boxes, radius))) and
            bool(np.all(mask.paths_are_contained([pts]))))


def cell_xy(mask, row, col):
    return (mask.origin[0] + col * mask.resolution,
            mask.origin[1] + (mask.free.shape[0] - 1 - row) * mask.resolution)


def crop_grid(mask, start, goal, pose, boxes, radius):
    """Limit the search region around this one bypass (not the entire route map)."""
    r, c, valid = mask._cells(np.asarray([start, goal]))
    if not valid.all():
        raise ValueError("start/join outside map")
    extra = int(math.ceil(GRID_MARGIN_M / mask.resolution))
    r0 = max(0, int(min(r)) - extra)
    r1 = min(mask.free.shape[0], int(max(r)) + extra + 1)
    c0 = max(0, int(min(c)) - extra)
    c1 = min(mask.free.shape[1], int(max(c)) + extra + 1)
    if (r1 - r0) * (c1 - c0) > MAX_SEARCH_CELLS:
        raise ValueError("A* crop too large")
    free = (mask.free[r0:r1, c0:c1] &
            (mask.clearance[r0:r1, c0:c1] >= MASK_RESERVE_M))
    rows, cols = np.indices(free.shape)
    x = mask.origin[0] + (cols + c0) * mask.resolution
    y = mask.origin[1] + (mask.free.shape[0] - 1 - (rows + r0)) * mask.resolution
    local = np.column_stack((x.ravel(), y.ravel()))
    free &= footprint_free(local, pose, boxes, radius).reshape(free.shape)
    start_rc = (int(r[0]) - r0, int(c[0]) - c0)
    goal_rc = (int(r[1]) - r0, int(c[1]) - c0)
    return free, start_rc, goal_rc, (r0, c0)


def astar(free, start, goal):
    """8-connected shortest-distance A*, forbidding diagonal corner cutting."""
    h, w = free.shape
    if not free[start] or not free[goal]:
        raise ValueError("start or join blocked")
    def heuristic(p):
        dx, dy = abs(goal[0] - p[0]), abs(goal[1] - p[1])
        return max(dx, dy) + (2 ** .5 - 1.) * min(dx, dy)
    queue = [(heuristic(start), 0., start)]
    distances = {start: 0.}
    parent = {}
    expanded = 0
    while queue:
        _, g, node = heapq.heappop(queue)
        if g > distances.get(node, float("inf")) + 1e-9:
            continue
        if node == goal:
            result = [node]
            while result[-1] != start:
                result.append(parent[result[-1]])
            return result[::-1]
        expanded += 1
        if expanded > MAX_EXPANSIONS:
            raise ValueError("A* expansion budget exceeded")
        for dr, dc, cost in NEIGHBORS:
            rr, cc = node[0] + dr, node[1] + dc
            if not (0 <= rr < h and 0 <= cc < w and free[rr, cc]):
                continue
            if dr and dc and (not free[node[0] + dr, node[1]] or
                              not free[node[0], node[1] + dc]):
                continue
            nxt = (rr, cc)
            ng = g + cost
            if ng + 1e-9 < distances.get(nxt, float("inf")):
                distances[nxt], parent[nxt] = ng, node
                heapq.heappush(queue, (ng + heuristic(nxt), ng, nxt))
    raise ValueError("no A* path")


def visible(mask, free, offset, a, b):
    """Inflated-occupancy line-of-sight including raster corner supercover."""
    segment = sample_line(a, b, mask.resolution / 5.)
    row, col, valid = mask._cells(segment)
    rr, cc = row - offset[0], col - offset[1]
    inbounds = valid & (rr >= 0) & (cc >= 0) & (rr < free.shape[0]) & (cc < free.shape[1])
    if not inbounds.all():
        return False
    if not free[rr, cc].all():
        return False
    # The authoritative mask's supercover rejects diagonal corner-touch cases.
    return bool(mask.segment_is_contained(a, b))


def prune(mask, free, offset, points):
    """Keep a bend only when the segment skipping it is unsafe."""
    answer = [np.asarray(points[0], dtype=float)]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not visible(mask, free, offset, points[i], points[j]):
            j -= 1
        if not visible(mask, free, offset, points[i], points[j]):
            raise ValueError("raster path cannot be safely simplified")
        answer.append(np.asarray(points[j], dtype=float))
        i = j
    return np.asarray(answer)


def resample(points, step=0.1):
    points = np.asarray(points, dtype=float)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.r_[True, lengths > 1e-5]
    points = points[keep]
    if len(points) < 2:
        raise ValueError("short path")
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    station = np.r_[np.arange(0., s[-1], step), s[-1]]
    xy = np.column_stack([np.interp(station, s, points[:, i]) for i in (0, 1)])
    return xy


def smooth(points, s_value):
    """Quintic B-spline when 6+ nonduplicate anchors are available."""
    anchors = resample(points, .45)
    dist = np.r_[0., np.cumsum(np.linalg.norm(np.diff(anchors, axis=0), axis=1))]
    if dist[-1] < .2:
        raise ValueError("short detour")
    u = dist / dist[-1]
    degree = min(5, len(anchors) - 1)
    tck, _ = splprep([anchors[:, 0], anchors[:, 1]], u=u, k=degree, s=s_value)
    dense = np.column_stack(splev(np.linspace(0, 1, max(50, int(dist[-1] / .025))),
                                  tck))
    dense[0], dense[-1] = anchors[0], anchors[-1]
    return resample(dense, .1)


def replan(original_route, mask, pose, station, objects):
    """Return a full in-memory Route metadata dict (original route preserved)."""
    if len(pose) != 3 or not all(math.isfinite(float(p)) for p in pose):
        raise ValueError("invalid chair-centre pose")
    if not math.isfinite(float(station)):
        raise ValueError("invalid route progress")
    boxes = static_boxes(objects)
    start = np.asarray(pose[:2], dtype=float)
    # Try tighter paths first; increased obstacle inflation changes the search.
    for radius in OBSTACLE_RADIUS_M:
        for offset in JOIN_OFFSETS_M:
            join_s = station + offset
            if join_s + .9 >= original_route.length:
                continue
            join = np.asarray([np.interp(join_s, original_route.s,
                                         original_route.xy[:, i]) for i in (0, 1)])
            try:
                free, start_rc, goal_rc, shift = crop_grid(
                    mask, start, join, pose, boxes, radius)
                indices = astar(free, start_rc, goal_rc)
                grid_points = [cell_xy(mask, r + shift[0], c + shift[1])
                               for r, c in indices]
                # Exact pose and original-route join, not snapped cell centres.
                raw = np.vstack([start, grid_points, join])
                raw = prune(mask, free, shift, raw)
                # Align the last tangent to the retained original route.
                after_s = [join_s + delta for delta in (.25, .50, .80)]
                continuation = np.column_stack([
                    np.interp(after_s, original_route.s, original_route.xy[:, i])
                    for i in (0, 1)])
                raw = np.vstack([raw, continuation])
                suffix_start = int(np.searchsorted(original_route.s, after_s[-1], side="right"))
                suffix = original_route.xy[suffix_start:]
                for smoothing in (1.05, .20, .03, 0.):
                    try:
                        detour = smooth(raw, smoothing)
                        if not checked_path(mask, detour, pose, boxes, radius):
                            continue
                        full = np.vstack((detour, suffix))
                        if np.linalg.norm(full[len(detour)] - full[len(detour) - 1]) < 1e-5:
                            full = np.delete(full, len(detour), axis=0)
                        if not checked_path(mask, full, pose, boxes, radius):
                            continue
                        if np.any(np.linalg.norm(np.diff(full, axis=0), axis=1) < 1e-5):
                            continue
                        ds = np.linalg.norm(np.diff(full, axis=0), axis=1)
                        s = np.r_[0., np.cumsum(ds)]
                        yaw = np.unwrap(np.arctan2(
                            np.gradient(full[:, 1], s), np.gradient(full[:, 0], s)))
                        metadata = {
                            "frame": "map",
                            "route_key": original_route.metadata["route_key"],
                            "reference_point": original_route.metadata["reference_point"],
                            "body_frame_profile": original_route.metadata["body_frame_profile"],
                            "chair_centre_in_body_xyz": original_route.metadata["chair_centre_in_body_xyz"],
                            "source": "runtime_static_astar",
                            "count": len(full),
                            "route_step_m": 0.1,
                            "replan_prefix_count": int(len(detour)),
                            "replan_original_join_station_m": float(join_s),
                            "waypoints": [
                                {"x": round(float(p[0]), 5), "y": round(float(p[1]), 5),
                                 "yaw_deg": round(float(math.degrees(a)), 5)}
                                for p, a in zip(full, yaw)]
                        }
                        return metadata
                    except (ValueError, TypeError, FloatingPointError):
                        continue
            except (ValueError, IndexError, TypeError, FloatingPointError):
                continue
    raise ValueError("NO_VALID_STATIC_DETOUR")

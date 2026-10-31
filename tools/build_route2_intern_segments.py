#!/usr/bin/env python3
"""Smooth the route2_intern line once, cut it at D4, and bind six routes.

    python3 tools/build_route2_intern_segments.py            # write
    python3 tools/build_route2_intern_segments.py --check    # verify only

Source: routes/20261028_route2_D5_to_A_waypoints.json, the 10-16 return drive
recentred to the chair. Everything else is derived from it and from the PCD:

1. The line is smoothed. It is a joystick trace, and on the 10-30 A_to_D5 run
   the chair followed its wiggle (+-0.07 m, ~4.4 m wavelength, waypoint
   heading noise p95 9 deg): the DWA heading critic takes the reference
   heading from neighbouring waypoints, so position noise nobody can see on
   the ground becomes a heading reference that swings by +-10 deg. Gaussian
   along arc length, sigma SIGMA_M; the A->J straight gets SIGMA_AJ_M because
   its line also wanders +-0.4 m over 15-20 m there.
2. The J corner is rounded with a wider local sigma. On 10-30 the chair took
   it at 0.8 m/s with w = 0 until 2 m before the corner (R 2.4-3.4 m),
   safety_gate stopped it 3.0 m short of the wall straight ahead, and the
   turn was then made at the 0.35 m/s floor with +-0.5 m overshoot.
3. The mask's north edge at the D4 bend is trimmed to keep the chair centre
   >= KERB_CLEAR_M from the kerb. The 10-28 mask ran over the kerb there and
   the 10-30 run stopped twice 0.40 m and 0.28 m short of it.
4. D4 is the point of the line nearest D4_XY (2026-10-31 operator choice),
   and the line is cut there into six keyed routes, all on the new mask.

No point of the new line comes closer to the PCD's walls and kerbs than
min(KERB_CLEAR_M, the driven line's own clearance there - CLEAR_SLACK_M):
where smoothing would, the line eases back towards where it was driven
(constrain()). The inside of the J corner is a building, so this, not
SIGMA_J_M, is what bounds how far that corner is rounded.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "static_livox_localization" / "scripts"))
sys.path.insert(0, str(ROOT / "tools"))

from reverse_route_assets import (  # noqa: E402
    reverse_stations, reverse_waypoints, write_json)
from route_assets import (  # noqa: E402
    route_content_sha256, sha256, validate_asset_binding)

ROUTES = ROOT / "routes"
SOURCE = ROUTES / "20261028_route2_D5_to_A_waypoints.json"
SOURCE_MASK = ROUTES / "route_2d_map_route2_D5_to_A.pgm"
PCD = ROOT / "maps" / "route2_intern_20261026" / "route2_intern.pcd"
MASK_NAME = "route_2d_map_route2_intern_20261031"
PREFIX = "20261031_route2_"

# Mask raster, identical to the 10-28 mask.
RES, OX, OY = 0.10, -66.9, -108.8

STEP_M = 0.2           # route_step_m, as the source
FINE_M = 0.05          # arc-length grid the smoothing runs on
SIGMA_M = 1.5          # removes ~90 % of the 4.4 m wiggle, keeps bends
SIGMA_AJ_M = 3.0       # the A->J straight
SIGMA_J_M = 3.0        # the J corner
TAPER_M = 4.0          # sigma changes over this much arc, not in a step
# Source-line arc (D5->A direction) of each zone. J route corner (86.8,
# -20.7) and the A->J straight, located by nearest source waypoint.
J_CORNER_XY = (88.3, -21.6)
J_HALF_M = 7.0
AJ_FROM_XY = (74.0, -20.2)     # J end of the straight
AJ_TO_XY = (36.0, -7.5)        # A end, before the cluttered stretch
D4_XY = (105.64, 60.00)

KERB_CLEAR_M = 0.45    # chair centre to kerb/wall, as the 10-28 east trim
TALL_M = (0.25, 1.6)   # above the local ground: kerb tops, walls, posts
D4_WINDOW = (100.0, 113.0, 57.0, 64.0)   # x0, x1, y0, y1 of the trim
MAX_SHIFT_M = 1.2      # anywhere; bounded far tighter outside J in practice
CLEAR_SLACK_M = 0.05  # where the driven line was already inside KERB_CLEAR_M
SELF_M = 0.6          # columns this close to the driven chair centre: see build()

D5_TO_A = "D5_to_A"
KEYS = ("D5_to_A", "A_to_D5", "D5_to_D4", "D4_to_A", "A_to_D4", "D4_to_D5")

POLICY_ADD = (
    "2026-10-31: line smoothed (Gaussian along arc, sigma %.1f m; %.1f m on "
    "the A->J straight; %.1f m at the J corner) from the 20261028 trace; "
    "mask north edge at the D4 bend (x %.0f-%.0f, y %.0f-%.0f) trimmed to "
    "keep the chair centre >= %.2f m from PCD kerb points (%.2f-%.2f m above "
    "the local ground), after the 10-30 stop_watchdog stops at the kerb"
    % (SIGMA_M, SIGMA_AJ_M, SIGMA_J_M, D4_WINDOW[0], D4_WINDOW[1],
       D4_WINDOW[2], D4_WINDOW[3], KERB_CLEAR_M, TALL_M[0], TALL_M[1]))


# ------------------------------------------------------------------ geometry
def arc_length(xy):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]


def resample(xy, step):
    s = arc_length(xy)
    t = np.linspace(0.0, s[-1], int(round(s[-1] / step)) + 1)
    return np.column_stack([np.interp(t, s, xy[:, 0]),
                            np.interp(t, s, xy[:, 1])]), t


def headings(xy):
    t = np.gradient(xy, axis=0)
    return np.arctan2(t[:, 1], t[:, 0])


def radius_profile(xy, span_m=1.0):
    """Turning radius over +-span_m chords, the scale the planner sees."""
    s = arc_length(xy)
    k = max(1, int(round(span_m / np.median(np.diff(s)))))
    h = np.unwrap(headings(xy))
    turn = np.abs(h[2 * k:] - h[:-2 * k])
    r = np.full(len(xy), np.inf)
    r[k:-k] = (s[2 * k:] - s[:-2 * k]) / np.maximum(turn, 1e-9)
    return r


def smooth(xy, sigma):
    """Gaussian smoothing along arc length with a per-sample sigma.

    Odd reflection about both ends keeps each endpoint and its direction.
    """
    n = len(xy)
    pad = int(math.ceil(3 * sigma.max() / FINE_M)) + 1
    head = 2 * xy[0] - xy[pad:0:-1]
    tail = 2 * xy[-1] - xy[-2:-pad - 2:-1]
    ext = np.vstack([head, xy, tail])
    out = np.empty_like(xy)
    for i in range(n):
        half = int(math.ceil(3 * sigma[i] / FINE_M))
        j = np.arange(i + pad - half, i + pad + half + 1)
        w = np.exp(-0.5 * ((j - i - pad) * FINE_M / sigma[i]) ** 2)
        out[i] = (w[:, None] * ext[j]).sum(0) / w.sum()
    return out


def sigma_profile(s, zones):
    """SIGMA_M, raised to each zone's sigma over [a, b] with a TAPER_M ramp."""
    sigma = np.full(len(s), SIGMA_M)
    for a, b, value in zones:
        ramp = np.clip(np.minimum(s - (a - TAPER_M), (b + TAPER_M) - s)
                       / TAPER_M, 0.0, 1.0)
        sigma = np.maximum(sigma, SIGMA_M + (value - SIGMA_M) * ramp)
    return sigma


# ------------------------------------------------------------------ PCD
def load_pcd(path):
    raw = path.read_bytes()
    start = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    header = raw[:start].decode("ascii")
    count = int([line for line in header.splitlines()
                 if line.startswith("POINTS")][0].split()[1])
    # POINTS x 16 bytes only: the file carries 3,904 bytes of zero padding.
    cloud = np.frombuffer(raw, dtype=np.float32, count=count * 4,
                          offset=start).reshape(-1, 4)[:, :3]
    return cloud[np.isfinite(cloud).all(axis=1)].astype(float)


def tall_points(cloud, box, min_neighbours=3):
    """Kerb tops and walls inside box: TALL_M above a 1 m-cell ground.

    Ground is the cell's 10th-percentile z, the estimate the D4 kerb was
    found with; isolated returns (fewer than min_neighbours within 0.2 m)
    are dropped as noise.
    """
    x0, x1, y0, y1 = box
    p = cloud[(cloud[:, 0] > x0 - 2) & (cloud[:, 0] < x1 + 2)
              & (cloud[:, 1] > y0 - 2) & (cloud[:, 1] < y1 + 2)]
    key = np.floor(p[:, 0]).astype(np.int64) * 100000 + \
        np.floor(p[:, 1]).astype(np.int64)
    order = np.argsort(key)
    key_sorted = key[order]
    edges = np.r_[0, np.nonzero(np.diff(key_sorted))[0] + 1, len(key)]
    ground = np.empty(len(p))
    for a, b in zip(edges[:-1], edges[1:]):
        idx = order[a:b]
        ground[idx] = np.percentile(p[idx, 2], 10)
    h = p[:, 2] - ground
    t = p[(h > TALL_M[0]) & (h < TALL_M[1])][:, :2]
    if len(t) == 0:
        return t
    near = cKDTree(t).query_ball_point(t, 0.2, return_length=True)
    return t[near > min_neighbours]


def wall_columns(cloud, cell=0.2, min_points=10, min_cells=4):
    """The 10-28 mask's obstacle columns: 0.2 m cells holding at least
    min_points returns TALL_M above the 3x3 lowest point, in clusters of at
    least min_cells. Robust against single returns; misses a thin kerb,
    which is why the D4 kerb is taken from tall_points as well.
    """
    lo = cloud[:, :2].min(0)
    c = np.floor((cloud[:, :2] - lo) / cell).astype(int)
    w, h = c.max(0) + 1
    flat = c[:, 1] * w + c[:, 0]
    zmin = np.full(w * h, np.inf)
    np.minimum.at(zmin, flat, cloud[:, 2])
    zloc = ndimage.minimum_filter(
        np.where(np.isfinite(zmin), zmin, 1e9).reshape(h, w), size=3)
    rel = cloud[:, 2] - zloc.ravel()[flat]
    band = (rel > TALL_M[0]) & (rel < TALL_M[1])
    obs = np.bincount(flat[band], minlength=w * h).reshape(h, w) >= min_points
    lab, n = ndimage.label(obs)
    sizes = ndimage.sum(obs, lab, range(1, n + 1))
    obs &= np.isin(lab, 1 + np.nonzero(sizes >= min_cells)[0])
    yy, xx = np.nonzero(obs)
    return np.column_stack([lo[0] + (xx + 0.5) * cell,
                            lo[1] + (yy + 0.5) * cell])


def constrain(source, smoothed, wall_tree, need, spread_m=2.0):
    """Blend from smoothed back towards the driven line where it is tight.

    alpha is 1 (smoothed) everywhere it can be; wherever the smoothed line
    comes closer to a wall than `need`, the cap there is halved and alpha is
    re-drawn as a smooth curve under all the caps, so the line eases back to
    where it was driven over a few metres instead of kinking. At alpha 0 the
    line IS the driven one, which meets `need` by construction.
    """
    cap = np.ones(len(source))
    size = int(round(2 * spread_m / FINE_M)) + 1
    for _ in range(40):
        alpha = ndimage.gaussian_filter1d(
            ndimage.minimum_filter1d(cap, size=size, mode="nearest"),
            sigma=spread_m / 2 / FINE_M, mode="nearest")
        alpha = np.minimum(alpha, cap)
        line = source + alpha[:, None] * (smoothed - source)
        bad = wall_tree.query(line)[0] < need
        if not bad.any():
            return line, alpha
        cap[bad] *= 0.5
    raise SystemExit("could not keep the line clear of the walls")


# ------------------------------------------------------------------ mask
def load_mask(path):
    return np.array(Image.open(path)) > 250


def cells(xy):
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    h = MASK_SHAPE[0]
    return (np.rint(h - 1 - (xy[:, 1] - OY) / RES).astype(int),
            np.rint((xy[:, 0] - OX) / RES).astype(int))


def cell_centres(rows, cols):
    h = MASK_SHAPE[0]
    return np.column_stack([OX + cols * RES, OY + (h - 1 - rows) * RES])


def trim_d4(free, line, kerb):
    """Clear free cells on the kerb's side of the line within the window.

    A cell goes when it is within KERB_CLEAR_M of a kerb point, or when it
    lies beyond the kerb (further from the line than the kerb is). The
    route's own connected component is then kept, so nothing is left
    stranded past the kerb.
    """
    x0, x1, y0, y1 = D4_WINDOW
    rows, cols = np.nonzero(free)
    xy = cell_centres(rows, cols)
    inside = (xy[:, 0] >= x0) & (xy[:, 0] <= x1) & \
        (xy[:, 1] >= y0) & (xy[:, 1] <= y1)
    rows, cols, xy = rows[inside], cols[inside], xy[inside]
    tree = cKDTree(line)
    tang = np.gradient(line, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    normal = np.column_stack([-tang[:, 1], tang[:, 0]])

    def lateral(points):
        _, i = tree.query(points)
        return np.sum((points - line[i]) * normal[i], axis=1), i

    kerb_lat, kerb_i = lateral(kerb)
    side = np.sign(np.median(kerb_lat))
    kerb = kerb[np.sign(kerb_lat) == side]
    kerb_lat, kerb_i = kerb_lat[np.sign(kerb_lat) == side], \
        kerb_i[np.sign(kerb_lat) == side]
    # nearest kerb offset per route index, for "beyond the kerb"
    reach = np.full(len(line), np.inf)
    np.minimum.at(reach, kerb_i, np.abs(kerb_lat))
    reach = ndimage.minimum_filter1d(reach, size=11)
    lat, i = lateral(xy)
    on_side = np.sign(lat) == side
    close = cKDTree(kerb).query(xy)[0] < KERB_CLEAR_M
    beyond = np.abs(lat) > reach[i]
    drop = on_side & (close | beyond)
    out = free.copy()
    out[rows[drop], cols[drop]] = False
    lab, _ = ndimage.label(out)
    r, c = cells(line)
    keep = np.unique(lab[r, c])
    out = np.isin(lab, keep[keep > 0])
    return out, int(drop.sum())


# ------------------------------------------------------------------ band
def band_stations(xy, free, step=0.5):
    """The 10-28 band builder: walk each station's normal to the mask edge."""
    s = arc_length(xy)
    st = np.arange(0.0, s[-1] + 1e-9, step)
    sx, sy = np.interp(st, s, xy[:, 0]), np.interp(st, s, xy[:, 1])
    h, w = free.shape

    def inside(x, y):
        r, c = cells([(x, y)])
        return 0 <= r[0] < h and 0 <= c[0] < w and free[r[0], c[0]]

    stations = []
    for i, (x, y) in enumerate(zip(sx, sy)):
        a = max(st[i] - 1.0, 0.0)
        b = min(st[i] + 1.0, s[-1])
        hd = math.atan2(np.interp(b, s, xy[:, 1]) - np.interp(a, s, xy[:, 1]),
                        np.interp(b, s, xy[:, 0]) - np.interp(a, s, xy[:, 0]))
        nx, ny = -math.sin(hd), math.cos(hd)

        def walk(sign):
            k = 0.0
            while k < 8.0 and inside(x + sign * nx * (k + 0.05),
                                     y + sign * ny * (k + 0.05)):
                k += 0.05
            return round(k, 2)

        stations.append(dict(
            x=round(float(x), 3), y=round(float(y), 3),
            heading_deg=round(math.degrees(hd), 3),
            left_m=walk(1), right_m=walk(-1),
            left_drop_m=0.0, right_drop_m=0.0,
            left_kind="candidate_mask_clip", right_kind="candidate_mask_clip",
            left_rise_m=0.0, right_rise_m=0.0))
    return stations


# ------------------------------------------------------------------ assets
def waypoint_list(xy, z):
    hd = np.degrees(headings(xy))
    return [dict(x=round(float(x), 4), y=round(float(y), 4),
                 z=round(float(zz), 3), yaw_deg=round(float(h), 1))
            for (x, y), zz, h in zip(xy, z, hd)]


def bind(route, band, mask_png, mask_yaml, prefix, id_tag):
    """Write <prefix>_{waypoints,safety_band}.json bound to the mask."""
    mask_sha, yaml_sha = sha256(mask_png.with_suffix(".pgm")), sha256(mask_yaml)
    route = dict(route)
    route.pop("asset_binding", None)
    route["count"] = len(route["waypoints"])
    route["operator_target_waypoint_index"] = route["count"] - 1
    last = route["waypoints"][-1]
    route["operator_target_xy_m"] = [last["x"], last["y"]]
    route["path_length_m"] = round(float(arc_length(np.array(
        [[p["x"], p["y"]] for p in route["waypoints"]]))[-1]), 3)
    content = route_content_sha256(route)
    route_id = "trace20261016:%s:%s:%s" % (id_tag, content[:12], mask_sha[:12])
    band = dict(band)
    band["route_id"] = route_id
    band["drivable_mask_sha256"] = mask_sha
    band["drivable_mask_yaml_sha256"] = yaml_sha
    route_path = prefix.with_name(prefix.name + "_waypoints.json")
    band_path = prefix.with_name(prefix.name + "_safety_band.json")
    write_json(band_path, band)
    route["asset_binding"] = {
        "route_id": route_id,
        "drivable_mask_sha256": mask_sha,
        "drivable_mask_yaml_sha256": yaml_sha,
        "safety_band_sha256": sha256(band_path),
        "route_content_sha256": content,
    }
    write_json(route_path, route)
    validate_asset_binding(route_path, band_path, mask_yaml)
    return route_path, band_path


def reversed_pair(route, band, name_route, name_band):
    route = dict(route)
    route["waypoints"] = reverse_waypoints(route["waypoints"])
    route["reversed_from"] = name_route
    band = dict(band)
    band["stations"] = reverse_stations(band["stations"])
    band["reversed_from"] = name_band
    return route, band


# ------------------------------------------------------------------ main
def build(args):
    global MASK_SHAPE
    source = json.loads(SOURCE.read_text(encoding="utf-8"))
    src_xy = np.array([[p["x"], p["y"]] for p in source["waypoints"]])
    src_z = np.array([p["z"] for p in source["waypoints"]])
    src_s = arc_length(src_xy)

    fine, fine_s = resample(src_xy, FINE_M)

    def arc_of(xy):
        return float(fine_s[cKDTree(fine).query(xy)[1]])

    j = arc_of(J_CORNER_XY)
    aj = sorted((arc_of(AJ_FROM_XY), arc_of(AJ_TO_XY)))
    sigma = sigma_profile(fine_s, [(aj[0], aj[1], SIGMA_AJ_M),
                                   (j - J_HALF_M, j + J_HALF_M, SIGMA_J_M)])
    smoothed = smooth(fine, sigma)

    # ------------------------------------------------ PCD clearance
    cloud = load_pcd(PCD)
    x0, x1, y0, y1 = D4_WINDOW
    kerb = tall_points(cloud, D4_WINDOW)
    kerb = kerb[(kerb[:, 0] >= x0 - 1) & (kerb[:, 0] <= x1 + 1)
                & (kerb[:, 1] >= y0 - 1) & (kerb[:, 1] <= y1 + 1)]
    walls = np.vstack([wall_columns(cloud), kerb])
    # Columns within SELF_M of the driven chair centre are not walls: within
    # 1 m of the line they spread evenly across it, right up to the line
    # itself, where the chair demonstrably drove - rider returns and ground
    # steps the 3x3 floor misreads. The 10-28 mask dropped them out to 0.8 m.
    walls = walls[cKDTree(src_xy).query(walls)[0] > SELF_M]
    wall_tree = cKDTree(walls)
    clear_src = wall_tree.query(fine)[0]
    # Where the driven line was already closer than KERB_CLEAR_M, the new
    # one may not give up more than CLEAR_SLACK_M of it.
    need = np.minimum(KERB_CLEAR_M, clear_src - CLEAR_SLACK_M)
    line, alpha = constrain(fine, smoothed, wall_tree, need)
    xy, s = resample(line, STEP_M)
    z = np.interp(s / s[-1] * src_s[-1], src_s, src_z)
    clear_new = wall_tree.query(xy)[0]
    shift = cKDTree(src_xy).query(xy)[0]
    if shift.max() > MAX_SHIFT_M:
        raise SystemExit("smoothing moved the line %.2f m" % shift.max())

    # ------------------------------------------------ mask
    MASK_SHAPE = np.array(Image.open(SOURCE_MASK)).shape
    free = load_mask(SOURCE_MASK)
    free, trimmed = trim_d4(free, xy, kerb)
    r, c = cells(xy)
    if not free[r, c].all():
        raise SystemExit("%d waypoints outside the mask"
                         % int((~free[r, c]).sum()))
    if ndimage.label(free)[1] != 1:
        raise SystemExit("mask is not one connected region")

    # ------------------------------------------------ D4 and the cut
    d4 = int(cKDTree(xy).query(D4_XY)[1])
    d4_miss = float(np.linalg.norm(xy[d4] - D4_XY))

    report = {
        "waypoints": len(xy), "length_m": round(float(s[-1]), 2),
        "d4_index_D5_to_A": d4, "d4_index_A_to_D5": len(xy) - 1 - d4,
        "d4_xy": [round(float(v), 3) for v in xy[d4]],
        "d4_miss_m": round(d4_miss, 3),
        "shift_max_m": round(float(shift.max()), 3),
        "shift_p95_m": round(float(np.percentile(shift, 95)), 3),
        "clearance_min_m": round(float(clear_new.min()), 3),
        "held_back_share": round(float(np.mean(alpha < 0.99)), 3),
        "mask_cells_trimmed": trimmed,
        "j_min_radius_src_m": round(float(radius_profile(src_xy)[
            np.abs(src_s - j) < J_HALF_M].min()), 2),
        "j_min_radius_new_m": round(float(radius_profile(xy)[
            np.abs(s - s[int(cKDTree(xy).query(J_CORNER_XY)[1])])
            < J_HALF_M].min()), 2),
        "heading_noise_p95_src_deg": None, "heading_noise_p95_new_deg": None,
    }

    def heading_noise(line):
        h = np.unwrap(headings(line))
        ref = ndimage.uniform_filter1d(h, 31, mode="nearest")
        return round(float(np.percentile(np.degrees(np.abs(h - ref)), 95)), 2)
    report["heading_noise_p95_src_deg"] = heading_noise(src_xy)
    report["heading_noise_p95_new_deg"] = heading_noise(xy)
    # Smoothing rounds the D4 bend inwards, away from the kerb, so the cut
    # point moves south of the chosen one; more than this is not that.
    if d4_miss > 0.5:
        raise SystemExit("D4 moved %.2f m off the chosen point" % d4_miss)
    if args.check:
        print(json.dumps(report, indent=1))
        return report

    # ------------------------------------------------ write the mask
    mask_png = ROUTES / (MASK_NAME + ".png")
    mask_yaml = ROUTES / (MASK_NAME + ".yaml")
    img = np.where(free, 254, 205).astype(np.uint8)
    Image.fromarray(img, mode="L").save(ROUTES / (MASK_NAME + ".pgm"))
    Image.fromarray(img, mode="L").save(mask_png)
    with open(mask_yaml, "w", encoding="utf-8", newline="\n") as sink:
        sink.write("image: %s.pgm\nmode: trinary\nresolution: 0.1\n"
                   "origin: [%.1f, %.1f, 0.0]\nnegate: 0\n"
                   "occupied_thresh: 0.65\nfree_thresh: 0.196\n"
                   % (MASK_NAME, OX, OY))

    old_band = json.loads((ROUTES / "20261028_route2_D5_to_A_safety_band.json")
                          .read_text(encoding="utf-8"))
    corridor = dict(old_band["corridor"])
    corridor["source"] = MASK_NAME + ".yaml"
    corridor["policy"] = corridor["policy"] + "; " + POLICY_ADD

    base = {k: v for k, v in source.items() if k not in (
        "waypoints", "asset_binding", "local_corner_rounding",
        "operator_lateral_shift")}
    base["source"] = ("20261028_route2_D5_to_A_waypoints.json (10-16 return "
                      "drive, chair centre), smoothed 2026-10-31")
    base["route_step_m"] = STEP_M
    base["smoothing"] = {
        "sigma_m": SIGMA_M, "aj_straight_sigma_m": SIGMA_AJ_M,
        "j_corner_sigma_m": SIGMA_J_M, "j_corner_xy": list(J_CORNER_XY),
        "j_min_radius_m": report["j_min_radius_new_m"],
        "max_shift_from_source_m": report["shift_max_m"],
        "source_corrections_kept": {
            "local_corner_rounding": source.get("local_corner_rounding"),
            "operator_lateral_shift": source.get("operator_lateral_shift"),
        },
        "note": "source_corrections_kept indices refer to the 20261028 "
                "source line, not to this one",
    }
    base["d4_cut"] = {"xy": report["d4_xy"], "chosen_xy": list(D4_XY),
                      "index_D5_to_A": d4,
                      "index_A_to_D5": len(xy) - 1 - d4}

    def stem(key):
        return ROUTES / (PREFIX + key)

    def forward(key, a, b):
        part = xy[a:b + 1]
        route = dict(base)
        route["waypoints"] = waypoint_list(part, z[a:b + 1])
        route["cut_from_D5_to_A_wp"] = [a, b]
        band = dict(frame="map", station_spacing_m=0.5,
                    stations=band_stations(part, free),
                    corridor=dict(corridor),
                    physical_edge_semantics=old_band[
                        "physical_edge_semantics"])
        band["corridor"]["stations_covered"] = len(band["stations"])
        band["corridor"]["stations_total"] = len(band["stations"])
        return route, band

    written = []
    last = len(xy) - 1
    for key, rev_key, a, b in ((D5_TO_A, "A_to_D5", 0, last),
                               ("D5_to_D4", "D4_to_D5", 0, d4),
                               ("D4_to_A", "A_to_D4", d4, last)):
        route, band = forward(key, a, b)
        rp, bp = bind(route, band, mask_png, mask_yaml, stem(key),
                      "smooth20261031")
        written += [rp, bp]
        rroute, rband = reversed_pair(
            json.loads(rp.read_text(encoding="utf-8")),
            json.loads(bp.read_text(encoding="utf-8")), rp.name, bp.name)
        rp2, bp2 = bind(rroute, rband, mask_png, mask_yaml, stem(rev_key),
                        "smooth20261031:reversed")
        written += [rp2, bp2]
    print(json.dumps(report, indent=1))
    for path in written:
        print("wrote", path.relative_to(ROOT))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--check", action="store_true",
                        help="compute and verify, write nothing")
    build(parser.parse_args())


if __name__ == "__main__":
    main()

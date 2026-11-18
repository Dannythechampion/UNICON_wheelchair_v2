#!/usr/bin/env python3
"""Old (straight strip) vs new (RouteFrame) bypass decisions, from a bag.

Usage:
  route_frame_replay.py <blackbox.bag> <person_bypass.bag> <route key> \\
      <out.csv> [t0 t1]

Read-only on the bags. Uses this checkout's cluster_guard (REPO env var or
the repo this file sits in). For every /perception/objects_summary with a
pose it records:
  old: the straight 0.45 m strip threat; GO_ROUND = parked, not a person,
       nearer than PLAN_AHEAD_M (what armed STATIC_OBJECT_BYPASS)
  new: waypoint_follower.cluster_threat's ordinary-object logic - the
       RouteFrame threat, replaced by the straight 0.5 m gate-zone threat
       when that is nearer and inside stop radius + 0.6 m (then marked
       off-route unless the same track is on the route); GO_ROUND only for
       an on-route threat
  the logged permit, and how many objects carry a broken profile slice.
Chair centre = /fast_lio_icp/pose + R(yaw)(-0.5, -0.2). Speed for the stop
radius comes from successive poses (0.9 + v*1.4 + v^2, as the logger).
"""
import bisect
import csv
import json
import math
import os
import sys
import time
from collections import Counter

import numpy as np
import rosbag

REPO = os.environ.get("REPO") or os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "static_livox_localization",
                                "scripts"))
import cluster_guard as cg  # noqa: E402

HALF_WIDTH, PLAN_AHEAD, GATE_HALF, GATE_EXTRA = 0.45, 8.0, 0.5, 0.6
CC = (-0.5, -0.2)

bb, pb, key, out = sys.argv[1:5]
t0 = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
t1 = float(sys.argv[6]) if len(sys.argv) > 6 else 1e12
route = json.load(open(os.path.join(
    REPO, "routes", "route2_manual_20261031", key + "_waypoints.json")))
wps = np.array([[w["x"], w["y"]] for w in route["waypoints"]], float)
arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(wps, axis=0),
                                                      axis=1))])

poses, ptimes, summaries = [], [], []
with rosbag.Bag(bb) as bag:
    for topic, msg, t in bag.read_messages(
            topics=["/fast_lio_icp/pose", "/perception/objects_summary"]):
        ts = t.to_sec()
        if not t0 - 1 <= ts <= t1 + 1:
            continue
        if topic == "/fast_lio_icp/pose":
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            poses.append((p.x + c * CC[0] - s * CC[1],
                          p.y + s * CC[0] + c * CC[1], yaw))
            ptimes.append(ts)
        else:
            summaries.append((ts, msg.data))
permits, mtimes = [], []
with rosbag.Bag(pb) as bag:
    for _, msg, t in bag.read_messages(topics=["/person_bypass/permit"]):
        ts = t.to_sec()
        if t0 - 1 <= ts <= t1 + 1:
            try:
                d = json.loads(msg.data)
            except ValueError:
                continue
            permits.append(d.get("reason", "") if d.get("active") else "")
            mtimes.append(ts)


def near(times, values, t, tol):
    i = bisect.bisect_left(times, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(times) and abs(times[j] - t) <= tol and (
                best is None or abs(times[j] - t) < abs(times[best] - t)):
            best = j
    return None if best is None else best


def go_round(th):
    return th is not None and th.parked and not th.is_person and \
        th.distance_m < PLAN_AHEAD and getattr(th, "on_route", True)


def region(x, y):
    return "J" if 80.0 <= x <= 92.0 and -25.0 <= y <= -8.0 else "other"


rows, nearest = [], None
for ts, data in summaries:
    if not t0 <= ts <= t1:
        continue
    j = near(ptimes, poses, ts, 0.15)
    if j is None:
        continue
    px, py, yaw = poses[j]
    k = max(j - 10, 0)
    dt = ptimes[j] - ptimes[k]
    speed = 0.0 if dt <= 0 else math.hypot(px - poses[k][0],
                                           py - poses[k][1]) / dt
    try:
        summary = cg.parse_summary(data)
    except ValueError:
        continue
    d = np.linalg.norm(wps - (px, py), axis=1)
    if nearest is None or d[nearest] > 3.0:
        nearest = int(np.argmin(d))
    else:
        lo, hi = max(nearest - 5, 0), min(nearest + 60, len(wps))
        nearest = int(lo + np.argmin(d[lo:hi]))
    frame = cg.RouteFrame.build(wps, arc, (px, py), yaw, nearest, PLAN_AHEAD)
    old = cg.nearest_threat(summary, HALF_WIDTH)
    every = cg.matching_threats(summary, HALF_WIDTH, route_frame=frame)
    new = every[0] if every else None
    zone = 0.9 + speed * 1.4 + speed * speed + GATE_EXTRA
    gate = cg.nearest_threat(summary, GATE_HALF)
    if gate is not None and gate.distance_m < zone and (
            new is None or gate.distance_m < new.distance_m):
        ids = {t.track_id for t in every if t.track_id is not None}
        gate.on_route = gate.track_id is not None and gate.track_id in ids
        new = gate
    broken = sum(1 for item in summary.objects
                 if cg.object_box(item) is not None and
                 cg.object_chair_points(item) is cg.BROKEN)
    m = near(mtimes, permits, ts, 0.15)
    logged = "" if m is None else permits[m]

    def desc(th):
        return ("", "", "", "") if th is None else (
            th.track_id, round(th.distance_m, 2), th.label,
            int(getattr(th, "on_route", True)))
    rows.append([round(ts, 2), time.strftime("%H:%M:%S", time.localtime(ts)),
                 round(px, 2), round(py, 2), nearest, region(px, py),
                 logged, int(go_round(old)), *desc(old)[:3],
                 int(go_round(new)), *desc(new), broken])

with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "clock", "x_cc", "y_cc", "wp", "region", "logged_permit",
                "old_goround", "o_track", "o_dist", "o_label",
                "new_goround", "n_track", "n_dist", "n_label", "n_on_route",
                "broken_objs"])
    w.writerows(rows)

for reg in ("J", "other", None):
    sel = [r for r in rows if reg is None or r[5] == reg]
    if sel:
        print("%-5s frames %5d  logged STATIC_OBJECT %4d  old GO_ROUND %4d  "
              "new GO_ROUND %4d  new off-route gate-zone %4d" % (
                  reg or "all", len(sel),
                  sum(1 for r in sel if r[6].startswith("STATIC_OBJECT")),
                  sum(r[7] for r in sel), sum(r[11] for r in sel),
                  sum(1 for r in sel if r[15] == 0)))
first_old = next((r for r in rows if r[5] == "J" and r[7]), None)
first_new = next((r for r in rows if r[5] == "J" and r[11]), None)
print("first J GO_ROUND old:", first_old and first_old[1:5] + first_old[8:11])
print("first J GO_ROUND new:", first_new and first_new[1:5] + first_new[12:15])
print("new GO_ROUND tracks (track, label, region):",
      Counter((r[12], r[14], r[5]) for r in rows if r[11]).most_common(6))
print("frames with broken profile slices:", sum(1 for r in rows if r[16]))

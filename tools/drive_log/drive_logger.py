#!/usr/bin/env python3
"""Read-only drive logger (2026-11-07). Subscribes only; publishes nothing.

Writes to ~/drive_logs/<YYYYmmdd_HHMMSS>/:
  drive.csv     10 Hz: pose (lidar), route error at the CHAIR CENTRE, every cmd_vel stage, wheel and IMU yaw
                rate, pitch/roll, status, permit, gate, semantic threat, and the
                straight-corridor threat the follower itself would pick, and
                (since 2026-11-08) the ROUTE-based threat of the revised
                checklist (route_threat.py next to this file): rt_* = on-route
                threat (distance along the route, route lateral d), rt_goround
                = it would arm a static bypass, gz_* = straight 0.5 m strip
                inside the stop radius + 0.6 m (gate-zone supplement, never a
                bypass), broken_objs = objects with a broken profile slice.
                (since 2026-11-09) sm_* = the DWA follower's lag compensation
                from /dwa_follower/smith_state: pose age, how far and how much
                yaw it led the plan, the base model vs the wheels, the command
                the base was holding. Every cycle of it is also in the bag.
  smith.jsonl   (since 2026-11-09) every /dwa_follower/smith_state message.
  objects.jsonl every objects_summary: each object with chair-frame and
                map-frame position, class, motion, band fraction.
  events.log    human-readable: status changes, permit on/off, gate blocks,
                mode changes, hard turns (|cmd w| >= 0.4, |gz| >= 0.45).
Stop with SIGINT/SIGTERM. Started and stopped by tools/drive_log.sh, which
tools/go.sh and tools/stop.sh call (since 2026-11-09).
"""
import json, math, os, signal, sys, time
import numpy as np
import rospy, rosparam
from std_msgs.msg import String, Int16, Int16MultiArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

HERE = os.path.dirname(os.path.abspath(__file__))
# In the repo (tools/drive_log/) the follower's own scripts are two levels up,
# so the logger judges threats with the code that is driving. A copy run from
# ~ falls back to whatever deploy is current.
REPO = os.path.dirname(os.path.dirname(HERE))
if not os.path.isdir(os.path.join(REPO, "src/static_livox_localization/scripts")):
    REPO = os.path.join(os.path.realpath(os.path.expanduser("~/wheelchair_deploys/current")), "source-linux")
SCRIPTS = os.path.join(REPO, "src/static_livox_localization/scripts")
sys.path.insert(0, SCRIPTS)
# waypoint_follower.CORRIDOR_HALF_WIDTH (not imported: that module starts a node)
CORRIDOR_HALF_WIDTH = 0.45
try:
    import cluster_guard as CG
except Exception as exc:  # keep logging even if the import fails
    CG = None
    print("cluster_guard import failed:", exc)
sys.path.insert(1, os.path.dirname(os.path.abspath(__file__)))
try:
    import route_threat as RT
except Exception as exc:
    RT = None
    print("route_threat import failed:", exc)
sys.path.insert(0, os.path.expanduser("~/catkin_ws/src/base_model/src"))
try:
    import wheel_cmd_tmp as WCT
    wheel_mps = WCT.wheel_speed_mps
except Exception:
    def wheel_mps(direction, magnitude):
        s = (float(magnitude) - 33) / 10.0 / 3.6
        return s if int(direction) == 67 else (-s if int(direction) == 87 else 0.0)

stamp = time.strftime("%Y%m%d_%H%M%S")
OUT = os.path.join(os.path.expanduser(os.environ.get("DRIVE_LOG_ROOT", "~/drive_logs")), stamp) + "/"
os.makedirs(OUT, exist_ok=True)

def route_paths():
    route = None
    for name in ("/waypoint_follower/route", "/waypoint_follower/route_file"):
        try:
            route = rospy.get_param(name); break
        except KeyError:
            pass
    if not route:
        route = os.path.join(REPO, "routes/route2_manual_20261031/A_to_D4_waypoints.json")
    band = route.replace("_waypoints.json", "_safety_band.json")
    return route, band

S = {}            # latest values
ROUTE = None      # route_threat.Route, set in main()
CC = [-0.5, -0.2, 0.0]
EV = open(OUT + "events.log", "w", buffering=1)
def ev(msg):
    line = "%s %s" % (time.strftime("%H:%M:%S") + ".%d" % int((time.time() % 1) * 10), msg)
    EV.write(line + "\n")

def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
def rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch, yaw_of(q)

def on_pose(m):
    p = m.pose.pose
    _, pitch, yaw = rpy(p.orientation)
    S["pose"] = (p.position.x, p.position.y, yaw, pitch)

def mk_twist(key):
    def cb(m): S[key] = (m.linear.x, m.angular.z)
    return cb

def on_status(m):
    kind = m.data.split(" ")[0]
    if kind != S.get("status_kind"):
        ev("STATUS %s  @%s" % (m.data, fmt_pose()))
    S["status"] = m.data; S["status_kind"] = kind

def on_permit(m):
    try: j = json.loads(m.data)
    except Exception: return
    on = bool(j.get("active")) and str(j.get("reason", "")).startswith(("STATIC_OBJECT", "PERSON"))
    key = (on, j.get("reason"), j.get("track_id"))
    if key != S.get("permit_key"):
        if on:
            ty = j.get("target_y_m")
            side = "" if ty is None else ("commit RIGHT (left turns vetoed)" if ty > 0 else "commit LEFT (right turns vetoed)")
            ev("PERMIT ON  %s track#%s target(%.2f,%+.2f) %s  @%s" % (
                j.get("reason"), j.get("track_id"), j.get("target_x_m") or 0, ty or 0, side, fmt_pose()))
        elif S.get("permit_key", (False,))[0]:
            ev("PERMIT OFF %s  @%s" % (j.get("reason"), fmt_pose()))
    S["permit_key"] = key; S["permit"] = j

def on_gate(m):
    try: j = json.loads(m.data)
    except Exception: return
    b = bool(j.get("blocked"))
    if b and not S.get("gate_blocked"):
        ev("GATE BLOCK %s zone_nearest=%s req v%.2f w%+.2f  @%s" % (
            j.get("reason"), j.get("zone_nearest_m"), j.get("requested_v") or 0, j.get("requested_w") or 0, fmt_pose()))
    S["gate_blocked"] = b; S["gate"] = j

def on_sem(m):
    try: S["sem"] = json.loads(m.data)
    except Exception: pass

def on_tip(m): S["tip"] = m.data

def on_wheel(m):
    d = m.data
    if len(d) >= 6:
        S["wheel"] = (int(d[1]), wheel_mps(d[2], d[3]), wheel_mps(d[4], d[5]))
        if S.get("mode") != int(d[1]):
            ev("BASE MODE %d (%s)  @%s" % (d[1], {65: "AUTO", 77: "MANUAL/STOP"}.get(int(d[1]), "?"), fmt_pose()))
        S["mode"] = int(d[1])

def on_vn(m):
    r, p, _ = rpy(m.orientation)
    S["vn"] = (m.angular_velocity.z, r, p)

def on_livox(m):
    S["lgz"] = m.angular_velocity.z

OBJ = open(OUT + "objects.jsonl", "w", buffering=1)
class _Summary:  # for cluster_guard
    pass
def on_objects(m):
    try: g = json.loads(m.data)
    except Exception: return
    if g.get("frame") != "chair_centre": return
    S["objs_raw"] = m.data
    pose = S.get("pose")
    out = []
    for o in g.get("objects", []):
        try:
            ox, oy = float(o["x"]), float(o["y"])
        except Exception:
            continue
        if math.hypot(ox, oy) > 10: continue
        rec = {k: o.get(k) for k in ("id", "class", "raw_class", "motion", "speed_mps", "size", "band_relation", "band_inside_fraction", "points", "age_s")}
        rec["x"], rec["y"] = round(ox, 2), round(oy, 2)
        if pose:
            x, y, yaw, _ = pose; c, s = math.cos(yaw), math.sin(yaw)
            cx = x + c * -0.5 - s * -0.2; cy = y + s * -0.5 + c * -0.2
            rec["map_x"] = round(cx + c * ox - s * oy, 2); rec["map_y"] = round(cy + s * ox + c * oy, 2)
        out.append(rec)
    OBJ.write(json.dumps({"t": round(rospy.get_time(), 3), "pose": pose and [round(v, 3) for v in pose], "objects": out}) + "\n")
    # the follower's own straight-corridor threat
    if CG is not None:
        try:
            summ = CG.parse_summary(m.data)
            th = CG.nearest_threat(summ, CORRIDOR_HALF_WIDTH)
            S["corr"] = None if th is None else (th.label, th.distance_m, th.lateral_m, th.track_id, th.motion)
            S["corr_gr"] = int(th is not None and not th.is_person and th.parked and th.distance_m < 8.0)
            ck = None if th is None else (th.track_id, th.label)
            if ck != S.get("corr_key"):
                if th is None: ev("CORRIDOR THREAT cleared  @%s" % fmt_pose())
                else: ev("CORRIDOR THREAT %s/%s track#%s at %.2f m lateral %s  @%s" % (
                    th.label, th.motion, th.track_id, th.distance_m, None if th.lateral_m is None else round(th.lateral_m, 2), fmt_pose()))
            S["corr_key"] = ck
        except Exception as exc:
            if not S.get("corr_err"):
                ev("corridor threat calc failed: %r" % exc); S["corr_err"] = True

    # the route-based threat (revised checklist), for comparison
    if CG is not None and RT is not None and ROUTE is not None and pose:
        try:
            summ = CG.parse_summary(m.data)
            x, y, yaw, _ = pose; c, s = math.cos(yaw), math.sin(yaw)
            cx = x + c * CC[0] - s * CC[1]; cy = y + s * CC[0] + c * CC[1]
            wh = S.get("wheel"); v = 0.5 * (wh[1] + wh[2]) if wh else 0.0
            d = np.linalg.norm(ROUTE.xy - [cx, cy], axis=1)
            r = RT.evaluate(CG, summ, ROUTE, (cx, cy), yaw, int(np.argmin(d)), v)
            S["rt"] = r
            th = r["route"]
            rk = None if th is None else (th.track_id, th.label)
            if rk != S.get("rt_key"):
                if th is None: ev("ROUTE THREAT cleared  @%s" % fmt_pose())
                else: ev("ROUTE THREAT %s/%s track#%s at %.2f m along route, route d %s%s  @%s" % (
                    th.label, th.motion, th.track_id, th.distance_m,
                    None if getattr(th, "route_d_m", None) is None else round(th.route_d_m, 2),
                    " -> would GO_ROUND" if RT.would_go_round(th) else "", fmt_pose()))
            S["rt_key"] = rk
            if r["broken"] and not S.get("broken_seen"):
                ev("BROKEN profile slice in %d object(s)  @%s" % (r["broken"], fmt_pose())); S["broken_seen"] = True
        except Exception as exc:
            if not S.get("rt_err"):
                ev("route threat calc failed: %r" % exc); S["rt_err"] = True

SMITH = open(OUT + "smith.jsonl", "w", buffering=1)
def on_smith(m):
    try: j = json.loads(m.data)
    except Exception: return
    SMITH.write(m.data + "\n")
    if j.get("mode") != S.get("smith_mode"):
        ev("LATENCY MODE %s horizon %s s" % (j.get("mode"), j.get("horizon_s")))
    S["smith_mode"] = j.get("mode")
    age = j.get("pose_age_s")
    if age is not None and age > 0.3 and time.time() - S.get("stale_ev", 0.0) > 5.0:
        ev("PLANNER POSE STALE %.2f s  @%s" % (age, fmt_pose())); S["stale_ev"] = time.time()
    # Launch after a stop (straight_launch.py, since 2026-11-11): the switches
    # this drive runs with, on every change, and each launch's start and end.
    sw = (j.get("launch_floor"), j.get("straight_launch"))
    if sw != (None, None) and sw != S.get("launch_switches"):
        ev("LAUNCH SWITCHES floor %s m/s, straight %s" % sw); S["launch_switches"] = sw
    act = bool(j.get("launch"))
    if act and not S.get("launch_on"):
        S["launch_t0"] = time.time()
        ev("LAUNCH start wheels %s / %s  @%s" % (j.get("wheel_l"), j.get("wheel_r"), fmt_pose()))
    elif S.get("launch_on") and not act:
        ev("LAUNCH end (%s) after ~%.1f s, wheels %s / %s  @%s" % (
            j.get("launch_exit") or "?", time.time() - S.get("launch_t0", time.time()),
            j.get("wheel_l"), j.get("wheel_r"), fmt_pose()))
    S["launch_on"] = act
    S["smith"] = j; S["smith_rx"] = time.time()

def fmt_pose():
    p = S.get("pose")
    return "-" if not p else "(%.1f,%.1f) yaw %.0f" % (p[0], p[1], math.degrees(p[2]))

# /dwa_follower/smith_state keys, prefixed sm_ in drive.csv (latest message).
SM_COLS = ["sm_mode", "sm_pose_age_s", "sm_wheel_age_s", "sm_lead_m", "sm_lead_yaw_deg",
           "sm_meas_v", "sm_meas_w", "sm_model_v", "sm_model_w", "sm_resid_v", "sm_resid_w",
           "sm_held_v", "sm_held_w", "sm_last_cmd_age_s", "sm_horizon_s",
           # since 2026-11-11 (straight_launch.py)
           "sm_wheel_l", "sm_wheel_r", "sm_launch", "sm_launch_armed", "sm_launch_exit",
           "sm_launch_floor", "sm_straight_launch"]

def main():
    rospy.init_node("drive_logger", anonymous=True, disable_signals=True)
    route_file, band_file = route_paths()
    global CC, ROUTE
    rdoc = json.load(open(route_file))
    wps = np.array([[w["x"], w["y"]] for w in rdoc["waypoints"]])
    ROUTE = RT.Route(wps) if RT is not None else None
    CC = rdoc.get("chair_centre_in_body_xyz") or [-0.5, -0.2, 0.0]
    band = json.load(open(band_file))["stations"]
    bxy = np.array([[s["x"], s["y"]] for s in band])
    caps = [s.get("max_speed_mps") for s in band]
    ev("logger start: route %s  out %s" % (os.path.basename(route_file), OUT))
    # Base and final-stage switches, read once: they are fixed at stack start.
    for name in ("/wheel_cmd/turn_boost", "/wheel_cmd/wheel_separation_m",
                 "/tip_guard/launch_floor_mps", "/waypoint_follower/launch_floor_mps",
                 "/waypoint_follower/straight_launch", "/waypoint_follower/latency_mode"):
        try:
            ev("PARAM %s = %s" % (name, rosparam.get_param(name)))
        except Exception:
            ev("PARAM %s unset" % name)
    rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped, on_pose, queue_size=5)
    for t in ("/cmd_vel_planned", "/cmd_vel_raw", "/cmd_vel_gated", "/cmd_vel_terrain_safe", "/cmd_vel"):
        rospy.Subscriber(t, Twist, mk_twist(t), queue_size=10)
    rospy.Subscriber("/waypoint_follower/status", String, on_status, queue_size=20)
    rospy.Subscriber("/person_bypass/permit", String, on_permit, queue_size=20)
    rospy.Subscriber("/safety_gate/status", String, on_gate, queue_size=20)
    rospy.Subscriber("/semantic_safety/status", String, on_sem, queue_size=20)
    rospy.Subscriber("/tip_guard/status", String, on_tip, queue_size=20)
    rospy.Subscriber("/wheel_status", Int16MultiArray, on_wheel, queue_size=50)
    rospy.Subscriber("/vectornav/IMU", Imu, on_vn, queue_size=50)
    rospy.Subscriber("/livox/imu", Imu, on_livox, queue_size=50)
    rospy.Subscriber("/perception/objects_summary", String, on_objects, queue_size=10)
    rospy.Subscriber("/dwa_follower/smith_state", String, on_smith, queue_size=50)

    cols = ["t", "clock", "status", "x", "y", "yaw_deg", "pitch_deg", "vn_roll_deg", "vn_pitch_deg",
            "wp", "cte_m", "head_err_deg", "band_cap",
            "plan_v", "plan_w", "raw_v", "raw_w", "gated_v", "gated_w", "terr_v", "terr_w", "final_v", "final_w",
            "base_mode", "wheel_l", "wheel_r", "wheel_yaw", "vn_gz", "livox_gz",
            "permit", "permit_track", "permit_tx", "permit_ty", "committed_side",
            "gate_blocked", "gate_reason", "gate_zone_m", "sem_threat", "sem_dist", "sem_motion", "sem_blocked",
            "corr_label", "corr_dist", "corr_lat", "corr_track", "corr_motion", "corr_goround",
            "rt_label", "rt_dist", "rt_d", "rt_track", "rt_motion", "rt_goround",
            "gz_label", "gz_dist", "gz_track", "gz_zone_m", "broken_objs", "chair_s", "chair_d", "tip"] + SM_COLS
    f = open(OUT + "drive.csv", "w", buffering=1)
    f.write(",".join(cols) + "\n")
    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *a: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *a: stop.update(now=True))
    last_hard = 0.0
    while not stop["now"]:
        time.sleep(0.1)
        p = S.get("pose")
        row = {"t": "%.2f" % rospy.get_time(), "clock": time.strftime("%H:%M:%S"), "status": '"%s"' % S.get("status", "")}
        if p:
            bx, by, yaw, pitch = p
            # route/DWA reference point is the chair centre, not the lidar
            c_, s_ = math.cos(yaw), math.sin(yaw)
            x = bx + c_ * CC[0] - s_ * CC[1]; y = by + s_ * CC[0] + c_ * CC[1]
            d = np.linalg.norm(wps - [x, y], axis=1); i = int(np.argmin(d))
            j, k = min(i + 3, len(wps) - 1), max(i - 3, 0)
            h = math.atan2(wps[j][1] - wps[k][1], wps[j][0] - wps[k][0])
            cte = -math.sin(h) * (x - wps[i][0]) + math.cos(h) * (y - wps[i][1])
            he = math.degrees((yaw - h + math.pi) % (2 * math.pi) - math.pi)
            bi = int(np.argmin(np.linalg.norm(bxy - [x, y], axis=1)))
            row.update(x="%.3f" % bx, y="%.3f" % by, yaw_deg="%.1f" % math.degrees(yaw), pitch_deg="%.1f" % math.degrees(pitch),
                       wp=i, cte_m="%+.3f" % cte, head_err_deg="%+.1f" % he, band_cap=caps[bi] if caps[bi] else "")
        vn = S.get("vn")
        if vn: row.update(vn_gz="%+.3f" % vn[0], vn_roll_deg="%.1f" % math.degrees(vn[1]), vn_pitch_deg="%.1f" % math.degrees(vn[2]))
        if "lgz" in S: row["livox_gz"] = "%+.3f" % S["lgz"]
        for key, name in (("/cmd_vel_planned", "plan"), ("/cmd_vel_raw", "raw"), ("/cmd_vel_gated", "gated"),
                          ("/cmd_vel_terrain_safe", "terr"), ("/cmd_vel", "final")):
            if key in S: row[name + "_v"], row[name + "_w"] = "%.3f" % S[key][0], "%+.3f" % S[key][1]
        wh = S.get("wheel")
        if wh: row.update(base_mode=wh[0], wheel_l="%.3f" % wh[1], wheel_r="%.3f" % wh[2], wheel_yaw="%+.3f" % ((wh[2] - wh[1]) / 0.54))
        j = S.get("permit") or {}
        on = bool(j.get("active")) and str(j.get("reason", "")).startswith(("STATIC_OBJECT", "PERSON"))
        ty = j.get("target_y_m")
        row.update(permit=j.get("reason", "") if on else "", permit_track=j.get("track_id") if on else "",
                   permit_tx=j.get("target_x_m") if on else "", permit_ty=ty if on else "",
                   committed_side=("RIGHT" if ty > 0 else "LEFT") if (on and ty is not None) else "")
        g = S.get("gate") or {}
        row.update(gate_blocked=int(bool(g.get("blocked"))), gate_reason=g.get("reason", ""), gate_zone_m=g.get("zone_nearest_m") or "")
        s_ = S.get("sem") or {}
        row.update(sem_threat=s_.get("threat_class") or "", sem_dist=s_.get("threat_distance_m") or "",
                   sem_motion=s_.get("threat_motion") or "", sem_blocked=int(bool(s_.get("blocked"))))
        c = S.get("corr")
        if c: row.update(corr_label=c[0], corr_dist="%.2f" % c[1], corr_lat="" if c[2] is None else "%.2f" % c[2], corr_track=c[3], corr_motion=c[4])
        if c: row["corr_goround"] = S.get("corr_gr", "")
        r = S.get("rt")
        if r:
            th = r["route"]
            if th is not None:
                rd = getattr(th, "route_d_m", None)
                row.update(rt_label=th.label, rt_dist="%.2f" % th.distance_m, rt_d="" if rd is None else "%+.2f" % rd,
                           rt_track=th.track_id, rt_motion=th.motion, rt_goround=int(RT.would_go_round(th)))
            gz = r["gate_zone"]
            if gz is not None:
                row.update(gz_label=gz.label, gz_dist="%.2f" % gz.distance_m, gz_track=gz.track_id, gz_zone_m="%.2f" % gz.zone_m)
            row.update(broken_objs=r["broken"], chair_s="" if r["chair_s"] is None else "%.2f" % r["chair_s"],
                       chair_d="" if r["chair_d"] is None else "%+.2f" % r["chair_d"])
        row["tip"] = '"%s"' % S.get("tip", "")
        sm = S.get("smith")
        # Only while the planner is publishing: a HOLD leaves the last one.
        if sm and time.time() - S.get("smith_rx", 0.0) < 0.5:
            for cn in SM_COLS:
                v = sm.get(cn[3:])
                row[cn] = "" if v is None else v
        f.write(",".join(str(row.get(cn, "")) for cn in cols) + "\n")
        fw = S.get("/cmd_vel", (0, 0))[1]; gz = vn[0] if vn else 0.0
        if (abs(fw) >= 0.4 or abs(gz) >= 0.45) and time.time() - last_hard > 1.5:
            ev("HARD TURN cmd w %+.2f, measured gz %+.2f, head_err %s, cte %s  @%s" % (
                fw, gz, row.get("head_err_deg"), row.get("cte_m"), fmt_pose()))
            last_hard = time.time()
    ev("logger stop")
    f.close(); OBJ.close(); SMITH.close(); EV.close()

if __name__ == "__main__":
    main()

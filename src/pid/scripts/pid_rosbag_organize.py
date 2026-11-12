#!/usr/bin/env python3
"""ROS1 Noetic: split FF-PD READY recordings by drive, safely handle .bag.active.

NUC (ROS Master does not need to be running):
  source /opt/ros/noetic/setup.bash
  python3 scripts/pid_rosbag_organize.py --date 2026-11-10
  python3 scripts/pid_rosbag_organize.py --date 2026-11-10 --write
  python3 scripts/pid_rosbag_organize.py --date 2026-11-10 --write --recover-active

Default = inspection ONLY. --write NEVER deletes or rewrites original files.
Recovery reindexes a COPY of .bag.active, never the original.
--run-dir selects one session; --date is retained for manual inspection.
"""
import argparse
import datetime as dt
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass

PRE_S = 20.0
POST_S = 15.0
TOPICS = ("/ff_pid/status", "/wheel_status", "/mode_cmd", "/cmd_vel",
          "/stop_watchdog/alarm")
CLOSE = {"PAUSED", "MANUAL_INTERVENTION", "GOAL_STOPPING", "GOAL_REACHED",
         "COMPETING_CMD_VEL_PUBLISHER", "EXTERNAL_STOP_REQUEST",
         "SAFETY_POLICY_INVALID"}


@dataclass
class Drive:
    start: float
    end: float
    route: str
    close: str = "UNCLOSED"


def stamp(t):
    return dt.datetime.fromtimestamp(t).strftime("%Y-%m-%d %H:%M:%S")


def wheel(msg):
    try:
        data = list(msg.data)
        if len(data) < 6:
            return None, None
        speeds = []
        for k in (2, 4):
            code, mag = int(data[k]), int(data[k + 1])
            if code not in (67, 83, 87) or not 33 <= mag <= 127:
                return int(data[1]), None
            speeds.append(0.0 if code == 83 else (mag - 33) / 36.0)
        return int(data[1]), max(speeds)
    except (ValueError, TypeError, AttributeError):
        return None, None


def ff_status(msg):
    try:
        data = json.loads(msg.data)
        if isinstance(data, dict):
            return str(data.get("state", "")), str(data.get("route") or "unknown")
    except (ValueError, TypeError, AttributeError):
        pass
    return "", "unknown"


def recorder_alive(directory):
    f = directory / "rosbag.pid"
    try:
        pid = int(f.read_text().strip())
        cmd = (Path("/proc") / str(pid) / "cmdline").read_bytes().replace(b"\0", b" ")
        return b"rosbag" in cmd and b"record" in cmd
    except (OSError, ValueError):
        return False


def enough_space(directory, extra_bytes):
    directory.mkdir(parents=True, exist_ok=True)
    # Keep at least 1 GiB available after copying the recording/export.
    return shutil.disk_usage(str(directory)).free >= extra_bytes + 1024 ** 3


def valid_bag(path):
    import rosbag
    with rosbag.Bag(str(path), "r") as bag:
        count = bag.get_message_count()
        if count <= 0:
            raise ValueError("empty or no recoverable ROS messages")
        return count


def recovered_copy(active, destination):
    """Reindex an isolated copy; original .bag.active is never modified."""
    if destination.exists():
        return valid_bag(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temp = destination.with_name(destination.name + ".active")
    if temp.exists():
        raise RuntimeError("previous recovery attempt needs inspection: " + str(temp))
    if active.stat().st_size == 0:
        raise ValueError("0-byte .bag.active: no data to recover")
    if not enough_space(destination.parent, active.stat().st_size):
        raise RuntimeError("insufficient disk space for active recovery")
    shutil.copy2(str(active), str(temp))
    proc = subprocess.run(["rosbag", "reindex", str(temp)],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          universal_newlines=True, timeout=300)
    if proc.returncode != 0:
        raise RuntimeError("rosbag reindex failed: " + proc.stdout[-500:])
    count = valid_bag(temp)
    os.replace(str(temp), str(destination))
    # rosbag reindex creates a backup of the copied file; the ORIGINAL
    # in READY is still intact. Remove only this redundant local backup.
    for backup in destination.parent.glob(temp.stem + "*.orig*"):
        if backup.is_file():
            backup.unlink()
    return count


def analyze(paths):
    import rosbag
    drives = []
    current = None
    route = "unknown"
    last_mode = None
    seen_wheel = 0
    seen_ff = 0
    auto_without_drive = False
    manual_moved = False
    alarm_seen = False
    last_t = None

    def close(t, why):
        nonlocal current
        if current is not None:
            current.end = max(current.end, t)
            current.close = why
            drives.append(current)
            current = None

    for path in paths:
        with rosbag.Bag(str(path), "r") as bag:
            for topic, msg, ros_time in bag.read_messages(topics=TOPICS):
                t = ros_time.to_sec()
                last_t = t
                if topic == "/ff_pid/status":
                    state, name = ff_status(msg)
                    seen_ff += 1
                    if name != "unknown":
                        route = name
                    if state == "RUNNING":
                        if current is None:
                            current = Drive(t, t, route)
                        current.end = t
                    elif state in CLOSE:
                        close(t, state)
                    elif state.startswith(("ERROR_STOP:", "SAFETY_STOP:")):
                        close(t, state)
                    elif current is not None:
                        current.end = t
                elif topic == "/wheel_status":
                    mode, speed = wheel(msg)
                    seen_wheel += 1
                    if speed is not None and speed >= 0.06:
                        if mode == 65:
                            if current is None:
                                current = Drive(t, t, route)
                            current.end = t
                        elif mode == 77:
                            manual_moved = True
                    if last_mode == 65 and mode == 77:
                        close(t, "MANUAL_MODE")
                    if mode == 65 and current is None:
                        auto_without_drive = True
                    if mode is not None:
                        last_mode = mode
                elif topic == "/mode_cmd":
                    if msg.data == 77:
                        close(t, "MANUAL_REQUEST")
                    elif msg.data == 65 and current is None:
                        auto_without_drive = True
                elif topic == "/stop_watchdog/alarm":
                    # Diagnostics are valuable even if no successful drive.
                    try:
                        a = json.loads(msg.data)
                        alarm_seen |= (isinstance(a, dict) and (
                            a.get("alarm") is True or
                            a.get("status") in ("ERROR", "ALARM", "FAULT")))
                    except (ValueError, AttributeError, TypeError):
                        alarm_seen = True

    if current is not None:
        close(current.end, "UNCLOSED")
    uncertainty = (auto_without_drive or manual_moved or alarm_seen or
                   any(d.close == "UNCLOSED" for d in drives))
    evidence = dict(wheel_messages=seen_wheel, ff_status_messages=seen_ff,
                    auto_without_drive=auto_without_drive,
                    manual_motion=manual_moved, watchdog_alarm=alarm_seen,
                    uncertain=uncertainty, last_message=stamp(last_t) if last_t else None)
    return drives, evidence


def export(paths, drive, dest):
    import genpy
    import rosbag
    if not enough_space(dest.parent, sum(p.stat().st_size for p in paths)):
        raise RuntimeError("insufficient disk space for drive export")
    lo = genpy.Time.from_sec(max(0.0, drive.start - PRE_S))
    hi = genpy.Time.from_sec(drive.end + POST_S)
    temp = dest.with_name(dest.name + ".partial")
    if temp.exists():
        raise FileExistsError(str(temp))
    written = 0
    try:
        with rosbag.Bag(str(temp), "w") as out:
            for path in paths:
                with rosbag.Bag(str(path), "r") as src:
                    for topic, raw, t, header in src.read_messages(
                            start_time=lo, end_time=hi, raw=True,
                            return_connection_header=True):
                        out.write(topic, raw, t=t, raw=True, connection_header=header)
                        written += 1
        if written == 0 or valid_bag(temp) != written:
            raise RuntimeError("export message count verification failed")
        os.replace(str(temp), str(dest))
        return written
    except Exception:
        # Retain a failed .partial for debugging, never delete sources.
        raise


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--date", help="READY directory date: YYYY-MM-DD")
    group.add_argument("--run-dir", help="one READY session directory")
    ap.add_argument("--input-root", default="~/pid_ready_logs")
    ap.add_argument("--output-root", default="~/pid_drive_bags")
    ap.add_argument("--write", action="store_true", help="write separated drives; originals untouched")
    ap.add_argument("--recover-active", action="store_true",
                    help="with --write, try to recover nonzero .bag.active COPIES")
    args = ap.parse_args()
    if args.date:
        try:
            dt.datetime.strptime(args.date, "%Y-%m-%d")
        except ValueError:
            ap.error("invalid --date")
    if args.recover_active and not args.write:
        ap.error("--recover-active requires --write")
    try:
        import rosbag  # noqa: F401
    except ImportError:
        ap.error("ROS Noetic required: source /opt/ros/noetic/setup.bash")
    root = Path(args.input_root).expanduser().resolve()
    out = Path(args.output_root).expanduser().resolve()
    if out == root or root in out.parents or out in root.parents:
        ap.error("input and output directories must be separate")
    if not root.is_dir():
        ap.error("READY log directory missing: " + str(root))
    if args.run_dir:
        run = Path(args.run_dir).expanduser().resolve()
        if not run.is_dir() or run.parent != root or not re.match(r"^\d{8}_", run.name):
            ap.error("--run-dir must be a READY session in --input-root")
        runs = [run]
    else:
        prefix = args.date.replace("-", "") + "_"
        runs = sorted(p for p in root.iterdir() if p.is_dir() and p.name.startswith(prefix))
    if not runs:
        print("[INFO] No matching READY session directories")
        return 0
    for run in runs:
        closed = sorted(run.glob("ready_topics*.bag"))
        active = sorted(run.glob("ready_topics*.bag.active"))
        live = recorder_alive(run)
        target = out / run.name[:8] / run.name
        print("\n=== %s: %d bag, %d active, live=%s ===" %
              (run.name, len(closed), len(active), live), flush=True)
        if live:
            print("[KEEP] Recorder still running; no processing or deletion")
            continue
        if active and any(time.time() - p.stat().st_mtime < 120 for p in active):
            print("[KEEP] Active file has changed recently; retry later")
            continue
        files = list(closed)
        recovery = []
        if active:
            for f in active:
                if f.stat().st_size == 0:
                    print("[KEEP] Zero-byte active (no data): " + f.name)
                    recovery.append({"source": str(f), "status": "ZERO_BYTES"})
                    continue
                if not (args.write and args.recover_active):
                    print("[KEEP] Incomplete recording: " + f.name +
                          " (use --write --recover-active)")
                    recovery.append({"source": str(f), "status": "NOT_RECOVERED"})
                    continue
                dest = target / "recovered" / f.name[:-7]
                try:
                    n = recovered_copy(f, dest)
                    files.append(dest)
                    recovery.append({"source": str(f), "status": "RECOVERED_COPY",
                                     "copy": str(dest), "messages": n})
                    print("[RECOVERED COPY] %s: %d messages" % (f.name, n))
                except Exception as exc:
                    recovery.append({"source": str(f), "status": "FAILED",
                                     "reason": str(exc)})
                    print("[KEEP] Recovery failed: %s (%s)" % (f.name, exc))
        files.sort(key=lambda p: p.name)
        if not files:
            print("[KEEP] No readable bag files")
            continue
        try:
            drives, evidence = analyze(files)
        except Exception as exc:
            print("[KEEP] Bag analysis failed: " + str(exc))
            continue
        print("[INFO] %d drives; evidence=%s" %
              (len(drives), json.dumps(evidence, ensure_ascii=False)))
        for i, d in enumerate(drives, 1):
            print("  #%02d %s -> %s route=%s end=%s" %
                  (i, stamp(d.start), stamp(d.end), d.route, d.close))
        if not args.write:
            print("[DRY RUN] Nothing changed")
            continue
        target.mkdir(parents=True, exist_ok=True)
        output_names = []
        completed = True
        for i, d in enumerate(drives, 1):
            route = re.sub(r"[^A-Za-z0-9_-]", "_", d.route)[:48]
            name = "drive_%02d_%s_%s.bag" % (
                i, dt.datetime.fromtimestamp(d.start).strftime("%H%M%S"), route)
            dest = target / name
            try:
                if dest.exists():
                    n = valid_bag(dest)
                    print("[EXISTS] %s: %d messages" % (name, n))
                else:
                    n = export(files, d, dest)
                    print("[SAVED] %s: %d messages" % (name, n))
                output_names.append(str(dest))
            except Exception as exc:
                completed = False
                print("[KEEP] Export failed: %s (%s)" % (name, exc))
                break
        report = dict(date=run.name[:8], run=str(run), originals=[str(x) for x in closed],
                      recovery=recovery, evidence=evidence,
                      drives=[dict(start=stamp(d.start), end=stamp(d.end),
                                   route=d.route, close=d.close) for d in drives],
                      outputs=output_names, exports_verified=completed,
                      originals_deleted=False)
        # This tool never deletes original READY recordings.
        tmp = target / "report.json.tmp"
        tmp.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                       encoding="utf-8")
        os.replace(str(tmp), str(target / "report.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())

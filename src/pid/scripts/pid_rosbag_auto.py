#!/usr/bin/env python3
"""Non-driving READY rosbag automation. NEVER deletes or edits original recordings.

Invoked automatically after successful READY shutdown and by a user-systemd
timer after login/boot (with lingering enabled). It only runs the existing
pid_rosbag_organize.py on finished sessions; it does not interact with ROS
Master, motor commands, or sensor nodes.

Writes per-drive bags under ~/pid_drive_bags/YYYYMMDD/<READY-session>/.
If any uncertainty or disk-pressure exists, skips work and preserves sources.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

GIB = 1024 ** 3
SESSION = re.compile(r"^\d{8}_\d{6}_[A-Za-z0-9]+$")
OWNED = {"roscore": "roscore", "livox": "msg_MID360.launch",
         "fastlio": "mapping_mid360.launch", "localization": "moving_localization.launch",
         "base": "wheel.launch", "geometry": "hybrid_geometric_objects.py",
         "fusion": "hybrid_object_fusion.py",
         "exclusion": "localization_exclusion_boxes.py",
         "stop_watchdog": "stop_watchdog.py"}


def identity_alive(pid, contains):
    """Do not confuse dead PIDs or reused PIDs with READY-owned processes."""
    try:
        pid = int(pid)
        cmd = (Path("/proc") / str(pid) / "cmdline").read_bytes().replace(b"\0", b" ")
        stat = (Path("/proc") / str(pid) / "stat").read_text().split(") ", 1)[1][0]
        return stat != "Z" and contains.encode() in cmd
    except (OSError, ValueError, IndexError):
        return False


def session_busy(run):
    pid_file = run / "rosbag.pid"
    try:
        pid = pid_file.read_text().strip()
        # Recorder argv contains the READY run directory's rosbag prefix.
        if identity_alive(pid, "rosbag") and identity_alive(pid, str(run / "ready_topics")):
            return True
    except OSError:
        pass
    registry = run / "started_pids.tsv"
    try:
        lines = registry.read_text().splitlines()
    except OSError:
        lines = []
    for line in lines:
        fields = line.split("\t")
        if len(fields) == 2 and fields[0] in OWNED:
            if identity_alive(fields[1], OWNED[fields[0]]):
                return True
    return False


def sources(run):
    return sorted(run.glob("ready_topics*.bag")) + sorted(run.glob("ready_topics*.bag.active"))


def signature(paths):
    return [{"name": p.name, "size": p.stat().st_size,
             "mtime_ns": p.stat().st_mtime_ns} for p in paths]


def complete(run, out, sig):
    marker = out / run.name[:8] / run.name / "auto_complete.json"
    try:
        saved = json.loads(marker.read_text())
        if saved["source_signature"] != sig:
            return False
        report = json.loads((marker.parent / "report.json").read_text())
        if not report.get("exports_verified"):
            return False
        if any(item["status"] not in ("RECOVERED_COPY", "ZERO_BYTES")
               for item in report.get("recovery", [])):
            return False
        return all(Path(p).is_file() and Path(p).stat().st_size > 0
                   for p in report.get("outputs", []))
    except (OSError, ValueError, KeyError, TypeError):
        return False


def pending_runs(root, out):
    if not root.is_dir():
        return []
    result = []
    for run in sorted(root.iterdir()):
        if not run.is_dir() or not SESSION.fullmatch(run.name):
            continue
        source_files = sources(run)
        if not source_files or session_busy(run):
            continue
        # Even after a crash, let an active file settle before copying it.
        if any(p.name.endswith(".active")
               and time.time() - p.stat().st_mtime < 120 for p in source_files):
            continue
        sig = signature(source_files)
        if complete(run, out, sig):
            continue
        result.append((run, source_files, sig))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input-root", default="~/pid_ready_logs")
    p.add_argument("--output-root", default="~/pid_drive_bags")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()
    root = Path(args.input_root).expanduser().resolve()
    out = Path(args.output_root).expanduser().resolve()
    if root == out or root in out.parents or out in root.parents:
        p.error("input and output roots must not overlap")
    if not root.is_dir():
        print("[INFO] No READY recording directory yet")
        return 0
    out.mkdir(parents=True, exist_ok=True)
    with (out / ".rosbag_auto.lock").open("a+") as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("[SKIP] Another organizer is running; will retry later")
            return 0
        candidates = pending_runs(root, out)
        if not candidates:
            print("[OK] No pending completed READY recordings")
            return 0

        trouble = False
        for run, paths, sig in candidates:
            # Recheck after acquiring the global lock; a new READY may have
            # started since discovery. A running recorder is never touched.
            if session_busy(run):
                print("[SKIP] %s: READY processes still running" % run.name)
                continue
            total = sum(f.stat().st_size for f in paths)
            free = shutil.disk_usage(str(out)).free
            # Conservative: source-sized export + source-sized recovery copy
            # + 2GiB reserve. Failure skips, never removes originals.
            if free < 2 * total + 2 * GIB:
                print("[SKIP] %s: disk low (free %.1f GiB, need %.1f GiB)" %
                      (run.name, free / GIB, (2 * total + 2 * GIB) / GIB))
                trouble = True
                continue
            print("[INFO] %s: %.2f GiB input, %.2f GiB free" %
                  (run.name, total / GIB, free / GIB), flush=True)
            if args.dry_run:
                continue
            organizer = Path(__file__).resolve().with_name("pid_rosbag_organize.py")
            if not organizer.is_file():
                print("[ERROR] Organizer missing: %s" % organizer)
                trouble = True
                continue
            command = [sys.executable, str(organizer),
                       "--run-dir", str(run), "--write", "--recover-active",
                       "--input-root", str(root), "--output-root", str(out)]
            print("[RUN] %s" % " ".join(command), flush=True)
            rc = subprocess.call(command)
            if rc:
                print("[WARN] Organizer failed (%d); originals preserved" % rc)
                trouble = True
                continue
            target = out / run.name[:8] / run.name
            try:
                report = json.loads((target / "report.json").read_text())
                recovered = report.get("recovery", [])
                outputs = report.get("outputs", [])
                verified = (report.get("exports_verified") is True
                            and all(it.get("status") in ("RECOVERED_COPY", "ZERO_BYTES")
                                    for it in recovered)
                            and all(Path(f).is_file() and Path(f).stat().st_size > 0
                                    for f in outputs))
                if not verified or sig != signature(sources(run)) or session_busy(run):
                    raise ValueError("incomplete exports or session changed")
                record = {"source_signature": sig, "processed_at": time.time(),
                          "outputs": outputs, "originals_deleted": False}
                tmp = target / "auto_complete.json.tmp"
                tmp.write_text(json.dumps(record, indent=2) + "\n")
                os.replace(str(tmp), str(target / "auto_complete.json"))
                print("[OK] %s: %d separate drives; originals intact" %
                      (run.name, len(outputs)), flush=True)
            except (OSError, ValueError, TypeError) as exc:
                print("[WARN] %s remains pending: %s" % (run.name, exc))
                trouble = True
        return 1 if trouble else 0


if __name__ == "__main__":
    sys.exit(main())

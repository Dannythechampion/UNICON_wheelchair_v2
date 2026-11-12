#!/usr/bin/env python3
"""Apply only the PID on-demand occupancy hook to a LOCAL team checkout.

This script changes src/static_livox_localization/scripts/obstacle_clusters.py
without replacing the file, preserving independent ground-filter fixes.
Run after copying personal PID scripts into the team checkout. Idempotent.
"""
import argparse
from pathlib import Path

MARK = "# PID_OCCUPANCY_HOOK_V1"


def guard_snapshot_calls(source):
    """Keep legacy __new__-constructed test fixtures usable without ROS Service.

    Production nodes always construct SnapshotService in __init__. This guard
    only skips the optional service when that constructor was bypassed.
    """
    clear_old = '        self.pid_occupancy_snapshot.clear()'
    clear_new = ('        if hasattr(self, "pid_occupancy_snapshot"):\n'
                 '            self.pid_occupancy_snapshot.clear()')
    update_old = '''        self.pid_occupancy_snapshot.update(
            newest_stamp, map_pose @ body_T_lidar,
            [(i if not tracks else int(tracks[i].id), cluster)
             for i, cluster in enumerate(clusters)])'''
    update_new = '''        if hasattr(self, "pid_occupancy_snapshot"):
            self.pid_occupancy_snapshot.update(
                newest_stamp, map_pose @ body_T_lidar,
                [(i if not tracks else int(tracks[i].id), cluster)
                 for i, cluster in enumerate(clusters)])'''
    if clear_old in source:
        source = source.replace(clear_old, clear_new, 1)
    if update_old in source:
        source = source.replace(update_old, update_new, 1)
    return source


def patch(source):
    if MARK in source:
        return guard_snapshot_calls(source)
    replacements = [
        ("from cloud_points import (COLLISION_MAX_HEIGHT_M,",
         """# PID occupied cells use a separate service; legacy summary unchanged.
import sys as _pid_occ_sys
from pathlib import Path as _pid_occ_Path
_pid_occ_sys.path.insert(0, str(_pid_occ_Path(__file__).resolve().parents[2] / "pid" / "scripts"))
from pid_occupancy_snapshot import SnapshotService
""" + MARK + """
from cloud_points import (COLLISION_MAX_HEIGHT_M,"""),
        ("        self.map_poses = MapPoseBuffer()",
         "        self.map_poses = MapPoseBuffer()\n        self.pid_occupancy_snapshot = SnapshotService()"),
        ("    def publish_empty(self, source_stamp, status):",
         "    def publish_empty(self, source_stamp, status):\n        self.pid_occupancy_snapshot.clear()"),
        ("        tracks = self.track(boxes)",
         """        tracks = self.track(boxes)
        # Capture only the latest filtered, already-clustered scan. Map
        # conversion uses the SAME synchronized map pose as geometric output.
        # The grid is constructed only when a stopped A* worker requests it.
        self.pid_occupancy_snapshot.update(
            newest_stamp, map_pose @ body_T_lidar,
            [(i if not tracks else int(tracks[i].id), cluster)
             for i, cluster in enumerate(clusters)])"""),
    ]
    result = source
    for old, new in replacements:
        count = result.count(old)
        if count != 1:
            raise RuntimeError("Patch rejected: expected exactly one anchor (%d): %s" % (count, old))
        result = result.replace(old, new, 1)
    return guard_snapshot_calls(result)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("team_checkout", help="Path to team repository root")
    args = ap.parse_args()
    target = (Path(args.team_checkout).resolve()
              / "src/static_livox_localization/scripts/obstacle_clusters.py")
    if not target.is_file():
        ap.error("expected team perception file not found: " + str(target))
    content = target.read_text(encoding="utf-8")
    revised = patch(content)
    if revised == content:
        print("Already patched; no changes")
        return
    compile(revised, str(target), "exec")  # fail before writing
    target.write_text(revised, encoding="utf-8")
    print("Patched geometric perception with on-demand PID occupancy service:", target)


if __name__ == "__main__":
    main()

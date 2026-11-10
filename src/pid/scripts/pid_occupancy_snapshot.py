#!/usr/bin/env python3
"""On-demand 20 cm occupied-cell snapshot from existing geometric LiDAR clusters.

This module belongs alongside static_livox_localization/scripts/obstacle_clusters.py
when deployed. It does NOT alter clustering, filtering, or motion classification.
No ROS dependency is imported until SnapshotService is instantiated.
"""
import json
import math
import threading

import numpy as np

SNAPSHOT_SERVICE = "/perception/pid_occupancy_snapshot"
CELL_M = 0.20
MAX_CELLS = 12000
MAX_CLUSTERS = 40
MAX_AGE_S = 0.8


def snapshot_from_clusters(stamp, map_t_lidar, tracked_clusters, cell_m=CELL_M):
    """Express occupied LiDAR XY cells in the immutable map frame.

    tracked_clusters is [(tracked_id, Nx3 lidar-frame xyz), ...]. Coordinates
    represent occupied 20 cm SQUARES, never infinitesimal occupied points.
    """
    transform = np.asarray(map_t_lidar, dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not math.isfinite(float(stamp)) or float(stamp) <= 0
            or not 0.05 <= float(cell_m) <= 0.5
            or len(tracked_clusters) > MAX_CLUSTERS):
        raise ValueError("invalid occupancy source")
    objects, count = [], 0
    for object_id, cloud in tracked_clusters:
        points = np.asarray(cloud, dtype=float)
        if (points.ndim != 2 or points.shape[1] < 3
                or not np.isfinite(points[:, :3]).all()):
            raise ValueError("invalid occupancy cluster")
        if len(points) == 0:
            continue
        map_xy = points[:, :3] @ transform[:2, :3].T + transform[:2, 3]
        cells = np.unique(np.floor(map_xy / cell_m).astype(np.int32), axis=0)
        count += len(cells)
        if count > MAX_CELLS:
            raise ValueError("occupancy snapshot too large")
        objects.append({"id": int(object_id), "cells": cells.tolist()})
    return {"status": "OK", "frame": "map", "stamp": float(stamp),
            "cell_m": float(cell_m), "objects": objects, "cell_count": count}


class SnapshotService:
    """Keep only the LAST filtered cluster observation, answer ROS Trigger on request.

    The 5 Hz acquisition path stores cluster references/arrays; grid quantization
    and JSON serialization happen ONLY when A* requests a snapshot.
    """
    def __init__(self, service_name=SNAPSHOT_SERVICE):
        import rospy
        from std_srvs.srv import Trigger
        self.rospy = rospy
        self.lock = threading.Lock()
        self.data = None
        self.server = rospy.Service(service_name, Trigger, self.handle)

    def clear(self):
        with self.lock:
            self.data = None

    def update(self, stamp, map_t_lidar, tracked_clusters):
        matrix = np.array(map_t_lidar, dtype=float, copy=True)
        clusters = [(int(object_id), np.array(points[:, :3], dtype=np.float32, copy=True))
                    for object_id, points in tracked_clusters]
        with self.lock:
            self.data = (float(stamp), matrix, clusters)

    def handle(self, _request):
        from std_srvs.srv import TriggerResponse
        with self.lock:
            stored = self.data
        if stored is None:
            return TriggerResponse(success=False, message="NO_OCCUPANCY")
        stamp, matrix, clusters = stored
        now = self.rospy.Time.now().to_sec()
        if not (stamp <= now + 0.2 and now - stamp <= MAX_AGE_S):
            return TriggerResponse(success=False, message="OCCUPANCY_STALE")
        try:
            snap = snapshot_from_clusters(stamp, matrix, clusters)
            return TriggerResponse(success=True, message=json.dumps(
                snap, separators=(",", ":"), allow_nan=False))
        except (TypeError, ValueError, OverflowError) as exc:
            return TriggerResponse(success=False, message="BAD_OCCUPANCY: " + str(exc)[:100])

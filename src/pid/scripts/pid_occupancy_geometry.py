"""ROS-independent occupied-cell geometry shared by A* and Safety.

Occupied cells are 2-D map squares. A KD-tree nearest-centre test with the
half-diagonal added to the clearance radius is conservative even at corners.
This is NOT an additional chassis inflation: the passed radius already has
the planner's collision clearance meaning.
"""
import math
import numpy as np
from scipy.spatial import cKDTree

MAX_SNAPSHOT_CELLS = 12000


class OccupiedCells:
    def __init__(self, snapshot):
        if (not isinstance(snapshot, dict) or snapshot.get("status") != "OK"
                or snapshot.get("frame") != "map"):
            raise ValueError("invalid occupancy snapshot status/frame")
        self.stamp = float(snapshot["stamp"])
        self.cell_m = float(snapshot["cell_m"])
        if not math.isfinite(self.stamp) or self.stamp <= 0 or not 0.05 <= self.cell_m <= 0.5:
            raise ValueError("invalid occupancy timestamp or resolution")
        grouped = snapshot.get("objects")
        if not isinstance(grouped, list):
            raise ValueError("missing occupancy objects")
        ids, cells = set(), []
        for obj in grouped:
            oid = int(obj["id"])
            if oid in ids:
                raise ValueError("duplicate occupied object ID")
            ids.add(oid)
            array = np.asarray(obj["cells"])
            if array.size == 0:
                continue
            if array.ndim != 2 or array.shape[1] != 2 or not np.isfinite(array).all():
                raise ValueError("bad occupied cells")
            if not np.array_equal(array, np.rint(array)) or np.max(np.abs(array)) > 1000000:
                raise ValueError("bad occupied cell indices")
            cells.append(array.astype(np.int32))
        merged = np.unique(np.concatenate(cells, axis=0), axis=0) if cells else np.empty((0,2), dtype=np.int32)
        if len(merged) > MAX_SNAPSHOT_CELLS:
            raise ValueError("occupancy snapshot too large")
        self.ids = ids
        self.cells = merged
        self.centres = (merged.astype(float) + 0.5) * self.cell_m
        self.tree = cKDTree(self.centres) if len(merged) else None
        self.half_diagonal = self.cell_m / math.sqrt(2)

    def free(self, xy, radius):
        arr = np.asarray(xy, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 2 or not np.isfinite(arr).all():
            raise ValueError("invalid collision sample")
        radius = float(radius)
        if radius < 0 or not math.isfinite(radius):
            raise ValueError("invalid clearance radius")
        if self.tree is None:
            raise ValueError("empty occupancy: cannot establish obstruction")
        distances, _ = self.tree.query(arr, k=1, workers=1)
        return distances > radius + self.half_diagonal

    def has_id(self, object_id):
        return int(object_id) in self.ids

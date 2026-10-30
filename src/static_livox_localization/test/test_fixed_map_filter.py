"""Fixed-map subtraction keeps novel objects while removing mapped walls."""

import hashlib
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    from fixed_map_filter import FixedMapFilter
finally:
    sys.path.pop(0)


def write_pcd(path, xyz, trailing=b""):
    xyz = np.asarray(xyz, dtype=np.float32)
    records = np.column_stack(
        [xyz, np.ones(len(xyz), dtype=np.float32)]
    ).astype(np.float32)
    header = (
        "VERSION .7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "DATA binary\n"
    ) % (len(records), len(records))
    path.write_bytes(header.encode("ascii") + records.tobytes() + trailing)


def test_mapped_wall_is_removed_but_nearby_person_survives(tmp_path):
    path = tmp_path / "map.pcd"
    wall = np.asarray([
        [x, 0.0, z]
        for x in np.arange(1.0, 2.1, 0.05)
        for z in np.arange(0.0, 1.1, 0.05)
    ])
    write_pcd(path, wall)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    fixed_map = FixedMapFilter(path, digest, tolerance_m=0.15)
    scan = np.asarray([
        [1.5, 0.04, 0.5, 1.0],
        [1.5, 0.22, 0.5, 2.0],
    ])

    novel = fixed_map.retain_novel(scan, np.eye(4))

    assert novel.shape == (1, 4)
    assert novel[0, 3] == 2.0


def test_map_hash_mismatch_fails_closed(tmp_path):
    path = tmp_path / "map.pcd"
    write_pcd(path, [[0.0, 0.0, 0.0]])

    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        FixedMapFilter(path, "0" * 64, tolerance_m=0.15)


def test_trailing_zero_padding_is_not_a_size_mismatch(tmp_path):
    # route2_intern.pcd carries exactly this much writer padding.
    path = tmp_path / "map.pcd"
    write_pcd(path, [[1.5, 0.0, 0.5]], trailing=b"\0" * 3904)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    fixed_map = FixedMapFilter(path, digest, tolerance_m=0.15)

    novel = fixed_map.retain_novel(
        np.asarray([[1.5, 0.04, 0.5, 1.0], [1.5, 0.4, 0.5, 2.0]]), np.eye(4))

    assert novel.shape == (1, 4)
    assert novel[0, 3] == 2.0


def test_nonzero_tail_is_still_a_size_mismatch(tmp_path):
    path = tmp_path / "map.pcd"
    write_pcd(path, [[1.5, 0.0, 0.5]], trailing=b"\0\0\0\x01")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()

    with pytest.raises(ValueError, match="payload size mismatch"):
        FixedMapFilter(path, digest, tolerance_m=0.15)

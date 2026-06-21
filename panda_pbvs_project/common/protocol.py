from __future__ import annotations

import struct
import numpy as np

POSE_FORMAT = "<6f"
POSE_SIZE = struct.calcsize(POSE_FORMAT)
MATRIX_FORMAT = "<16d"
MATRIX_SIZE = struct.calcsize(MATRIX_FORMAT)


def pack_pose6(pose: np.ndarray) -> bytes:
    pose = np.asarray(pose, dtype=np.float32).reshape(6)
    return struct.pack(POSE_FORMAT, *pose)


def unpack_pose6(data: bytes) -> np.ndarray:
    if len(data) != POSE_SIZE:
        raise ValueError(f"Expected {POSE_SIZE} bytes, got {len(data)}.")
    return np.asarray(struct.unpack(POSE_FORMAT, data), dtype=float)


def pack_matrix4(transform: np.ndarray) -> bytes:
    transform = np.asarray(transform, dtype=float).reshape(4, 4)
    return struct.pack(MATRIX_FORMAT, *transform.reshape(-1))


def unpack_matrix4(data: bytes) -> np.ndarray:
    if len(data) != MATRIX_SIZE:
        raise ValueError(f"Expected {MATRIX_SIZE} bytes, got {len(data)}.")
    return np.asarray(struct.unpack(MATRIX_FORMAT, data), dtype=float).reshape(4, 4)

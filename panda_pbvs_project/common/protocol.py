from __future__ import annotations

from dataclasses import dataclass
import math
import struct

import numpy as np


POSE_FORMAT = "<6f"
POSE_SIZE = struct.calcsize(POSE_FORMAT)

TASK_POSE_MAGIC = b"PTP2"
TASK_POSE_VERSION = 2
TASK_POSE_FORMAT = "<4sBBHQf16d"
TASK_POSE_SIZE = struct.calcsize(TASK_POSE_FORMAT)


@dataclass(frozen=True)
class TaskPosePacket:
    T_TS: np.ndarray
    sequence_id: int
    confidence: float
    valid: bool


def pack_pose6(pose: np.ndarray) -> bytes:
    pose = np.asarray(pose, dtype=np.float32).reshape(6)
    return struct.pack(POSE_FORMAT, *pose)


def unpack_pose6(data: bytes) -> np.ndarray:
    if len(data) != POSE_SIZE:
        raise ValueError(
            f"Expected {POSE_SIZE} bytes, got {len(data)}."
        )
    return np.asarray(
        struct.unpack(POSE_FORMAT, data),
        dtype=float,
    )


def pack_task_pose(
    T_TS: np.ndarray,
    *,
    sequence_id: int,
    confidence: float,
    valid: bool = True,
) -> bytes:
    """
    Encode the only supported task-pose wire format.

    sequence_id must increase only for a genuinely new tracker estimate.
    Republishing one estimate must preserve its sequence_id.
    """

    if (
        isinstance(sequence_id, bool)
        or not isinstance(sequence_id, (int, np.integer))
    ):
        raise TypeError("sequence_id must be an integer.")

    sequence_id = int(sequence_id)
    if not 0 <= sequence_id <= (1 << 64) - 1:
        raise ValueError("sequence_id must fit in uint64.")

    confidence = float(confidence)
    if not math.isfinite(confidence):
        raise ValueError("confidence must be finite.")
    if not 0.0 <= confidence <= 1.0:
        raise ValueError(
            "confidence must be between 0 and 1."
        )

    transform = np.asarray(T_TS, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError(
            "T_TS must have shape (4, 4), "
            f"got {transform.shape}."
        )

    return struct.pack(
        TASK_POSE_FORMAT,
        TASK_POSE_MAGIC,
        TASK_POSE_VERSION,
        int(bool(valid)),
        0,
        sequence_id,
        confidence,
        *transform.reshape(-1),
    )


def unpack_task_pose(data: bytes) -> TaskPosePacket:
    """Decode the only supported task-pose wire format."""

    if len(data) != TASK_POSE_SIZE:
        raise ValueError(
            f"Expected {TASK_POSE_SIZE} bytes, "
            f"got {len(data)}."
        )

    (
        magic,
        version,
        valid_value,
        reserved,
        sequence_id,
        confidence,
        *matrix_values,
    ) = struct.unpack(TASK_POSE_FORMAT, data)

    if magic != TASK_POSE_MAGIC:
        raise ValueError(
            f"Invalid task-pose magic: {magic!r}."
        )
    if version != TASK_POSE_VERSION:
        raise ValueError(
            "Unsupported task-pose protocol version: "
            f"{version}."
        )
    if valid_value not in (0, 1):
        raise ValueError(
            "Task-pose valid field must be 0 or 1."
        )
    if reserved != 0:
        raise ValueError(
            "Task-pose reserved field must be zero."
        )

    confidence = float(confidence)
    if (
        not math.isfinite(confidence)
        or not 0.0 <= confidence <= 1.0
    ):
        raise ValueError(
            "Decoded confidence is invalid."
        )

    return TaskPosePacket(
        T_TS=np.asarray(
            matrix_values,
            dtype=float,
        ).reshape(4, 4),
        sequence_id=int(sequence_id),
        confidence=confidence,
        valid=bool(valid_value),
    )

from __future__ import annotations

from dataclasses import dataclass
import math
import struct

import numpy as np


POSE_FORMAT = "<6f"
POSE_SIZE = struct.calcsize(POSE_FORMAT)

MATRIX_FORMAT = "<16d"
MATRIX_SIZE = struct.calcsize(MATRIX_FORMAT)


# Existing matrix-only task-pose packet.
TASK_POSE_LEGACY_VERSION = 1


# Version 2:
#
#   magic:       4 bytes
#   version:     uint8
#   valid:       uint8
#   reserved:    uint16
#   sequence_id: uint64
#   confidence:  float32
#   T_TS:        16 float64 values
#
# Total: 148 bytes.
TASK_POSE_V2_MAGIC = b"PTP2"
TASK_POSE_V2_VERSION = 2
TASK_POSE_V2_FORMAT = "<4sBBHQf16d"
TASK_POSE_V2_SIZE = struct.calcsize(
    TASK_POSE_V2_FORMAT
)


@dataclass(frozen=True)
class TaskPosePacket:
    T_TS: np.ndarray
    version: int
    sequence_id: int | None
    confidence: float
    valid: bool


def pack_pose6(pose: np.ndarray) -> bytes:
    pose = np.asarray(
        pose,
        dtype=np.float32,
    ).reshape(6)

    return struct.pack(
        POSE_FORMAT,
        *pose,
    )


def unpack_pose6(data: bytes) -> np.ndarray:
    if len(data) != POSE_SIZE:
        raise ValueError(
            f"Expected {POSE_SIZE} bytes, got {len(data)}."
        )

    return np.asarray(
        struct.unpack(POSE_FORMAT, data),
        dtype=float,
    )


def pack_matrix4(transform: np.ndarray) -> bytes:
    transform = np.asarray(
        transform,
        dtype=float,
    ).reshape(4, 4)

    return struct.pack(
        MATRIX_FORMAT,
        *transform.reshape(-1),
    )


def unpack_matrix4(data: bytes) -> np.ndarray:
    if len(data) != MATRIX_SIZE:
        raise ValueError(
            f"Expected {MATRIX_SIZE} bytes, got {len(data)}."
        )

    return np.asarray(
        struct.unpack(MATRIX_FORMAT, data),
        dtype=float,
    ).reshape(4, 4)


def pack_task_pose(T_TS: np.ndarray) -> bytes:
    """
    Encode the existing matrix-only packet.

    Retained temporarily so the current simulator remains compatible.
    """

    return pack_matrix4(T_TS)


def unpack_task_pose(data: bytes) -> np.ndarray:
    """Decode the existing matrix-only packet."""

    return unpack_matrix4(data)


def pack_task_pose_v2(
    T_TS: np.ndarray,
    *,
    sequence_id: int,
    confidence: float,
    valid: bool = True,
) -> bytes:
    """
    Encode a task-pose packet with source observation metadata.

    sequence_id must increase only when the visual tracker produces a
    genuinely new pose estimate. Re-publication of an old estimate must
    preserve its original sequence_id.
    """

    if (
        isinstance(sequence_id, bool)
        or not isinstance(
            sequence_id,
            (int, np.integer),
        )
    ):
        raise TypeError(
            "sequence_id must be an integer."
        )

    sequence_id = int(sequence_id)

    if not 0 <= sequence_id <= (1 << 64) - 1:
        raise ValueError(
            "sequence_id must fit in uint64."
        )

    confidence = float(confidence)

    if not math.isfinite(confidence):
        raise ValueError(
            "confidence must be finite."
        )

    if not 0.0 <= confidence <= 1.0:
        raise ValueError(
            "confidence must be between 0 and 1."
        )

    transform = np.asarray(
        T_TS,
        dtype=float,
    ).reshape(4, 4)

    return struct.pack(
        TASK_POSE_V2_FORMAT,
        TASK_POSE_V2_MAGIC,
        TASK_POSE_V2_VERSION,
        int(bool(valid)),
        0,  # Reserved.
        sequence_id,
        confidence,
        *transform.reshape(-1),
    )


def unpack_task_pose_v2(
    data: bytes,
) -> TaskPosePacket:
    if len(data) != TASK_POSE_V2_SIZE:
        raise ValueError(
            "Expected "
            f"{TASK_POSE_V2_SIZE} bytes, "
            f"got {len(data)}."
        )

    unpacked = struct.unpack(
        TASK_POSE_V2_FORMAT,
        data,
    )

    (
        magic,
        version,
        valid_value,
        reserved,
        sequence_id,
        confidence,
        *matrix_values,
    ) = unpacked

    if magic != TASK_POSE_V2_MAGIC:
        raise ValueError(
            f"Invalid task-pose magic: {magic!r}."
        )

    if version != TASK_POSE_V2_VERSION:
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

    T_TS = np.asarray(
        matrix_values,
        dtype=float,
    ).reshape(4, 4)

    return TaskPosePacket(
        T_TS=T_TS,
        version=version,
        sequence_id=int(sequence_id),
        confidence=confidence,
        valid=bool(valid_value),
    )


def decode_task_pose_packet(
    data: bytes,
) -> TaskPosePacket:
    """
    Decode either the current legacy packet or version 2.

    Legacy packets remain supported during migration, but they do not
    provide source-level duplicate detection.
    """

    if len(data) == MATRIX_SIZE:
        return TaskPosePacket(
            T_TS=unpack_matrix4(data),
            version=TASK_POSE_LEGACY_VERSION,
            sequence_id=None,
            confidence=1.0,
            valid=True,
        )

    if len(data) == TASK_POSE_V2_SIZE:
        return unpack_task_pose_v2(data)

    raise ValueError(
        "Unsupported task-pose packet size: "
        f"{len(data)}."
    )
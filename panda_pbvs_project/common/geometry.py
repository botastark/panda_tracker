from __future__ import annotations

import math
import numpy as np


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float).reshape(3)
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])


def project_to_so3(rotation: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.asarray(rotation, dtype=float).reshape(3, 3))
    result = u @ vt
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vt
    return result


def so3_exp(rotation_vector: np.ndarray) -> np.ndarray:
    phi = np.asarray(rotation_vector, dtype=float).reshape(3)
    theta = np.linalg.norm(phi)
    if theta < 1e-10:
        return np.eye(3) + skew(phi)

    axis = phi / theta
    axis_hat = skew(axis)
    return (
        np.eye(3)
        + math.sin(theta) * axis_hat
        + (1.0 - math.cos(theta)) * (axis_hat @ axis_hat)
    )


def so3_log(rotation: np.ndarray) -> np.ndarray:
    rotation = project_to_so3(rotation)
    cos_theta = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    theta = math.acos(float(cos_theta))

    if theta < 1e-8:
        return 0.5 * np.array([
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ])

    factor = theta / (2.0 * math.sin(theta))
    return factor * np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ])
    ry = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ])
    rz = np.array([
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ])
    return rz @ ry @ rx


def rpy_from_rotation(rotation: np.ndarray) -> np.ndarray:
    rotation = project_to_so3(rotation)
    pitch = math.atan2(
        -rotation[2, 0],
        math.hypot(rotation[0, 0], rotation[1, 0]),
    )
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return np.array([roll, pitch, yaw])


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = project_to_so3(rotation)
    transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=float).reshape(4, 4)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def pose6_to_transform(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=float).reshape(6)
    return make_transform(
        rotation_from_rpy(pose[3], pose[4], pose[5]),
        pose[:3],
    )


def transform_to_pose6(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=float).reshape(4, 4)
    return np.concatenate([
        transform[:3, 3],
        rpy_from_rotation(transform[:3, :3]),
    ])


def clamp_norm(vector: np.ndarray, maximum_norm: float) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if norm <= maximum_norm or norm < 1e-12:
        return vector
    return vector * (maximum_norm / norm)

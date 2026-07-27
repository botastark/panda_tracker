from __future__ import annotations

import numpy as np


def finite_transform(transform: np.ndarray) -> bool:
    transform = np.asarray(transform, dtype=float)

    if transform.shape != (4, 4):
        return False

    if not np.all(np.isfinite(transform)):
        return False

    if not np.allclose(
        transform[3],
        [0.0, 0.0, 0.0, 1.0],
        atol=1e-9,
    ):
        return False

    rotation = transform[:3, :3]

    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3),
        atol=1e-6,
    ):
        return False

    return bool(
        np.isclose(
            np.linalg.det(rotation),
            1.0,
            atol=1e-6,
        )
    )


def clamp_workspace(
    transform: np.ndarray,
    minimum: np.ndarray,
    maximum: np.ndarray,
) -> np.ndarray:
    result = np.asarray(transform, dtype=float).copy()
    result[:3, 3] = np.clip(result[:3, 3], minimum, maximum)
    return result

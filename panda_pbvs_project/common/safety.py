from __future__ import annotations

import numpy as np


def finite_transform(transform: np.ndarray) -> bool:
    transform = np.asarray(transform)
    return (
        transform.shape == (4, 4)
        and np.all(np.isfinite(transform))
        and abs(transform[3, 3] - 1.0) < 1e-6
    )


def clamp_workspace(
    transform: np.ndarray,
    minimum: np.ndarray,
    maximum: np.ndarray,
) -> np.ndarray:
    result = np.asarray(transform, dtype=float).copy()
    result[:3, 3] = np.clip(result[:3, 3], minimum, maximum)
    return result

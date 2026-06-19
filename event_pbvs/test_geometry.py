#!/usr/bin/env python3
"""Offline geometry checks for event_pbvs_tracker.py."""

import math
import numpy as np

from event_pbvs_tracker import (
    invert_transform,
    make_transform,
    pose6_to_transform,
    rpy_zyx_to_rotation,
    so3_exp,
    so3_log,
    transform_to_pose6,
)


def main() -> None:
    pose = np.array([0.1, 0.5, 0.3, 0.2, -0.1, 0.4])
    transform = pose6_to_transform(pose)
    recovered = transform_to_pose6(transform)
    assert np.allclose(pose, recovered, atol=1e-9)

    identity = transform @ invert_transform(transform)
    assert np.allclose(identity, np.eye(4), atol=1e-9)

    phi = np.array([0.1, -0.2, 0.05])
    recovered_phi = so3_log(so3_exp(phi))
    assert np.allclose(phi, recovered_phi, atol=1e-9)

    rotation = rpy_zyx_to_rotation(0.2, -0.1, 0.4)
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
    assert math.isclose(np.linalg.det(rotation), 1.0, abs_tol=1e-9)

    tool = make_transform(np.eye(3), np.array([0.0, 0.0, 0.15]))
    assert np.allclose(tool[:3, 3], [0.0, 0.0, 0.15])

    print("All PBVS geometry checks passed.")


if __name__ == "__main__":
    main()

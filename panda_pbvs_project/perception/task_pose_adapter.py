from __future__ import annotations

import numpy as np

from common.geometry import invert_transform
from common.safety import finite_transform


def task_pose_from_camera_target(
    *,
    T_CT: np.ndarray,
    T_EC: np.ndarray,
    T_ES: np.ndarray,
) -> np.ndarray:
    """
    Convert an eye-in-hand target pose into the PBVS task pose.

    Frame convention:
        T_AB maps coordinates from frame B into frame A.

    Inputs:
        T_CT: target T expressed in camera C
        T_EC: camera C expressed in end-effector E
        T_ES: stick-tip S expressed in end-effector E

    Output:
        T_TS: stick-tip S expressed in target T
    """

    T_CT = np.asarray(T_CT, dtype=float)
    T_EC = np.asarray(T_EC, dtype=float)
    T_ES = np.asarray(T_ES, dtype=float)

    for name, transform in (
        ("T_CT", T_CT),
        ("T_EC", T_EC),
        ("T_ES", T_ES),
    ):
        if transform.shape != (4, 4):
            raise ValueError(
                f"{name} must have shape (4, 4), "
                f"got {transform.shape}."
            )

        if not finite_transform(transform):
            raise ValueError(
                f"{name} is not a finite rigid transform."
            )

    # Target expressed in the end-effector frame.
    T_ET = T_EC @ T_CT

    # Both T and S are now related through E:
    #
    #     T_TS = inverse(T_ET) @ T_ES
    #
    T_TS = invert_transform(T_ET) @ T_ES

    if not finite_transform(T_TS):
        raise ValueError(
            "Computed T_TS is not a finite rigid transform."
        )

    return T_TS
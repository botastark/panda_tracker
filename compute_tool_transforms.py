#!/usr/bin/env python3
"""Convert flange-referenced tool measurements into controller transforms.

Convention:
    T_XY = pose of frame Y expressed in frame X
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

The supplied tip and camera measurements must already be expressed in the
Panda flange frame F:
    T_FS: stick-tip frame S expressed in flange frame F
    T_FC: camera frame C expressed in flange frame F

The script uses the measured/printed Panda Hand transform:
    T_FE = F_T_EE

and computes:
    T_ES = inv(T_FE) @ T_FS
    T_EC = inv(T_FE) @ T_FC
    T_CS = inv(T_FC) @ T_FS
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def make_transform_mm_deg(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rotation_from_rpy(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
    )
    T[:3, 3] = np.array([x_mm, y_mm, z_mm], dtype=float) / 1000.0
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    p = T[:3, 3]
    result = np.eye(4)
    result[:3, :3] = R.T
    result[:3, 3] = -R.T @ p
    return result


def validate_transform(name: str, T: np.ndarray) -> None:
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError(f"{name} must be a finite 4x4 matrix.")
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{name} has an invalid bottom row.")
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-8):
        raise ValueError(f"{name} rotation is not orthonormal.")
    if not math.isclose(float(np.linalg.det(R)), 1.0, abs_tol=1e-8):
        raise ValueError(f"{name} rotation determinant is not +1.")


def matrix_list(T: np.ndarray) -> list[list[float]]:
    return [[float(value) for value in row] for row in T]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compute T_ES/T_EC/T_CS from flange-referenced measurements."
    )

    parser.add_argument("--fe-z-mm", type=float, default=103.4)
    parser.add_argument("--fe-yaw-deg", type=float, default=-45.0)

    parser.add_argument("--tip-x-mm", type=float, required=True)
    parser.add_argument("--tip-y-mm", type=float, required=True)
    parser.add_argument("--tip-z-mm", type=float, required=True)
    parser.add_argument("--tip-roll-deg", type=float, default=0.0)
    parser.add_argument("--tip-pitch-deg", type=float, default=0.0)
    parser.add_argument("--tip-yaw-deg", type=float, default=0.0)

    parser.add_argument("--camera-x-mm", type=float)
    parser.add_argument("--camera-y-mm", type=float)
    parser.add_argument("--camera-z-mm", type=float)
    parser.add_argument("--camera-roll-deg", type=float, default=0.0)
    parser.add_argument("--camera-pitch-deg", type=float, default=0.0)
    parser.add_argument("--camera-yaw-deg", type=float, default=0.0)

    parser.add_argument(
        "--output",
        type=Path,
        help="Optional JSON file for the generated transform fragment.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    T_FE = make_transform_mm_deg(
        0.0,
        0.0,
        args.fe_z_mm,
        0.0,
        0.0,
        args.fe_yaw_deg,
    )

    T_FS = make_transform_mm_deg(
        args.tip_x_mm,
        args.tip_y_mm,
        args.tip_z_mm,
        args.tip_roll_deg,
        args.tip_pitch_deg,
        args.tip_yaw_deg,
    )

    T_EF = invert_transform(T_FE)
    T_ES = T_EF @ T_FS

    validate_transform("T_FE", T_FE)
    validate_transform("T_FS", T_FS)
    validate_transform("T_ES", T_ES)

    output: dict[str, object] = {
        "T_FE_reference_only": matrix_list(T_FE),
        "T_FS_measurement_reference_only": matrix_list(T_FS),
        "T_ES": matrix_list(T_ES),
    }

    camera_position_values = (
        args.camera_x_mm,
        args.camera_y_mm,
        args.camera_z_mm,
    )
    camera_supplied = all(value is not None for value in camera_position_values)
    camera_partially_supplied = any(
        value is not None for value in camera_position_values
    ) and not camera_supplied

    if camera_partially_supplied:
        raise ValueError(
            "Provide all of --camera-x-mm, --camera-y-mm, and --camera-z-mm."
        )

    if camera_supplied:
        T_FC = make_transform_mm_deg(
            args.camera_x_mm,
            args.camera_y_mm,
            args.camera_z_mm,
            args.camera_roll_deg,
            args.camera_pitch_deg,
            args.camera_yaw_deg,
        )
        T_EC = T_EF @ T_FC
        T_CS = invert_transform(T_FC) @ T_FS

        validate_transform("T_FC", T_FC)
        validate_transform("T_EC", T_EC)
        validate_transform("T_CS", T_CS)

        composition_error = float(np.max(np.abs(T_EC @ T_CS - T_ES)))
        if composition_error > 1e-9:
            raise RuntimeError(
                f"T_EC @ T_CS consistency failed: {composition_error:.3e}"
            )

        output.update(
            {
                "T_FC_measurement_reference_only": matrix_list(T_FC),
                "T_EC": matrix_list(T_EC),
                "T_CS": matrix_list(T_CS),
                "composition_error_T_EC_T_CS_vs_T_ES": composition_error,
            }
        )

    text = json.dumps(output, indent=2)
    print(text)

    print("\nPhysical interpretation")
    print(
        "tip origin expressed in E [m] =",
        np.array2string(T_ES[:3, 3], precision=6),
    )
    print(
        "tip distance from E origin [m] =",
        f"{np.linalg.norm(T_ES[:3, 3]):.6f}",
    )

    if args.output is not None:
        args.output.write_text(text + "\n")
        print(f"\nWrote {args.output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

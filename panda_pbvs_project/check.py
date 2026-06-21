#!/usr/bin/env python3
"""Check the derived tool transform T_ES = T_EC @ T_CS.

Run from the repository root, for example:

    python3 check_t_es.py \
        --config panda_pbvs_project/configs/pbvs_robot.json
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

from common.config import load_pbvs_config


def validate_transform(name: str, transform: np.ndarray) -> None:
    matrix = np.asarray(transform, dtype=float)

    if matrix.shape != (4, 4):
        raise ValueError(f"{name}: expected shape (4, 4), got {matrix.shape}.")
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name}: contains non-finite values.")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{name}: invalid homogeneous bottom row.")

    rotation = matrix[:3, :3]
    orthogonality_error = np.linalg.norm(
        rotation.T @ rotation - np.eye(3),
        ord="fro",
    )
    determinant = float(np.linalg.det(rotation))

    if orthogonality_error > 1e-6:
        raise ValueError(
            f"{name}: rotation is not orthonormal; "
            f"error={orthogonality_error:.3e}."
        )
    if not math.isclose(determinant, 1.0, abs_tol=1e-6):
        raise ValueError(
            f"{name}: rotation determinant must be +1; got {determinant:.9f}."
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify T_ES derived from T_EC and T_CS."
    )
    parser.add_argument("--config", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_pbvs_config(args.config)

    missing = [
        name
        for name in ("T_EC", "T_CS", "T_TS_des", "T_ES")
        if not hasattr(config, name)
    ]
    if missing:
        print(
            "FAIL: configuration object is missing: " + ", ".join(missing),
            file=sys.stderr,
        )
        if "T_ES" in missing:
            print(
                "Add the T_ES property to PBVSConfig before running this check.",
                file=sys.stderr,
            )
        return 1

    T_EC = np.asarray(config.T_EC, dtype=float)
    T_CS = np.asarray(config.T_CS, dtype=float)
    T_ES = np.asarray(config.T_ES, dtype=float)
    expected_T_ES = T_EC @ T_CS

    try:
        validate_transform("T_EC", T_EC)
        validate_transform("T_CS", T_CS)
        validate_transform("T_ES", T_ES)
        validate_transform("T_TS_des", np.asarray(config.T_TS_des, dtype=float))
    except ValueError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

    composition_error = np.max(np.abs(T_ES - expected_T_ES))
    inverse_error = np.max(
        np.abs(T_ES @ np.linalg.inv(T_ES) - np.eye(4))
    )

    print("T_EC =")
    print(np.array2string(T_EC, precision=6, suppress_small=True))
    print("\nT_CS =")
    print(np.array2string(T_CS, precision=6, suppress_small=True))
    print("\nT_ES =")
    print(np.array2string(T_ES, precision=6, suppress_small=True))

    print("\nDerived checks")
    print(f"max|T_ES - T_EC @ T_CS| = {composition_error:.3e}")
    print(f"max|T_ES @ inv(T_ES) - I| = {inverse_error:.3e}")

    tip_position_E = T_ES[:3, 3]
    tip_distance_from_E = float(np.linalg.norm(tip_position_E))

    print("\nPhysical interpretation")
    print(
        "stick-tip origin expressed in E [m] =",
        np.array2string(tip_position_E, precision=6),
    )
    print(f"distance from E origin to stick tip = {tip_distance_from_E:.6f} m")
    print(
        "x_S axis expressed in E =",
        np.array2string(T_ES[:3, 0], precision=6),
    )
    print(
        "y_S axis expressed in E =",
        np.array2string(T_ES[:3, 1], precision=6),
    )
    print(
        "z_S axis expressed in E =",
        np.array2string(T_ES[:3, 2], precision=6),
    )

    if composition_error > 1e-12:
        print(
            "\nFAIL: config.T_ES does not exactly match T_EC @ T_CS.",
            file=sys.stderr,
        )
        return 1

    if inverse_error > 1e-9:
        print(
            "\nFAIL: T_ES inverse consistency check failed.",
            file=sys.stderr,
        )
        return 1

    print("\nPASS: T_ES is valid and equals T_EC @ T_CS.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

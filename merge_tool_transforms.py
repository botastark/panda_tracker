#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import shutil
from datetime import datetime
from pathlib import Path

import numpy as np


def validate_transform(name: str, value: object) -> np.ndarray:
    T = np.asarray(value, dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4; got {T.shape}.")
    if not np.all(np.isfinite(T)):
        raise ValueError(f"{name} contains non-finite values.")
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{name} has an invalid bottom row.")

    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6):
        raise ValueError(f"{name} rotation is not orthonormal.")
    if not math.isclose(float(np.linalg.det(R)), 1.0, abs_tol=1e-6):
        raise ValueError(f"{name} rotation determinant is not +1.")
    return T


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--tool-transforms", type=Path, required=True)
    p.add_argument("--pbvs-config", type=Path, required=True)
    p.add_argument("--check-only", action="store_true")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    tool_path = args.tool_transforms.expanduser().resolve()
    config_path = args.pbvs_config.expanduser().resolve()

    tool = json.loads(tool_path.read_text())
    config = json.loads(config_path.read_text())

    if "T_ES" not in tool:
        raise KeyError("tool_transforms.json does not contain T_ES.")

    transforms = {"T_ES": validate_transform("T_ES", tool["T_ES"])}
    for name in ("T_EC", "T_CS"):
        if name in tool:
            transforms[name] = validate_transform(name, tool[name])

    if "T_EC" in transforms and "T_CS" in transforms:
        error = float(np.max(np.abs(
            transforms["T_EC"] @ transforms["T_CS"] - transforms["T_ES"]
        )))
        if error > 1e-8:
            raise ValueError(
                f"T_EC @ T_CS != T_ES; max error={error:.3e}"
            )
        print(f"PASS: T_EC @ T_CS = T_ES; max error={error:.3e}")

    for name, T in transforms.items():
        print(f"\n{name} =")
        print(np.array2string(T, precision=8, suppress_small=True))

    if args.check_only:
        print("\nCheck-only mode: no file changed.")
        return 0

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = config_path.with_name(
        f"{config_path.name}.backup_{timestamp}"
    )
    shutil.copy2(config_path, backup)

    for name, T in transforms.items():
        config[name] = T.tolist()

    config_path.write_text(json.dumps(config, indent=2) + "\n")
    print(f"\nUpdated: {config_path}")
    print(f"Backup:  {backup}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

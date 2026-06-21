#!/usr/bin/env python3
"""Record a candidate Panda PBVS workspace while hand-guiding the robot.

The script listens directly to explorer_tracker's UDP state stream:
    little-endian <6f> = x, y, z, roll, pitch, yaw

It records:
    - EE origin E bounds in Panda base B
    - stick-tip origin S bounds in Panda base B, using configured T_ES
    - a candidate EE workspace shrunk inward by a configurable margin

It never sends robot commands and never edits the PBVS configuration.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import time
from pathlib import Path
from typing import Any

import numpy as np


POSE_FORMAT = "<6f"
POSE_SIZE = struct.calcsize(POSE_FORMAT)


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
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


def pose6_to_transform(pose: tuple[float, ...]) -> np.ndarray:
    x, y, z, roll, pitch, yaw = pose
    T = np.eye(4, dtype=float)
    T[:3, :3] = rotation_from_rpy(roll, pitch, yaw)
    T[:3, 3] = [x, y, z]
    return T


def validate_transform(name: str, value: Any) -> np.ndarray:
    T = np.asarray(value, dtype=float)

    if T.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4; got {T.shape}.")
    if not np.all(np.isfinite(T)):
        raise ValueError(f"{name} contains non-finite values.")
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{name} has an invalid homogeneous bottom row.")

    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6):
        raise ValueError(f"{name} rotation is not orthonormal.")
    if not math.isclose(float(np.linalg.det(R)), 1.0, abs_tol=1e-6):
        raise ValueError(f"{name} rotation determinant is not +1.")

    return T


def vector_list(value: np.ndarray) -> list[float]:
    return [float(x) for x in value]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record EE and attached stick-tip workspace bounds."
    )
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
        help="PBVS JSON containing calibrated T_ES.",
    )
    parser.add_argument(
        "--bind-ip",
        default="0.0.0.0",
        help="Local address receiving explorer_tracker state.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=6200,
        help="UDP state port. Use 6200 when listening directly to explorer_tracker.",
    )
    parser.add_argument(
        "--margin-mm",
        type=float,
        default=20.0,
        help="Inward margin applied to the observed EE bounds.",
    )
    parser.add_argument(
        "--print-hz",
        type=float,
        default=10.0,
        help="Terminal update rate.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("workspace_capture.json"),
        help="Output JSON report.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not 1 <= args.port <= 65535:
        raise ValueError("--port must be between 1 and 65535.")
    if args.margin_mm < 0.0:
        raise ValueError("--margin-mm must be non-negative.")
    if args.print_hz <= 0.0:
        raise ValueError("--print-hz must be positive.")

    raw = json.loads(args.config.read_text())
    T_ES = validate_transform("T_ES", raw["T_ES"])

    print("Configured T_ES:")
    print(np.array2string(T_ES, precision=6, suppress_small=True))
    print()
    print("This program sends no commands.")
    print("Stop run_control.py and every other UDP command sender first.")
    print("Guide the robot only through your established safe hand-guiding mode.")
    input("\nPress Enter when ready to begin recording...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind_ip, args.port))
    sock.settimeout(0.5)

    print(
        f"\nRecording Panda state from {args.bind_ip}:{args.port}"
        f" ({POSE_SIZE} bytes, {POSE_FORMAT})."
    )
    print("Move through the intended safe boundary slowly.")
    print("Press Ctrl-C to finish and save the report.\n")

    e_min = np.full(3, np.inf)
    e_max = np.full(3, -np.inf)
    s_min = np.full(3, np.inf)
    s_max = np.full(3, -np.inf)

    sample_count = 0
    rejected_count = 0
    first_time: float | None = None
    last_time: float | None = None
    last_print = 0.0
    latest_pose6: tuple[float, ...] | None = None
    latest_T_BE: np.ndarray | None = None
    latest_T_BS: np.ndarray | None = None

    try:
        while True:
            try:
                packet, source = sock.recvfrom(2048)
            except socket.timeout:
                print(
                    "\rWaiting for Panda UDP state..."
                    "                                              ",
                    end="",
                    flush=True,
                )
                continue

            if len(packet) != POSE_SIZE:
                rejected_count += 1
                continue

            pose6 = struct.unpack(POSE_FORMAT, packet)
            if not np.all(np.isfinite(pose6)):
                rejected_count += 1
                continue

            T_BE = pose6_to_transform(pose6)
            T_BS = T_BE @ T_ES

            p_E = T_BE[:3, 3]
            p_S = T_BS[:3, 3]

            e_min = np.minimum(e_min, p_E)
            e_max = np.maximum(e_max, p_E)
            s_min = np.minimum(s_min, p_S)
            s_max = np.maximum(s_max, p_S)

            now = time.monotonic()
            if first_time is None:
                first_time = now
            last_time = now
            sample_count += 1

            latest_pose6 = pose6
            latest_T_BE = T_BE
            latest_T_BS = T_BS

            if now - last_print >= 1.0 / args.print_hz:
                roll, pitch, yaw = pose6[3:]
                print(
                    "\r"
                    f"E xyz={np.array2string(p_E, precision=4)}  "
                    f"S xyz={np.array2string(p_S, precision=4)}  "
                    f"RPYdeg={np.array2string(np.degrees([roll, pitch, yaw]), precision=1)}  "
                    f"samples={sample_count}",
                    end="",
                    flush=True,
                )
                last_print = now

    except KeyboardInterrupt:
        print("\n\nRecording stopped.")

    finally:
        sock.close()

    if sample_count == 0 or latest_T_BE is None or latest_T_BS is None:
        raise RuntimeError("No valid Panda state packets were received.")

    margin = args.margin_mm / 1000.0
    candidate_min = e_min + margin
    candidate_max = e_max - margin
    span = e_max - e_min

    candidate_valid = bool(np.all(candidate_min < candidate_max))

    duration = (
        0.0
        if first_time is None or last_time is None
        else last_time - first_time
    )
    rate = sample_count / duration if duration > 0.0 else 0.0

    report: dict[str, Any] = {
        "source": {
            "bind_ip": args.bind_ip,
            "port": args.port,
            "packet_format": POSE_FORMAT,
        },
        "samples": {
            "accepted": sample_count,
            "rejected": rejected_count,
            "duration_s": duration,
            "average_rate_hz": rate,
        },
        "calibration": {
            "T_ES": T_ES.tolist(),
        },
        "observed_bounds_B": {
            "E_min": vector_list(e_min),
            "E_max": vector_list(e_max),
            "E_span": vector_list(span),
            "S_min": vector_list(s_min),
            "S_max": vector_list(s_max),
        },
        "candidate_workspace_E": {
            "margin_m": margin,
            "valid": candidate_valid,
            "min": vector_list(candidate_min),
            "max": vector_list(candidate_max),
        },
        "last_sample": {
            "pose6_BE": [float(x) for x in latest_pose6],
            "E_xyz_B": vector_list(latest_T_BE[:3, 3]),
            "S_xyz_B": vector_list(latest_T_BS[:3, 3]),
        },
        "warning": (
            "This is an axis-aligned translational envelope. It does not prove "
            "collision safety for the full holder/stick over arbitrary orientations."
        ),
    }

    args.output.write_text(json.dumps(report, indent=2) + "\n")

    print("\nObserved EE bounds in B:")
    print("  min:", np.array2string(e_min, precision=6))
    print("  max:", np.array2string(e_max, precision=6))
    print("  span:", np.array2string(span, precision=6))

    print("\nObserved stick-tip bounds in B:")
    print("  min:", np.array2string(s_min, precision=6))
    print("  max:", np.array2string(s_max, precision=6))

    print(f"\nCandidate EE workspace with {args.margin_mm:.1f} mm inward margin:")
    print("  min:", np.array2string(candidate_min, precision=6))
    print("  max:", np.array2string(candidate_max, precision=6))

    if candidate_valid:
        print("\nCandidate pbvs_robot.json fragment:")
        print(
            json.dumps(
                {
                    "workspace": {
                        "min": vector_list(candidate_min),
                        "max": vector_list(candidate_max),
                    }
                },
                indent=2,
            )
        )
    else:
        print(
            "\nWARNING: the observed span is too small for the selected margin "
            "on at least one axis. Capture a larger range or reduce --margin-mm."
        )

    print(f"\nSaved report: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

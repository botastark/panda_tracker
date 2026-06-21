#!/usr/bin/env python3
"""Convert live predicted T_TS into triangle pose T_BT for MuJoCo.

Inputs
------
1. Panda EE pose T_BE as little-endian <6f>:
       x_B, y_B, z_B, roll_B, pitch_B, yaw_B
2. Predicted task pose T_TS as little-endian <16d>, row-major 4x4.

Configured fixed transform
--------------------------
T_ES from the PBVS JSON.

Computation
-----------
T_BS = T_BE @ T_ES
T_BT = T_BS @ inv(T_TS)

Output
------
Triangle pose T_BT as little-endian <6f>, suitable for the existing
MuJoCo simulator triangle input.
"""

from __future__ import annotations

import argparse
import json
import math
import select
import socket
import struct
import time
from pathlib import Path

import numpy as np


POSE6_FORMAT = "<6f"
POSE6_SIZE = struct.calcsize(POSE6_FORMAT)
MATRIX_FORMAT = "<16d"
MATRIX_SIZE = struct.calcsize(MATRIX_FORMAT)


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


def pose6_to_transform(values: tuple[float, ...]) -> np.ndarray:
    x, y, z, roll, pitch, yaw = values
    T = np.eye(4, dtype=float)
    T[:3, :3] = rotation_from_rpy(roll, pitch, yaw)
    T[:3, 3] = [x, y, z]
    return T


def project_to_so3(rotation: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.asarray(rotation, dtype=float).reshape(3, 3))
    result = u @ vt
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vt
    return result


def rpy_from_rotation(rotation: np.ndarray) -> np.ndarray:
    R = project_to_so3(rotation)
    pitch = math.atan2(
        -R[2, 0],
        math.hypot(R[0, 0], R[1, 0]),
    )
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw], dtype=float)


def transform_to_pose6(T: np.ndarray) -> np.ndarray:
    return np.concatenate(
        [
            np.asarray(T[:3, 3], dtype=float),
            rpy_from_rotation(T[:3, :3]),
        ]
    )


def invert_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    p = T[:3, 3]
    result = np.eye(4, dtype=float)
    result[:3, :3] = R.T
    result[:3, 3] = -R.T @ p
    return result


def valid_transform(T: np.ndarray) -> bool:
    if T.shape != (4, 4):
        return False
    if not np.all(np.isfinite(T)):
        return False
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        return False

    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-4):
        return False
    if not math.isclose(float(np.linalg.det(R)), 1.0, abs_tol=1e-4):
        return False
    return True


def bind_udp(ip: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((ip, port))
    sock.setblocking(False)
    return sock


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize predicted T_TS as a triangle pose in MuJoCo."
    )
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
        help="PBVS JSON containing calibrated T_ES.",
    )
    parser.add_argument("--robot-bind-ip", default="127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=6203)
    parser.add_argument(
        "--task-bind-ip",
        default="0.0.0.0",
        help="Listen on all interfaces for the remote algorithm.",
    )
    parser.add_argument(
        "--task-port",
        type=int,
        default=6502,
        help="Port receiving predicted T_TS <16d>.",
    )
    parser.add_argument("--triangle-ip", default="127.0.0.1")
    parser.add_argument(
        "--triangle-port",
        type=int,
        default=6601,
        help="MuJoCo triangle-pose input port.",
    )
    parser.add_argument("--robot-timeout", type=float, default=0.2)
    parser.add_argument("--task-timeout", type=float, default=0.3)
    parser.add_argument("--rate", type=float, default=60.0)
    parser.add_argument("--status-period", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.rate <= 0.0:
        raise ValueError("--rate must be positive.")
    if args.robot_timeout <= 0.0 or args.task_timeout <= 0.0:
        raise ValueError("Timeouts must be positive.")

    raw = json.loads(args.config.read_text())
    T_ES = np.asarray(raw["T_ES"], dtype=float)
    if not valid_transform(T_ES):
        raise ValueError("Configured T_ES is not a valid rigid transform.")

    robot_socket = bind_udp(args.robot_bind_ip, args.robot_port)
    task_socket = bind_udp(args.task_bind_ip, args.task_port)
    output_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    triangle_destination = (args.triangle_ip, args.triangle_port)

    latest_T_BE: np.ndarray | None = None
    latest_T_TS: np.ndarray | None = None
    latest_robot_time = 0.0
    latest_task_time = 0.0

    rejected_robot = 0
    rejected_task = 0
    output_count = 0
    last_output_time = 0.0
    last_status_time = 0.0
    period = 1.0 / args.rate

    print(
        "Predicted-task visualization bridge\n"
        f"  Panda T_BE input: {args.robot_bind_ip}:{args.robot_port} "
        f"({POSE6_SIZE} bytes, {POSE6_FORMAT})\n"
        f"  predicted T_TS:   {args.task_bind_ip}:{args.task_port} "
        f"({MATRIX_SIZE} bytes, {MATRIX_FORMAT})\n"
        f"  MuJoCo T_BT out:  {args.triangle_ip}:{args.triangle_port} "
        f"({POSE6_SIZE} bytes, {POSE6_FORMAT})\n"
    )

    try:
        while True:
            readable, _, _ = select.select(
                [robot_socket, task_socket],
                [],
                [],
                0.02,
            )

            now = time.monotonic()

            for sock in readable:
                packet, source = sock.recvfrom(2048)

                if sock is robot_socket:
                    if len(packet) != POSE6_SIZE:
                        rejected_robot += 1
                        continue

                    values = struct.unpack(POSE6_FORMAT, packet)
                    if not np.all(np.isfinite(values)):
                        rejected_robot += 1
                        continue

                    latest_T_BE = pose6_to_transform(values)
                    latest_robot_time = now

                else:
                    if len(packet) != MATRIX_SIZE:
                        rejected_task += 1
                        continue

                    values = struct.unpack(MATRIX_FORMAT, packet)
                    T_TS = np.asarray(values, dtype=float).reshape(4, 4)

                    if not valid_transform(T_TS):
                        rejected_task += 1
                        continue

                    latest_T_TS = T_TS
                    latest_task_time = now

            if now - last_output_time < period:
                continue

            robot_fresh = (
                latest_T_BE is not None
                and now - latest_robot_time <= args.robot_timeout
            )
            task_fresh = (
                latest_T_TS is not None
                and now - latest_task_time <= args.task_timeout
            )

            if robot_fresh and task_fresh:
                T_BS = latest_T_BE @ T_ES
                T_BT = T_BS @ invert_transform(latest_T_TS)

                pose_BT = transform_to_pose6(T_BT)
                output_socket.sendto(
                    struct.pack(
                        POSE6_FORMAT,
                        *pose_BT.astype(np.float32),
                    ),
                    triangle_destination,
                )
                output_count += 1
                last_output_time = now

                if now - last_status_time >= args.status_period:
                    print(
                        "triangle_xyz_B=",
                        np.array2string(T_BT[:3, 3], precision=5),
                        "task_xyz_T=",
                        np.array2string(latest_T_TS[:3, 3], precision=5),
                        f"out={output_count}",
                        f"rejected(robot/task)="
                        f"{rejected_robot}/{rejected_task}",
                    )
                    last_status_time = now
            elif now - last_status_time >= args.status_period:
                robot_age = (
                    float("inf")
                    if latest_T_BE is None
                    else now - latest_robot_time
                )
                task_age = (
                    float("inf")
                    if latest_T_TS is None
                    else now - latest_task_time
                )
                print(
                    "waiting:",
                    f"robot_age={robot_age:.3f}s",
                    f"task_age={task_age:.3f}s",
                    f"rejected(robot/task)="
                    f"{rejected_robot}/{rejected_task}",
                )
                last_status_time = now

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        robot_socket.close()
        task_socket.close()
        output_socket.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

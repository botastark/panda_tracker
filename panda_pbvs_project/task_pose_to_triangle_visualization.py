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
import select
import socket
import time
from pathlib import Path

import numpy as np
from common.geometry import (
    invert_transform,
    pose6_to_transform,
    transform_to_pose6,
)
from common.protocol import (
    POSE_FORMAT,
    POSE_SIZE,
    MATRIX_FORMAT,
    MATRIX_SIZE,
    pack_pose6,
    unpack_pose6,
    pack_task_pose,
    unpack_task_pose

)
from common.safety import finite_transform


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
        f"({POSE_SIZE} bytes, {POSE_SIZE})\n"
        f"  predicted T_TS:   {args.task_bind_ip}:{args.task_port} "
        f"({MATRIX_SIZE} bytes, {MATRIX_FORMAT})\n"
        f"  MuJoCo T_BT out:  {args.triangle_ip}:{args.triangle_port} "
        f"({POSE_SIZE} bytes, {POSE_SIZE})\n"
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
                    if len(packet) != POSE_SIZE:
                        rejected_robot += 1
                        continue

                    values = unpack_pose6(packet)
                    if not np.all(np.isfinite(values)):
                        rejected_robot += 1
                        continue

                    latest_T_BE = pose6_to_transform(values)
                    latest_robot_time = now

                else:
                    if len(packet) != MATRIX_SIZE:
                        rejected_task += 1
                        continue

                    T_TS = unpack_task_pose(packet)

                    if not finite_transform(T_TS):
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
                    pack_pose6(pose_BT),
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

#!/usr/bin/env python3
"""Publish the measured stick-tip pose T_TS over UDP.

Frame convention
----------------
T_XY is the pose of frame Y expressed in frame X and maps coordinates
from frame Y into frame X.

The vision input pose is:

    pose6 = [
        x_T_S, y_T_S, z_T_S,
        roll_T_S, pitch_T_S, yaw_T_S,
    ]

where:
    - x_T_S, y_T_S, z_T_S are the coordinates of the stick-tip frame S
      origin expressed along the triangle-frame T axes, in metres.
    - roll_T_S, pitch_T_S, yaw_T_S describe the orientation R_TS of the
      stick-tip frame S relative to the triangle frame T, in radians.
    - R_TS = Rz(yaw) @ Ry(pitch) @ Rx(roll).

Wire format
-----------
The published packet is a row-major 4x4 homogeneous transform T_TS:

    little-endian <16d
    16 IEEE-754 float64 values
    exactly 128 bytes

This is the canonical direct-controller task-pose transport.
"""

from __future__ import annotations

import argparse
import signal
import socket
import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from panda_pbvs_project.common.geometry import pose6_to_transform
from panda_pbvs_project.common.protocol import (
    MATRIX_FORMAT,
    MATRIX_SIZE,
    pack_task_pose,
)
from panda_pbvs_project.common.safety import finite_transform


@dataclass
class TaskPosePublisher:
    destination_ip: str = "127.0.0.1"
    destination_port: int = 6501

    def __post_init__(self) -> None:
        if not 1 <= self.destination_port <= 65535:
            raise ValueError("destination_port must be between 1 and 65535.")
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._destination = (self.destination_ip, self.destination_port)

    def publish_matrix(self, T_TS: np.ndarray) -> None:
        """Publish one validated 4x4 T_TS matrix."""
        if not finite_transform(T_TS):
            raise ValueError("T_TS is not a valid homogeneous transform.")

        self._socket.sendto(
            pack_task_pose(T_TS),
            self._destination,
        )

    def publish_pose6(self, pose6: Sequence[float] | np.ndarray) -> np.ndarray:
        """Convert a six-value vision pose to T_TS, publish it, and return it."""
        T_TS = pose6_to_transform(pose6)
        self.publish_matrix(T_TS)
        return T_TS

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> "TaskPosePublisher":
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish a constant test T_TS task pose over UDP."
    )
    parser.add_argument("--x", type=float, required=True, help="x_T_S [m]")
    parser.add_argument("--y", type=float, required=True, help="y_T_S [m]")
    parser.add_argument("--z", type=float, required=True, help="z_T_S [m]")
    parser.add_argument("--roll", type=float, default=0.0, help="roll_T_S [rad]")
    parser.add_argument("--pitch", type=float, default=0.0, help="pitch_T_S [rad]")
    parser.add_argument("--yaw", type=float, default=0.0, help="yaw_T_S [rad]")
    parser.add_argument("--rate", type=float, default=30.0, help="Publish rate [Hz]")
    parser.add_argument("--destination-ip", default="127.0.0.1")
    parser.add_argument("--destination-port", type=int, default=6501)
    parser.add_argument(
        "--once",
        action="store_true",
        help="Publish one packet and exit instead of streaming.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.rate <= 0.0:
        raise ValueError("--rate must be positive.")

    pose6 = np.array(
        [args.x, args.y, args.z, args.roll, args.pitch, args.yaw],
        dtype=np.float64,
    )
    T_TS = pose6_to_transform(pose6)

    print("pose6 = [x_T_S, y_T_S, z_T_S, roll_T_S, pitch_T_S, yaw_T_S]")
    print("pose6 =", np.array2string(pose6, precision=6))
    print("T_TS =\n", np.array2string(T_TS, precision=6, suppress_small=True))
    print(
        f"Packet: {MATRIX_SIZE} bytes ({MATRIX_FORMAT}), "
        f"destination={args.destination_ip}:{args.destination_port}"
    )

    running = True

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    with TaskPosePublisher(
        destination_ip=args.destination_ip,
        destination_port=args.destination_port,
    ) as publisher:
        if args.once:
            publisher.publish_matrix(T_TS)
            return 0

        period = 1.0 / args.rate
        next_send = time.monotonic()
        sent = 0
        last_status = next_send

        while running:
            now = time.monotonic()
            if now < next_send:
                time.sleep(next_send - now)
                continue

            next_send += period
            if next_send < now - period:
                next_send = now + period

            publisher.publish_matrix(T_TS)
            sent += 1

            if now - last_status >= 1.0:
                print(f"Published {sent} T_TS packets.")
                last_status = now

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

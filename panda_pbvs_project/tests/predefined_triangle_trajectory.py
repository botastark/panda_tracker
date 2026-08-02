#!/usr/bin/env python3
"""Publish a repeatable, smooth triangle trajectory for PBVS testing."""

from __future__ import annotations

import argparse
import math
import signal
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from common.protocol import pack_pose6


@dataclass(frozen=True)
class Waypoint:
    name: str
    duration: float
    pose: np.ndarray


def pose(
    x: float,
    y: float,
    z: float,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
) -> np.ndarray:
    return np.array(
        [
            x,
            y,
            z,
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        ],
        dtype=float,
    )


DEFAULT_CENTER_POSE = pose(
    0.50,
    0.00,
    0.40,
)


def build_waypoints(
    center: np.ndarray,
) -> tuple[Waypoint, ...]:
    """
    Build the predefined trajectory relative to an equilibrium pose.

    Translation offsets are expressed in the Panda base frame. Rotation
    offsets are added to the equilibrium roll, pitch, and yaw values.
    """

    center = np.asarray(center, dtype=float).reshape(6)

    def offset(
        *,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        droll_deg: float = 0.0,
        dpitch_deg: float = 0.0,
        dyaw_deg: float = 0.0,
    ) -> np.ndarray:
        result = center.copy()

        result[:3] += np.array(
            [dx, dy, dz],
            dtype=float,
        )

        result[3:] += np.radians(
            [
                droll_deg,
                dpitch_deg,
                dyaw_deg,
            ]
        )

        return result

    return (
        Waypoint(
            "center",
            3.0,
            center.copy(),
        ),
        Waypoint(
            "move_positive_x",
            3.0,
            offset(dx=0.04),
        ),
        Waypoint(
            "move_positive_y",
            3.0,
            offset(dx=0.04, dy=0.04),
        ),
        Waypoint(
            "move_positive_z",
            3.0,
            offset(dx=0.04, dy=0.04, dz=0.04),
        ),
        Waypoint(
            "rotate_yaw",
            3.0,
            offset(
                dx=0.04,
                dy=0.04,
                dz=0.04,
                dyaw_deg=10.0,
            ),
        ),
        Waypoint(
            "rotate_pitch",
            3.0,
            offset(
                dx=0.04,
                dy=0.04,
                dz=0.04,
                dpitch_deg=-8.0,
                dyaw_deg=10.0,
            ),
        ),
        Waypoint(
            "return_center",
            5.0,
            center.copy(),
        ),
    )


def parse_destination(value: str) -> tuple[str, int]:
    host, separator, port_text = value.rpartition(":")

    if not separator or not host:
        raise argparse.ArgumentTypeError(
            f"Invalid destination {value!r}; expected HOST:PORT."
        )

    try:
        port = int(port_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"Invalid destination port in {value!r}."
        ) from exc

    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError(
            "Port must be between 1 and 65535."
        )

    return host, port


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish the predefined PBVS triangle trajectory."
    )
    parser.add_argument(
        "--destination",
        type=parse_destination,
        default=("127.0.0.1", 6601),
        help="Triangle destination HOST:PORT. Default: 127.0.0.1:6601.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="Publishing rate in packets/s.",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Number of complete trajectory repetitions.",
    )
    parser.add_argument(
        "--initial-hold",
        type=float,
        default=15.0,
        help=(
            "Seconds to hold the first triangle pose before beginning "
            "the moving trajectory."
        ),
    )

    parser.add_argument(
        "--center-pose",
        nargs=6,
        type=float,
        default=[
            0.50,
            0.00,
            0.40,
            0.00,
            0.00,
            0.00,
        ],
        metavar=(
            "X",
            "Y",
            "Z",
            "ROLL_DEG",
            "PITCH_DEG",
            "YAW_DEG",
        ),
        help=(
            "Equilibrium triangle pose in the Panda base frame. "
            "Position is in metres and orientation is in degrees."
        ),
    )
    return parser.parse_args()


def smoothstep(value: float) -> float:
    value = min(max(value, 0.0), 1.0)
    return value * value * (3.0 - 2.0 * value)


def publish_segment(
    sock: socket.socket,
    destination: tuple[str, int],
    start: np.ndarray,
    target: np.ndarray,
    duration: float,
    rate: float,
    running: list[bool],
) -> None:
    period = 1.0 / rate
    start_time = time.monotonic()
    next_send = start_time

    while running[0]:
        now = time.monotonic()
        elapsed = now - start_time
        fraction = min(elapsed / duration, 1.0)
        alpha = smoothstep(fraction)
        current = start + alpha * (target - start)

        sock.sendto(
            pack_pose6(current),
            destination,
        )

        if fraction >= 1.0:
            return

        next_send += period
        time.sleep(max(0.0, next_send - time.monotonic()))


def main() -> int:
    args = parse_args()
    if args.initial_hold <= 0.0:
        raise ValueError("--initial-hold must be positive.")

    if args.rate <= 0.0:
        raise ValueError("--rate must be positive.")

    if args.repeat < 1:
        raise ValueError("--repeat must be at least 1.")
    center = pose(*args.center_pose)
    waypoints = build_waypoints(center)
    running = [True]

    def request_stop(_signum: int, _frame: object) -> None:
        running[0] = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    current = waypoints[0].pose.copy()

    print(
        f"Publishing to {args.destination[0]}:{args.destination[1]} "
        f"at {args.rate:.1f} Hz"
    )

    try:
        for repetition in range(args.repeat):
            print(f"Trajectory repetition {repetition + 1}/{args.repeat}")

            for waypoint_index, waypoint in enumerate(waypoints):
                if not running[0]:
                    return 0

                print(f"Starting: {waypoint.name}")
                segment_duration = (
                    args.initial_hold
                    if repetition == 0 and waypoint_index == 0
                    else waypoint.duration
                )

                publish_segment(
                    sock=sock,
                    destination=args.destination,
                    start=current,
                    target=waypoint.pose,
                    duration=segment_duration,
                    rate=args.rate,
                    running=running,
                )

                current = waypoint.pose.copy()

                print(
                    f"Reached: {waypoint.name}; "
                    f"xyz={np.round(current[:3], 3)}, "
                    f"rpy_deg={np.round(np.degrees(current[3:]), 1)}"
                )

        print("Predefined triangle trajectory: COMPLETE")
    finally:
        sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
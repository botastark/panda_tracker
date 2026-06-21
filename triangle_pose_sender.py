#!/usr/bin/env python3
"""Stream a triangle pose to one or more UDP consumers.

Packet format, little-endian <6f> (24 bytes):
    x_B, y_B, z_B, roll_B, pitch_B, yaw_B

Position is in metres and orientation is in radians. The pose is the triangle
frame T expressed in the Panda base frame B.

By default each packet is sent to:
    127.0.0.1:6601  MuJoCo simulator
    127.0.0.1:6602  run_robot / run_control

Interactive commands while streaming:
    set X Y Z R P Y       absolute pose; angles in degrees
    move DX DY DZ         translate in metres
    rotate DR DP DY       rotate in degrees
    show                   print current pose
    quit                   stop
"""

from __future__ import annotations

import argparse
import math
import signal
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Sequence


POSE_FORMAT = "<6f"
POSE_SIZE = struct.calcsize(POSE_FORMAT)


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
        raise argparse.ArgumentTypeError("Port must be between 1 and 65535.")
    return host, port


@dataclass
class TrianglePose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def packet(self) -> bytes:
        values = (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Triangle pose contains a non-finite value.")
        return struct.pack(POSE_FORMAT, *values)

    def text(self) -> str:
        return (
            f"xyz_B=[{self.x:.4f}, {self.y:.4f}, {self.z:.4f}] m, "
            "rpy_B=["
            f"{math.degrees(self.roll):.2f}, "
            f"{math.degrees(self.pitch):.2f}, "
            f"{math.degrees(self.yaw):.2f}] deg"
        )


class SharedPose:
    def __init__(self, pose: TrianglePose) -> None:
        self._pose = pose
        self._lock = threading.Lock()

    def get(self) -> TrianglePose:
        with self._lock:
            return TrianglePose(**vars(self._pose))

    def set_absolute(self, values: Sequence[float]) -> None:
        x, y, z, roll_deg, pitch_deg, yaw_deg = values
        with self._lock:
            self._pose = TrianglePose(
                x=x,
                y=y,
                z=z,
                roll=math.radians(roll_deg),
                pitch=math.radians(pitch_deg),
                yaw=math.radians(yaw_deg),
            )

    def move(self, values: Sequence[float]) -> None:
        dx, dy, dz = values
        with self._lock:
            self._pose.x += dx
            self._pose.y += dy
            self._pose.z += dz

    def rotate(self, values: Sequence[float]) -> None:
        droll_deg, dpitch_deg, dyaw_deg = values
        with self._lock:
            self._pose.roll += math.radians(droll_deg)
            self._pose.pitch += math.radians(dpitch_deg)
            self._pose.yaw += math.radians(dyaw_deg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream a triangle pose in the Panda base frame over UDP."
    )
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=30.0, help="Packets/s.")
    parser.add_argument(
        "--destination",
        action="append",
        type=parse_destination,
        dest="destinations",
        help=(
            "Destination HOST:PORT; may be repeated. Defaults to "
            "127.0.0.1:6601 and 127.0.0.1:6602."
        ),
    )
    parser.add_argument(
        "--no-interactive",
        action="store_true",
        help="Disable stdin commands and only stream the initial pose.",
    )
    return parser.parse_args()


def command_loop(shared_pose: SharedPose, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            line = input().strip()
        except (EOFError, KeyboardInterrupt):
            stop_event.set()
            return

        if not line:
            continue

        parts = line.split()
        command = parts[0].lower()

        try:
            values = [float(value) for value in parts[1:]]
            if command == "set" and len(values) == 6:
                shared_pose.set_absolute(values)
            elif command == "move" and len(values) == 3:
                shared_pose.move(values)
            elif command == "rotate" and len(values) == 3:
                shared_pose.rotate(values)
            elif command == "show" and len(values) == 0:
                pass
            elif command in {"quit", "exit", "q"} and len(values) == 0:
                stop_event.set()
                return
            else:
                print(
                    "Commands: set X Y Z R P Y | move DX DY DZ | "
                    "rotate DR DP DY | show | quit"
                )
                continue
        except ValueError as exc:
            print(f"Invalid command: {exc}")
            continue

        print(shared_pose.get().text())


def main() -> int:
    args = parse_args()
    if args.rate <= 0.0:
        raise ValueError("--rate must be positive.")

    destinations = (
        args.destinations
        if args.destinations
        else [("127.0.0.1", 6601), ("127.0.0.1", 6602)]
    )

    shared_pose = SharedPose(
        TrianglePose(
            x=args.x,
            y=args.y,
            z=args.z,
            roll=math.radians(args.roll_deg),
            pitch=math.radians(args.pitch_deg),
            yaw=math.radians(args.yaw_deg),
        )
    )

    stop_event = threading.Event()

    def stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    if not args.no_interactive:
        threading.Thread(
            target=command_loop,
            args=(shared_pose, stop_event),
            daemon=True,
        ).start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.rate
    next_send = time.monotonic()
    sent = 0
    last_status = next_send

    print(f"Triangle packet: {POSE_SIZE} bytes ({POSE_FORMAT})")
    print("Destinations: " + ", ".join(f"{h}:{p}" for h, p in destinations))
    print(shared_pose.get().text())

    try:
        while not stop_event.is_set():
            now = time.monotonic()
            if now < next_send:
                stop_event.wait(next_send - now)
                continue

            packet = shared_pose.get().packet()
            for destination in destinations:
                sock.sendto(packet, destination)
            sent += 1

            if now - last_status >= 1.0:
                print(f"streaming {sent} poses; {shared_pose.get().text()}")
                last_status = now

            next_send += period
            if next_send < now - period:
                next_send = now + period
    finally:
        sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

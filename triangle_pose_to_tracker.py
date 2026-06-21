#!/usr/bin/env python3
"""Compute tracker pose T_TC from physical Panda and triangle UDP streams.

Inputs:
    robot state port (default 6203), <6f>
        x_B_E, y_B_E, z_B_E, roll_B_E, pitch_B_E, yaw_B_E

    triangle port (default 6602), <6f>
        x_B_T, y_B_T, z_B_T, roll_B_T, pitch_B_T, yaw_B_T

Output:
    tracker port (default 6501), <16d>
        row-major 4x4 transform T_TC = inv(T_BT) @ T_BE @ T_EC

The PBVS controller can keep its existing tracker receiver unchanged. This
bridge removes MuJoCo from the control path; MuJoCo may run visualization-only.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import socket
import struct
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np


POSE_FORMAT = "<6f"
POSE_SIZE = struct.calcsize(POSE_FORMAT)
TRACKER_FORMAT = "<16d"
TRACKER_SIZE = struct.calcsize(TRACKER_FORMAT)


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def pose6_to_transform(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=float).reshape(6)
    if not np.all(np.isfinite(pose)):
        raise ValueError("Pose contains non-finite values.")
    transform = np.eye(4)
    transform[:3, :3] = rotation_from_rpy(*pose[3:])
    transform[:3, 3] = pose[:3]
    return transform


def validate_transform(name: str, value: object) -> np.ndarray:
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4, got {transform.shape}.")
    if not np.all(np.isfinite(transform)):
        raise ValueError(f"{name} contains non-finite values.")
    return transform


class LatestPose:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose: Optional[np.ndarray] = None
        self._time = 0.0
        self._packets = 0

    def set(self, pose: np.ndarray) -> None:
        with self._lock:
            self._pose = pose.copy()
            self._time = time.monotonic()
            self._packets += 1

    def get(self) -> tuple[Optional[np.ndarray], float, int]:
        with self._lock:
            return (
                None if self._pose is None else self._pose.copy(),
                self._time,
                self._packets,
            )


class PoseReceiver(threading.Thread):
    def __init__(
        self,
        name: str,
        bind_ip: str,
        port: int,
        latest: LatestPose,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(daemon=True)
        self.name = name
        self.bind_ip = bind_ip
        self.port = port
        self.latest = latest
        self.stop_event = stop_event

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.bind_ip, self.port))
        sock.settimeout(0.2)
        print(f"Listening for {self.name} on {self.bind_ip}:{self.port}")

        try:
            while not self.stop_event.is_set():
                try:
                    packet, source = sock.recvfrom(2048)
                except socket.timeout:
                    continue

                if len(packet) != POSE_SIZE:
                    print(
                        f"Ignoring {len(packet)}-byte {self.name} packet from "
                        f"{source[0]}:{source[1]}; expected {POSE_SIZE}."
                    )
                    continue

                pose = np.asarray(struct.unpack(POSE_FORMAT, packet), dtype=float)
                if not np.all(np.isfinite(pose)):
                    print(f"Ignoring non-finite {self.name} packet.")
                    continue

                self.latest.set(pose)
        finally:
            sock.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert Panda + triangle poses into tracker T_TC packets."
    )
    parser.add_argument(
        "--pbvs-config",
        type=Path,
        required=True,
        help="JSON configuration containing T_EC.",
    )
    parser.add_argument("--robot-bind-ip", default="127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=6203)
    parser.add_argument("--triangle-bind-ip", default="127.0.0.1")
    parser.add_argument("--triangle-port", type=int, default=6602)
    parser.add_argument("--tracker-ip", default="127.0.0.1")
    parser.add_argument("--tracker-port", type=int, default=6501)
    parser.add_argument("--robot-timeout", type=float, default=0.25)
    parser.add_argument("--triangle-timeout", type=float, default=0.5)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--status-period", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.rate <= 0.0:
        raise ValueError("--rate must be positive.")
    if args.robot_timeout <= 0.0 or args.triangle_timeout <= 0.0:
        raise ValueError("Timeouts must be positive.")

    config = json.loads(args.pbvs_config.expanduser().read_text())
    T_EC = validate_transform("T_EC", config["T_EC"])

    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    latest_robot = LatestPose()
    latest_triangle = LatestPose()

    PoseReceiver(
        "physical Panda pose",
        args.robot_bind_ip,
        args.robot_port,
        latest_robot,
        stop_event,
    ).start()
    PoseReceiver(
        "triangle pose",
        args.triangle_bind_ip,
        args.triangle_port,
        latest_triangle,
        stop_event,
    ).start()

    output_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    output_destination = (args.tracker_ip, args.tracker_port)

    print(
        f"Publishing T_TC to {args.tracker_ip}:{args.tracker_port} "
        f"at up to {args.rate:.1f} Hz"
    )

    period = 1.0 / args.rate
    next_send = time.monotonic()
    last_status = next_send
    sent = 0

    try:
        while not stop_event.is_set():
            now = time.monotonic()
            if now < next_send:
                stop_event.wait(next_send - now)
                continue
            next_send += period
            if next_send < now - period:
                next_send = now + period

            robot_pose, robot_time, robot_packets = latest_robot.get()
            triangle_pose, triangle_time, triangle_packets = latest_triangle.get()

            robot_fresh = (
                robot_pose is not None
                and now - robot_time <= args.robot_timeout
            )
            triangle_fresh = (
                triangle_pose is not None
                and now - triangle_time <= args.triangle_timeout
            )

            if robot_fresh and triangle_fresh:
                T_BE = pose6_to_transform(robot_pose)
                T_BT = pose6_to_transform(triangle_pose)
                T_BC = T_BE @ T_EC
                T_TC = np.linalg.inv(T_BT) @ T_BC

                packet = struct.pack(TRACKER_FORMAT, *T_TC.reshape(-1))
                if len(packet) != TRACKER_SIZE:
                    raise RuntimeError("Unexpected tracker packet size.")
                output_socket.sendto(packet, output_destination)
                sent += 1

            if args.status_period > 0.0 and now - last_status >= args.status_period:
                state = (
                    "ready"
                    if robot_fresh and triangle_fresh
                    else (
                        f"waiting: robot={'fresh' if robot_fresh else 'stale'}, "
                        f"triangle={'fresh' if triangle_fresh else 'stale'}"
                    )
                )
                message = (
                    f"{state}; robot_packets={robot_packets}, "
                    f"triangle_packets={triangle_packets}, tracker_sent={sent}"
                )
                if robot_fresh and triangle_fresh:
                    tip_distance = float(
                        np.linalg.norm(T_BT[:3, 3] - T_BE[:3, 3])
                    )
                    camera_distance = float(np.linalg.norm(T_TC[:3, 3]))
                    message += (
                        f", EE_triangle_distance={tip_distance:.4f} m, "
                        f"camera_triangle_distance={camera_distance:.4f} m"
                    )
                print(message)
                last_status = now
    finally:
        output_socket.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

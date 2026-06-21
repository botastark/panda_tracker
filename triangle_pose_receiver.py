#!/usr/bin/env python3
"""Reusable UDP triangle-pose receiver for run_robot/run_control.

Packet format is little-endian <6f>:
    x_B, y_B, z_B, roll_B, pitch_B, yaw_B
where position is in metres and angles are in radians.
"""

from __future__ import annotations

import math
import socket
import struct
import threading
import time
from typing import Optional

import numpy as np


TRIANGLE_POSE_FORMAT = "<6f"
TRIANGLE_POSE_SIZE = struct.calcsize(TRIANGLE_POSE_FORMAT)


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
        raise ValueError("Triangle pose contains non-finite values.")
    transform = np.eye(4)
    transform[:3, :3] = rotation_from_rpy(*pose[3:])
    transform[:3, 3] = pose[:3]
    return transform


class TrianglePoseReceiver(threading.Thread):
    def __init__(self, bind_ip: str = "127.0.0.1", port: int = 6602) -> None:
        super().__init__(daemon=True)
        self.bind_ip = bind_ip
        self.port = port
        self._lock = threading.Lock()
        self._pose: Optional[np.ndarray] = None
        self._received_time = 0.0
        self._stop_event = threading.Event()
        self._socket: Optional[socket.socket] = None

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket = sock
        sock.bind((self.bind_ip, self.port))
        sock.settimeout(0.2)

        while not self._stop_event.is_set():
            try:
                packet, _source = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(packet) != TRIANGLE_POSE_SIZE:
                continue

            pose = np.asarray(
                struct.unpack(TRIANGLE_POSE_FORMAT, packet),
                dtype=float,
            )
            if not np.all(np.isfinite(pose)):
                continue

            with self._lock:
                self._pose = pose
                self._received_time = time.monotonic()

        sock.close()

    def stop(self) -> None:
        self._stop_event.set()
        if self._socket is not None:
            self._socket.close()

    def latest(self) -> tuple[Optional[np.ndarray], float]:
        with self._lock:
            return (
                None if self._pose is None else self._pose.copy(),
                self._received_time,
            )

    def fresh_transform(
        self,
        timeout: float = 0.5,
    ) -> tuple[Optional[np.ndarray], bool]:
        pose, received_time = self.latest()
        fresh = (
            pose is not None
            and time.monotonic() - received_time <= timeout
        )
        if not fresh:
            return None, False
        return pose6_to_transform(pose), True

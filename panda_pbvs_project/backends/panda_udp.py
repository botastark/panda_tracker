from __future__ import annotations

import socket
import threading
import time
import numpy as np

from common.geometry import pose6_to_transform, transform_to_pose6
from common.protocol import POSE_SIZE, pack_pose6, unpack_pose6


class PandaUdpBackend:
    def __init__(
        self,
        panda_ip: str,
        command_port: int,
        state_bind_ip: str,
        state_port: int,
    ) -> None:
        self.destination = (panda_ip, command_port)
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._lock = threading.Lock()
        self._latest_pose: np.ndarray | None = None
        self._latest_time = 0.0
        self._running = True

        self._receiver = threading.Thread(
            target=self._receive_loop,
            args=(state_bind_ip, state_port),
            daemon=True,
        )
        self._receiver.start()

    def _receive_loop(self, bind_ip: str, port: int) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((bind_ip, port))
        sock.settimeout(0.1)

        try:
            while self._running:
                try:
                    data, _ = sock.recvfrom(1024)
                except socket.timeout:
                    continue

                if len(data) != POSE_SIZE:
                    continue

                pose = unpack_pose6(data)
                if not np.all(np.isfinite(pose)):
                    continue

                with self._lock:
                    self._latest_pose = pose6_to_transform(pose)
                    self._latest_time = time.monotonic()
        finally:
            sock.close()

    def get_current_pose(self) -> tuple[np.ndarray | None, float]:
        with self._lock:
            pose = None if self._latest_pose is None else self._latest_pose.copy()
            timestamp = self._latest_time

        age = float("inf") if pose is None else time.monotonic() - timestamp
        return pose, age

    def send_target_pose(self, T_BE_target: np.ndarray) -> None:
        self.command_socket.sendto(
            pack_pose6(transform_to_pose6(T_BE_target)),
            self.destination,
        )

    def healthy(self) -> bool:
        pose, age = self.get_current_pose()
        return pose is not None and age < 0.5

    def close(self) -> None:
        self._running = False
        self.command_socket.close()

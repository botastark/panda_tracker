from __future__ import annotations

import socket
import threading
import time
import numpy as np

from common.protocol import MATRIX_SIZE, unpack_matrix4
from control.pbvs_controller import TaskPoseMeasurement

class TaskPoseUdpSource:
    def __init__(self, bind_ip: str, port: int) -> None:
        self._lock = threading.Lock()
        self._latest: TaskPoseMeasurement | None = None
        self._running = True
        self._thread = threading.Thread(
            target=self._receive_loop,
            args=(bind_ip, port),
            daemon=True,
        )
        self._thread.start()

    def _receive_loop(self, bind_ip: str, port: int) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((bind_ip, port))
        sock.settimeout(0.1)

        try:
            while self._running:
                try:
                    data, _ = sock.recvfrom(2048)
                except socket.timeout:
                    continue

                if len(data) != MATRIX_SIZE:
                    continue

                matrix = unpack_matrix4(data)
                valid = np.all(np.isfinite(matrix))
                measurement = TaskPoseMeasurement(
                    T_TS=matrix,
                    timestamp=time.monotonic(),
                    valid=valid,
                )
                with self._lock:
                    self._latest = measurement
        finally:
            sock.close()

    def get_latest(self) -> TaskPoseMeasurement | None:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._running = False

from __future__ import annotations

import socket
import struct
import threading
import time

import numpy as np

from common.protocol import unpack_task_pose
from common.safety import finite_transform
from control.pbvs_controller import TaskPoseMeasurement


class TaskPosePacketFilter:
    """
    Convert versioned task-pose datagrams into controller measurements.

    Duplicate, reordered, malformed, and unsupported packets are ignored.
    Ignored packets do not refresh the controller freshness timestamp.
    """

    def __init__(
        self,
        *,
        minimum_confidence: float = 0.5,
    ) -> None:
        if not 0.0 <= minimum_confidence <= 1.0:
            raise ValueError(
                "minimum_confidence must be between 0 and 1."
            )

        self.minimum_confidence = float(
            minimum_confidence
        )
        self._last_source_sequence: int | None = None
        self._next_measurement_sequence = 1

    @property
    def last_source_sequence(
        self,
    ) -> int | None:
        return self._last_source_sequence

    def process_datagram(
        self,
        data: bytes,
        *,
        arrival_time: float,
    ) -> TaskPoseMeasurement | None:
        try:
            packet = unpack_task_pose(data)
        except (ValueError, struct.error):
            return None

        if (
            self._last_source_sequence is not None
            and packet.sequence_id
            <= self._last_source_sequence
        ):
            return None

        # Record every new source observation, including an invalid one.
        self._last_source_sequence = packet.sequence_id

        T_TS = np.asarray(
            packet.T_TS,
            dtype=float,
        ).copy()

        valid = (
            packet.valid
            and packet.confidence
            >= self.minimum_confidence
            and finite_transform(T_TS)
        )

        measurement = TaskPoseMeasurement(
            T_TS=T_TS,
            timestamp=float(arrival_time),
            valid=valid,
            sequence_id=self._next_measurement_sequence,
        )

        self._next_measurement_sequence += 1
        return measurement


class TaskPoseUdpSource:
    def __init__(
        self,
        bind_ip: str,
        port: int,
        *,
        minimum_confidence: float = 0.5,
    ) -> None:
        self._lock = threading.Lock()
        self._latest: TaskPoseMeasurement | None = None
        self._running = True
        self._filter = TaskPosePacketFilter(
            minimum_confidence=minimum_confidence,
        )

        self._thread = threading.Thread(
            target=self._receive_loop,
            args=(bind_ip, port),
            daemon=True,
        )
        self._thread.start()

    def _receive_loop(
        self,
        bind_ip: str,
        port: int,
    ) -> None:
        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )
        sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )
        sock.bind((bind_ip, port))
        sock.settimeout(0.1)

        try:
            while self._running:
                try:
                    data, _ = sock.recvfrom(2048)
                except socket.timeout:
                    continue

                measurement = (
                    self._filter.process_datagram(
                        data,
                        arrival_time=time.monotonic(),
                    )
                )

                if measurement is None:
                    continue

                with self._lock:
                    self._latest = measurement
        finally:
            sock.close()

    def get_latest(
        self,
    ) -> TaskPoseMeasurement | None:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=0.5)

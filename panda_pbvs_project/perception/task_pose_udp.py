from __future__ import annotations

import struct
import threading
import time

import socket

import numpy as np

from common.protocol import (
    TASK_POSE_LEGACY_VERSION,
    decode_task_pose_packet,
)
from common.safety import finite_transform
from control.pbvs_controller import TaskPoseMeasurement


class TaskPosePacketFilter:
    """
    Convert task-pose datagrams into controller measurements.

    Version-2 duplicates and reordered packets are ignored. Ignored
    packets do not create a new measurement and therefore do not refresh
    the controller freshness timestamp.

    Legacy matrix-only packets remain supported temporarily and are
    treated as new measurements because they contain no source sequence.
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

        self.minimum_confidence = (
            float(minimum_confidence)
        )

        self._last_v2_source_sequence: int | None = None

        # Controller-facing sequence numbers are local and always
        # increase, regardless of which packet version is received.
        self._next_measurement_sequence = 1

    @property
    def last_v2_source_sequence(
        self,
    ) -> int | None:
        return self._last_v2_source_sequence

    def process_datagram(
        self,
        data: bytes,
        *,
        arrival_time: float,
    ) -> TaskPoseMeasurement | None:
        """
        Return a new controller measurement, or None when the datagram
        should be ignored.

        None is returned for:
        - malformed packets,
        - duplicate version-2 sequences,
        - older/reordered version-2 sequences.
        """

        try:
            packet = decode_task_pose_packet(data)
        except (ValueError, struct_error):
            return None

        if packet.version != TASK_POSE_LEGACY_VERSION:
            if packet.sequence_id is None:
                return None

            if (
                self._last_v2_source_sequence is not None
                and packet.sequence_id
                <= self._last_v2_source_sequence
            ):
                # Duplicate or reordered packet.
                # Crucially, arrival_time is not propagated.
                return None

            # Record the source sequence even when confidence or validity
            # causes this observation to be rejected. A retransmission of
            # the same invalid observation must not look new.
            self._last_v2_source_sequence = (
                packet.sequence_id
            )

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
            sequence_id=(
                self._next_measurement_sequence
            ),
        )

        self._next_measurement_sequence += 1

        return measurement


# Avoid importing struct solely for one exception name in the main logic.
# struct.error inherits from Exception but not ValueError.
try:
    import struct

    struct_error = struct.error
except ImportError:  # pragma: no cover
    struct_error = Exception


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
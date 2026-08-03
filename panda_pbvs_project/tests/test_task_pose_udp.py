from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


from common.protocol import (  # noqa: E402
    TASK_POSE_MAGIC,
    TASK_POSE_SIZE,
    TASK_POSE_VERSION,
    pack_task_pose,
    unpack_task_pose,
)
from perception.task_pose_udp import (  # noqa: E402
    TaskPosePacketFilter,
)


def transform(
    *,
    x: float = 0.0,
) -> np.ndarray:
    result = np.eye(4)
    result[0, 3] = x
    return result


class TaskPoseProtocolTests(unittest.TestCase):
    def test_round_trip(self) -> None:
        T_TS = transform(x=0.123)

        encoded = pack_task_pose(
            T_TS,
            sequence_id=42,
            confidence=0.75,
            valid=True,
        )
        decoded = unpack_task_pose(encoded)

        self.assertEqual(len(encoded), TASK_POSE_SIZE)
        self.assertEqual(decoded.sequence_id, 42)
        self.assertAlmostEqual(
            decoded.confidence,
            0.75,
            places=6,
        )
        self.assertTrue(decoded.valid)

        np.testing.assert_allclose(
            decoded.T_TS,
            T_TS,
            atol=0.0,
        )

    def test_unsupported_128_byte_packet_is_rejected(
        self,
    ) -> None:
        unsupported_packet = bytes(128)

        with self.assertRaises(ValueError):
            unpack_task_pose(
                unsupported_packet
            )

    def test_invalid_magic_is_rejected(self) -> None:
        encoded = bytearray(
            pack_task_pose(
                transform(),
                sequence_id=1,
                confidence=1.0,
            )
        )
        encoded[:4] = b"BAD!"

        with self.assertRaises(ValueError):
            unpack_task_pose(bytes(encoded))

    def test_unsupported_version_is_rejected(
        self,
    ) -> None:
        encoded = bytearray(
            pack_task_pose(
                transform(),
                sequence_id=1,
                confidence=1.0,
            )
        )
        encoded[4] = TASK_POSE_VERSION + 1

        with self.assertRaises(ValueError):
            unpack_task_pose(bytes(encoded))

    def test_protocol_constants_are_expected(
        self,
    ) -> None:
        self.assertEqual(TASK_POSE_MAGIC, b"PTP2")
        self.assertEqual(TASK_POSE_VERSION, 2)
        self.assertEqual(TASK_POSE_SIZE, 148)


class TaskPosePacketFilterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.packet_filter = TaskPosePacketFilter(
            minimum_confidence=0.60,
        )

    def packet(
        self,
        *,
        sequence_id: int,
        x: float = 0.0,
        confidence: float = 0.90,
        valid: bool = True,
    ) -> bytes:
        return pack_task_pose(
            transform(x=x),
            sequence_id=sequence_id,
            confidence=confidence,
            valid=valid,
        )

    def test_new_sequence_is_accepted(self) -> None:
        measurement = (
            self.packet_filter.process_datagram(
                self.packet(sequence_id=1),
                arrival_time=10.0,
            )
        )

        self.assertIsNotNone(measurement)
        self.assertTrue(measurement.valid)
        self.assertEqual(
            measurement.timestamp,
            10.0,
        )
        self.assertEqual(
            self.packet_filter.last_source_sequence,
            1,
        )

    def test_duplicate_sequence_is_ignored(
        self,
    ) -> None:
        first = self.packet_filter.process_datagram(
            self.packet(sequence_id=1),
            arrival_time=10.0,
        )

        duplicate = (
            self.packet_filter.process_datagram(
                self.packet(
                    sequence_id=1,
                    x=0.25,
                ),
                arrival_time=20.0,
            )
        )

        self.assertIsNotNone(first)
        self.assertIsNone(duplicate)
        self.assertEqual(first.timestamp, 10.0)

    def test_older_sequence_is_ignored(self) -> None:
        newest = self.packet_filter.process_datagram(
            self.packet(sequence_id=10),
            arrival_time=10.0,
        )

        reordered = (
            self.packet_filter.process_datagram(
                self.packet(sequence_id=9),
                arrival_time=11.0,
            )
        )

        self.assertIsNotNone(newest)
        self.assertIsNone(reordered)
        self.assertEqual(
            self.packet_filter.last_source_sequence,
            10,
        )

    def test_low_confidence_is_marked_invalid(
        self,
    ) -> None:
        measurement = (
            self.packet_filter.process_datagram(
                self.packet(
                    sequence_id=1,
                    confidence=0.20,
                ),
                arrival_time=10.0,
            )
        )

        self.assertIsNotNone(measurement)
        self.assertFalse(measurement.valid)

    def test_tracker_invalid_flag_is_preserved(
        self,
    ) -> None:
        measurement = (
            self.packet_filter.process_datagram(
                self.packet(
                    sequence_id=1,
                    valid=False,
                ),
                arrival_time=10.0,
            )
        )

        self.assertIsNotNone(measurement)
        self.assertFalse(measurement.valid)

    def test_nonfinite_pose_is_marked_invalid(
        self,
    ) -> None:
        T_TS = transform()
        T_TS[0, 3] = np.nan

        measurement = (
            self.packet_filter.process_datagram(
                pack_task_pose(
                    T_TS,
                    sequence_id=1,
                    confidence=0.90,
                ),
                arrival_time=10.0,
            )
        )

        self.assertIsNotNone(measurement)
        self.assertFalse(measurement.valid)

    def test_malformed_packet_is_ignored(
        self,
    ) -> None:
        measurement = (
            self.packet_filter.process_datagram(
                b"not-a-task-pose-packet",
                arrival_time=10.0,
            )
        )

        self.assertIsNone(measurement)

    def test_unsupported_128_byte_packet_is_ignored(
        self,
    ) -> None:
        measurement = (
            self.packet_filter.process_datagram(
                bytes(128),
                arrival_time=10.0,
            )
        )

        self.assertIsNone(measurement)


if __name__ == "__main__":
    unittest.main(verbosity=2)

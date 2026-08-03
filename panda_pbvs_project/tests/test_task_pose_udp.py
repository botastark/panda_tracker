from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


from common.protocol import (  # noqa: E402
    TASK_POSE_LEGACY_VERSION,
    TASK_POSE_V2_VERSION,
    decode_task_pose_packet,
    pack_task_pose,
    pack_task_pose_v2,
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
    def test_v2_round_trip(self) -> None:
        T_TS = transform(x=0.123)

        decoded = decode_task_pose_packet(
            pack_task_pose_v2(
                T_TS,
                sequence_id=42,
                confidence=0.75,
                valid=True,
            )
        )

        self.assertEqual(
            decoded.version,
            TASK_POSE_V2_VERSION,
        )
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

    def test_legacy_packet_remains_supported(
        self,
    ) -> None:
        T_TS = transform(x=0.456)

        decoded = decode_task_pose_packet(
            pack_task_pose(T_TS)
        )

        self.assertEqual(
            decoded.version,
            TASK_POSE_LEGACY_VERSION,
        )
        self.assertIsNone(decoded.sequence_id)
        self.assertEqual(decoded.confidence, 1.0)
        self.assertTrue(decoded.valid)

        np.testing.assert_allclose(
            decoded.T_TS,
            T_TS,
            atol=0.0,
        )


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
        return pack_task_pose_v2(
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
            self.packet_filter.last_v2_source_sequence,
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

        # Because no replacement measurement was produced, the source
        # will retain the original timestamp of 10.0.
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
            self.packet_filter.last_v2_source_sequence,
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

        data = pack_task_pose_v2(
            T_TS,
            sequence_id=1,
            confidence=0.90,
        )

        measurement = (
            self.packet_filter.process_datagram(
                data,
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

    def test_legacy_packets_remain_operational(
        self,
    ) -> None:
        first = self.packet_filter.process_datagram(
            pack_task_pose(transform(x=0.0)),
            arrival_time=10.0,
        )

        second = self.packet_filter.process_datagram(
            pack_task_pose(transform(x=0.1)),
            arrival_time=11.0,
        )

        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        self.assertTrue(first.valid)
        self.assertTrue(second.valid)

        self.assertGreater(
            second.sequence_id,
            first.sequence_id,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
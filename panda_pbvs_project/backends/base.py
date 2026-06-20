from __future__ import annotations

from typing import Protocol
import numpy as np


class RobotBackend(Protocol):
    def get_current_pose(self) -> tuple[np.ndarray | None, float]:
        """Return (T_BE, age_seconds)."""

    def send_target_pose(self, T_BE_target: np.ndarray) -> None:
        """Send an absolute EE target."""

    def healthy(self) -> bool:
        """Return whether state feedback is fresh and usable."""

    def close(self) -> None:
        """Release sockets/resources."""

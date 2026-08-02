from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import json
import math
import numpy as np
from common.safety import finite_transform

def _matrix(raw: dict[str, Any], name: str) -> np.ndarray:
    value = np.asarray(raw[name], dtype=float)
    if value.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4.")
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{name} contains invalid values.")
    return value


@dataclass(frozen=True)
class PBVSConfig:
    control_rate_hz: float
    control_orientation: bool
    kp_position: float
    kp_orientation: float
    max_linear_speed: float
    max_angular_speed: float
    max_command_lead: float
    panda_state_timeout: float
    tracker_timeout: float
    max_tracker_position_jump: float
    max_tracker_angle_jump: float
    max_enable_position_error: float
    max_enable_orientation_error: float
    consecutive_valid_required: int

    # Target-motion feedforward.
    target_feedforward_enabled: bool
    target_velocity_filter_alpha: float
    max_target_linear_speed: float
    max_target_angular_speed: float

    workspace_min: np.ndarray
    workspace_max: np.ndarray
    # Direct-controller transforms.
    T_ES: np.ndarray
    T_TS_des: np.ndarray
    tool_visualization: dict[str, Any]

def load_pbvs_config(path: Path) -> PBVSConfig:
    raw = json.loads(path.read_text())
    workspace = raw.get("workspace", {})
    workspace_min = np.asarray(
        workspace.get("min", [-1.0, -1.0, -1.0]),
        dtype=float,
    )
    workspace_max = np.asarray(
        workspace.get("max", [1.0, 1.0, 1.0]),
        dtype=float,
    )
    T_ES = _matrix(raw, "T_ES")
    T_TS_des = _matrix(raw, "T_TS_des")

    if not finite_transform(T_ES):
        raise ValueError("T_ES is not a valid homogeneous transform.")

    if not finite_transform(T_TS_des):
        raise ValueError(
            "T_TS_des is not a valid homogeneous transform."
        )

    config = PBVSConfig(
        control_rate_hz=float(raw["control_rate_hz"]),
        control_orientation=bool(
            raw.get("control_orientation", True)
        ),
        kp_position=float(raw["kp_position"]),
        kp_orientation=float(raw["kp_orientation"]),
        max_linear_speed=float(raw["max_linear_speed"]),
        max_angular_speed=math.radians(
            float(raw["max_angular_speed_deg"])
        ),
        max_command_lead=float(raw["max_command_lead"]),
        panda_state_timeout=float(raw["panda_state_timeout"]),
        tracker_timeout=float(raw["tracker_timeout"]),
        max_tracker_position_jump=float(
            raw["max_tracker_position_jump"]
        ),
        max_tracker_angle_jump=math.radians(
            float(raw["max_tracker_angle_jump_deg"])
        ),
        max_enable_position_error=float(
            raw["max_enable_position_error"]
        ),
        max_enable_orientation_error=math.radians(
            float(raw["max_enable_orientation_error_deg"])
        ),
        consecutive_valid_required=int(
            raw["consecutive_valid_required"]
        ),
        target_feedforward_enabled=bool(
            raw.get("target_feedforward_enabled", False)
        ),
        target_velocity_filter_alpha=float(
            raw.get("target_velocity_filter_alpha", 0.25)
        ),
        max_target_linear_speed=float(
            raw.get(
                "max_target_linear_speed",
                raw["max_linear_speed"],
            )
        ),
        max_target_angular_speed=math.radians(
            float(
                raw.get(
                    "max_target_angular_speed_deg",
                    raw["max_angular_speed_deg"],
                )
            )
        ),
        workspace_min=workspace_min,
        workspace_max=workspace_max,
        T_ES=T_ES,
        T_TS_des=T_TS_des,
        tool_visualization=dict(
            raw.get("tool_visualization", {})
        ),
    )
    if not 0.0 < config.target_velocity_filter_alpha <= 1.0:
        raise ValueError(
            "target_velocity_filter_alpha must be in (0, 1]."
        )

    if config.max_target_linear_speed < 0.0:
        raise ValueError(
            "max_target_linear_speed must be non-negative."
        )


    if config.max_target_angular_speed < 0.0:
        raise ValueError(
            "max_target_angular_speed must be non-negative."
        )
    if config.max_command_lead <= 0.0:
        raise ValueError("max_command_lead must be positive.")

    return config

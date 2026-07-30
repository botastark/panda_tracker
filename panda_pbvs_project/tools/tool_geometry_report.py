#!/usr/bin/env python3
"""Report consistency and physical-axis checks for PBVS tool geometry."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from common.geometry import invert_transform, so3_log
from common.safety import finite_transform


def matrix(raw: dict[str, Any], name: str) -> np.ndarray:
    value = np.asarray(raw[name], dtype=float)
    if not finite_transform(value):
        raise ValueError(f"{name} is not a valid homogeneous transform.")
    return value


def vector(
    raw: dict[str, Any],
    name: str,
    default: list[float],
) -> np.ndarray:
    value = np.asarray(raw.get(name, default), dtype=float).reshape(3)
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{name} contains non-finite values.")
    return value


def normalized(value: np.ndarray, name: str) -> np.ndarray:
    norm = float(np.linalg.norm(value))
    if norm < 1e-12:
        raise ValueError(f"{name} must be non-zero.")
    return value / norm


def angle_degrees(left: np.ndarray, right: np.ndarray) -> float:
    left = normalized(left, "left axis")
    right = normalized(right, "right axis")
    cosine = float(np.clip(np.dot(left, right), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def point_in_parent(transform: np.ndarray, point: np.ndarray) -> np.ndarray:
    return transform[:3, :3] @ point + transform[:3, 3]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Report transform closure and camera/stick alignment from a "
            "PBVS JSON configuration."
        )
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument(
        "--require-camera-perpendicular-to-ee-z",
        action="store_true",
    )
    parser.add_argument(
        "--require-camera-positive-ee-z",
        action="store_true",
    )
    parser.add_argument(
        "--require-stick-positive-ee-z",
        action="store_true",
    )
    parser.add_argument(
        "--axis-tolerance-deg",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--max-chain-translation-mm",
        type=float,
        default=0.01,
    )
    parser.add_argument(
        "--max-chain-angle-deg",
        type=float,
        default=0.01,
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    raw = json.loads(args.config.read_text())

    T_EC = matrix(raw, "T_EC")
    T_CS = matrix(raw, "T_CS")
    T_ES = matrix(raw, "T_ES")

    derived_T_ES = T_EC @ T_CS
    chain_delta = invert_transform(T_ES) @ derived_T_ES
    chain_translation_mm = 1000.0 * float(
        np.linalg.norm(chain_delta[:3, 3])
    )
    chain_angle_deg = math.degrees(
        float(np.linalg.norm(so3_log(chain_delta[:3, :3])))
    )

    visual = dict(raw.get("tool_visualization", {}))
    T_EH = np.asarray(visual.get("T_EH", np.eye(4)), dtype=float)
    if not finite_transform(T_EH):
        raise ValueError("tool_visualization.T_EH is invalid.")

    lens_axis_C = normalized(
        vector(visual, "camera_lens_axis_C", [0.0, 0.0, 1.0]),
        "camera_lens_axis_C",
    )
    camera_axis_E = T_EC[:3, :3] @ lens_axis_C
    ee_z = np.array([0.0, 0.0, 1.0])
    camera_to_ee_z_deg = angle_degrees(camera_axis_E, ee_z)
    camera_perpendicular_error_deg = abs(camera_to_ee_z_deg - 90.0)
    camera_positive_ee_z_error_deg = camera_to_ee_z_deg

    stick_axis_E = T_ES[:3, :3] @ np.array([0.0, 0.0, 1.0])
    stick_to_positive_ee_z_deg = angle_degrees(stick_axis_E, ee_z)

    T_HC = np.asarray(visual.get("T_HC", np.eye(4)), dtype=float)
    T_HS = np.asarray(visual.get("T_HS", np.eye(4)), dtype=float)
    if not finite_transform(T_HC):
        raise ValueError("tool_visualization.T_HC is invalid.")
    if not finite_transform(T_HS):
        raise ValueError("tool_visualization.T_HS is invalid.")

    derived_T_EC = T_EH @ T_HC
    derived_T_ES_from_holder = T_EH @ T_HS
    camera_holder_delta = invert_transform(T_EC) @ derived_T_EC
    tip_holder_delta = invert_transform(T_ES) @ derived_T_ES_from_holder
    camera_holder_translation_mm = 1000.0 * float(
        np.linalg.norm(camera_holder_delta[:3, 3])
    )
    camera_holder_angle_deg = math.degrees(
        float(np.linalg.norm(so3_log(camera_holder_delta[:3, :3])))
    )
    tip_holder_translation_mm = 1000.0 * float(
        np.linalg.norm(tip_holder_delta[:3, 3])
    )
    tip_holder_angle_deg = math.degrees(
        float(np.linalg.norm(so3_log(tip_holder_delta[:3, :3])))
    )

    stick_mount_H = vector(visual, "stick_mount_H", [0.0, 0.0, 0.0])
    rod_end_H = vector(visual, "rod_end_H", [0.0, 0.0, 0.0])
    sphere_center_H = vector(
        visual,
        "sphere_center_H",
        [0.0, 0.0, 0.0],
    )
    tool_tip_H = T_HS[:3, 3]
    stick_radius_m = float(visual.get("stick_radius", 0.0))
    sphere_center_to_tip_mm = 1000.0 * float(
        np.linalg.norm(tool_tip_H - sphere_center_H)
    )
    sphere_radius_mm = 1000.0 * stick_radius_m
    stick_mount_E = point_in_parent(T_EH, stick_mount_H)
    rod_end_E = point_in_parent(T_EH, rod_end_H)
    sphere_center_E = point_in_parent(T_EH, sphere_center_H)
    shaft_vector_E = rod_end_E - stick_mount_E
    shaft_length_mm = 1000.0 * float(np.linalg.norm(shaft_vector_E))
    shaft_to_positive_ee_z_deg = (
        float("nan")
        if shaft_length_mm < 1e-9
        else angle_degrees(shaft_vector_E, ee_z)
    )

    holder_center_H = vector(
        visual,
        "holder_center_H",
        [0.0, 0.0, 0.0],
    )
    holder_center_E = point_in_parent(T_EH, holder_center_H)
    holder_box_rotation_deg = float(
        visual.get("holder_box_rotation_deg", 0.0)
    )
    holder_box_rotation_rad = math.radians(holder_box_rotation_deg)
    R_H_holder_box = np.array(
        [
            [
                math.cos(holder_box_rotation_rad),
                -math.sin(holder_box_rotation_rad),
                0.0,
            ],
            [
                math.sin(holder_box_rotation_rad),
                math.cos(holder_box_rotation_rad),
                0.0,
            ],
            [0.0, 0.0, 1.0],
        ]
    )
    R_E_holder_box = T_EH[:3, :3] @ R_H_holder_box
    holder_box_yaw_E_deg = math.degrees(
        math.atan2(R_E_holder_box[1, 0], R_E_holder_box[0, 0])
    )

    support_1_holder_H = vector(
        visual,
        "camera_support_1_holder_H",
        [0.0, 0.0, 0.0],
    )
    support_2_holder_H = vector(
        visual,
        "camera_support_2_holder_H",
        [0.0, 0.0, 0.0],
    )
    support_1_camera_H = vector(
        visual,
        "camera_support_1_camera_H",
        [0.0, 0.0, 0.0],
    )
    support_2_camera_H = vector(
        visual,
        "camera_support_2_camera_H",
        [0.0, 0.0, 0.0],
    )
    support_lengths_mm = 1000.0 * np.array(
        [
            np.linalg.norm(support_1_camera_H - support_1_holder_H),
            np.linalg.norm(support_2_camera_H - support_2_holder_H),
        ]
    )
    support_1_direction_H = support_1_camera_H - support_1_holder_H
    support_2_direction_H = support_2_camera_H - support_2_holder_H
    support_to_rod_deg = np.array(
        [
            angle_degrees(
                support_1_direction_H,
                np.array([0.0, 0.0, 1.0]),
            ),
            angle_degrees(
                support_2_direction_H,
                np.array([0.0, 0.0, 1.0]),
            ),
        ]
    )
    camera_body_center_H = vector(
        visual,
        "camera_body_center_H",
        [0.0, 0.0, 0.0],
    )
    support_axial_coordinates_mm = 1000.0 * np.array(
        [
            support_1_holder_H[2],
            support_1_camera_H[2],
            support_2_holder_H[2],
            support_2_camera_H[2],
        ]
    )
    camera_body_axial_offset_mm = 1000.0 * camera_body_center_H[2]
    support_camera_axial_error_mm = float(
        np.max(
            np.abs(
                support_axial_coordinates_mm
                - camera_body_axial_offset_mm
            )
        )
    )

    camera_mount_H = vector(
        visual,
        "camera_c_mount_center_H",
        [0.0, 0.0, 0.0],
    )
    camera_optical_origin_H = vector(
        visual,
        "camera_optical_origin_H",
        [0.0, 0.0, 0.0],
    )
    camera_visual_to_transform_mm = 1000.0 * float(
        np.linalg.norm(camera_optical_origin_H - T_HC[:3, 3])
    )
    camera_mount_to_optical_mm = 1000.0 * float(
        np.linalg.norm(T_HC[:3, 3] - camera_mount_H)
    )
    camera_to_stick_axis_mm = 1000.0 * float(
        np.linalg.norm(
            T_HC[:2, 3] - sphere_center_H[:2]
        )
    )

    print(f"Config: {args.config}")
    print("Convention: T_XY is frame Y expressed in frame X.")
    print(
        "T_EC @ T_CS -> T_ES residual: "
        f"{chain_translation_mm:.6f} mm, {chain_angle_deg:.6f} deg"
    )
    print(
        "camera optical axis in E:",
        np.array2string(camera_axis_E, precision=6),
    )
    print(
        "camera optical axis vs +z_E: "
        f"{camera_to_ee_z_deg:.3f} deg "
        f"(positive-parallel error "
        f"{camera_positive_ee_z_error_deg:.3f} deg; "
        f"perpendicular error {camera_perpendicular_error_deg:.3f} deg)"
    )
    print(
        "stick +z_S axis in E:",
        np.array2string(stick_axis_E, precision=6),
    )
    print(
        "stick +z_S axis vs +z_E: "
        f"{stick_to_positive_ee_z_deg:.3f} deg"
    )
    print(
        "T_EH @ T_HC -> T_EC residual: "
        f"{camera_holder_translation_mm:.6f} mm, "
        f"{camera_holder_angle_deg:.6f} deg"
    )
    print(
        "T_EH @ T_HS -> T_ES residual: "
        f"{tip_holder_translation_mm:.6f} mm, "
        f"{tip_holder_angle_deg:.6f} deg"
    )
    print(
        "visual rod mount/end E [m]:",
        np.array2string(stick_mount_E, precision=6),
        np.array2string(rod_end_E, precision=6),
    )
    print(
        "holder datum/box center E [m]:",
        np.array2string(T_EH[:3, 3], precision=6),
        np.array2string(holder_center_E, precision=6),
    )
    print(
        "holder-box z rotation H/E: "
        f"{holder_box_rotation_deg:.3f} / "
        f"{holder_box_yaw_E_deg:.3f} deg"
    )
    print(
        "camera-support lengths: "
        f"{support_lengths_mm[0]:.3f} / "
        f"{support_lengths_mm[1]:.3f} mm"
    )
    print(
        "camera center from pin along rod: "
        f"{camera_body_axial_offset_mm:.3f} mm"
    )
    print(
        "camera-support endpoint z_H [mm]:",
        np.array2string(
            support_axial_coordinates_mm,
            precision=3,
        ),
    )
    print(
        "camera-support angle to rod: "
        f"{support_to_rod_deg[0]:.3f} / "
        f"{support_to_rod_deg[1]:.3f} deg"
    )
    print(
        "sphere center/tool tip H [m]:",
        np.array2string(sphere_center_H, precision=6),
        np.array2string(tool_tip_H, precision=6),
    )
    print(
        "sphere center-to-tip / configured radius: "
        f"{sphere_center_to_tip_mm:.3f} / {sphere_radius_mm:.3f} mm"
    )
    print(
        "sphere center/tool tip E [m]:",
        np.array2string(sphere_center_E, precision=6),
        np.array2string(T_ES[:3, 3], precision=6),
    )
    print(
        "visual rod start-to-end: "
        f"{shaft_length_mm:.3f} mm, "
        f"{shaft_to_positive_ee_z_deg:.3f} deg from +z_E"
    )
    print(
        "camera C-mount to nominal optical origin: "
        f"{camera_mount_to_optical_mm:.3f} mm"
    )
    print(
        "visual optical origin vs T_HC translation: "
        f"{camera_visual_to_transform_mm:.6f} mm"
    )
    print(
        "camera-to-stick transverse axis separation: "
        f"{camera_to_stick_axis_mm:.3f} mm"
    )

    failures: list[str] = []

    if chain_translation_mm > args.max_chain_translation_mm:
        failures.append("T_ES translation chain closure exceeds tolerance")
    if chain_angle_deg > args.max_chain_angle_deg:
        failures.append("T_ES rotation chain closure exceeds tolerance")
    if camera_holder_translation_mm > args.max_chain_translation_mm:
        failures.append("T_EH @ T_HC translation closure exceeds tolerance")
    if camera_holder_angle_deg > args.max_chain_angle_deg:
        failures.append("T_EH @ T_HC rotation closure exceeds tolerance")
    if tip_holder_translation_mm > args.max_chain_translation_mm:
        failures.append("T_EH @ T_HS translation closure exceeds tolerance")
    if tip_holder_angle_deg > args.max_chain_angle_deg:
        failures.append("T_EH @ T_HS rotation closure exceeds tolerance")
    if abs(sphere_center_to_tip_mm - sphere_radius_mm) > 0.01:
        failures.append(
            "sphere center-to-tip distance does not equal sphere radius"
        )
    if camera_visual_to_transform_mm > args.max_chain_translation_mm:
        failures.append(
            "camera_optical_origin_H does not equal T_HC translation"
        )
    if np.max(np.abs(support_to_rod_deg - 90.0)) > args.axis_tolerance_deg:
        failures.append("camera supports are not perpendicular to the rod")
    if support_camera_axial_error_mm > args.max_chain_translation_mm:
        failures.append(
            "camera supports are not aligned with the camera-center "
            "axial coordinate"
        )
    if (
        args.require_camera_perpendicular_to_ee_z
        and camera_perpendicular_error_deg > args.axis_tolerance_deg
    ):
        failures.append("camera optical axis is not perpendicular to EE +z")
    if (
        args.require_camera_positive_ee_z
        and camera_positive_ee_z_error_deg > args.axis_tolerance_deg
    ):
        failures.append("camera optical axis is not aligned with EE +z")
    if (
        args.require_stick_positive_ee_z
        and stick_to_positive_ee_z_deg > args.axis_tolerance_deg
    ):
        failures.append("stick +z axis is not aligned with EE +z")
    if (
        args.require_stick_positive_ee_z
        and shaft_length_mm >= 1e-9
        and shaft_to_positive_ee_z_deg > args.axis_tolerance_deg
    ):
        failures.append(
            "visual stick mount-to-tip direction is not aligned with EE +z"
        )

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1

    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

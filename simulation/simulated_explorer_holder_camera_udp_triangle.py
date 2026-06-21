#!/usr/bin/env python3
"""
Kinematic Panda simulator that mirrors explorer's UDP interface.

Receives:
    command port 2600, <6f: commanded x, y, z, roll, pitch, yaw
    optional real-state port 6201, <6f: measured physical Panda EE pose
    triangle port 6601, <6f: triangle x, y, z, roll, pitch, yaw in Panda base B

Sends:
    state port 6200, <6f: simulated EE state (normal simulation mode)
    tracker port 6500, <16d: synthetic tracker pose T_TC

The triangle is never invented at a fixed offset. It remains hidden until a
fresh external triangle pose is received. While triangle packets remain fresh,
the simulator places the triangle in the scene and computes T_TC from the
simulated camera pose and streamed triangle pose.

Because mirror mode currently receives only a 6-DoF end-effector pose, it
matches the camera/end-effector pose but cannot guarantee the same redundant
joint configuration as the physical seven-joint arm.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import mujoco
import mujoco.viewer
import numpy as np


POSE_FORMAT = "<6f"
POSE_SIZE = struct.calcsize(POSE_FORMAT)
TRACKER_FORMAT = "<16d"


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def so3_log(rotation: np.ndarray) -> np.ndarray:
    cos_theta = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    theta = math.acos(float(cos_theta))
    if theta < 1e-8:
        return 0.5 * np.array([
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ])
    return theta / (2.0 * math.sin(theta)) * np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def rpy_from_rotation(rotation: np.ndarray) -> np.ndarray:
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return np.array([roll, pitch, yaw])


def make_transform(rotation: np.ndarray, position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position
    return transform


def quat_wxyz_from_rotation(rotation: np.ndarray) -> np.ndarray:
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, rotation.reshape(-1))
    return quaternion



def validate_transform(name: str, transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError(f"{name} must be a 4x4 transform.")
    if not np.all(np.isfinite(transform)):
        raise ValueError(f"{name} contains non-finite values.")
    return transform


def load_transform_config(
    path: Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict]:
    raw = json.loads(path.read_text())
    T_EC = validate_transform("T_EC", np.asarray(raw["T_EC"], dtype=float))
    T_CS = validate_transform("T_CS", np.asarray(raw["T_CS"], dtype=float))
    T_TS_des = validate_transform(
        "T_TS_des", np.asarray(raw["T_TS_des"], dtype=float)
    )
    
    tool_visualization = raw.get("tool_visualization", {})
    T_TC_des = T_TS_des @ np.linalg.inv(T_CS)
    return T_EC, T_CS, T_TS_des, T_TC_des, tool_visualization


def vector_string(vector: np.ndarray) -> str:
    return " ".join(f"{float(value):.10g}" for value in np.asarray(vector).reshape(-1))


def quaternion_string(rotation: np.ndarray) -> str:
    return vector_string(quat_wxyz_from_rotation(rotation))


def add_axis_geoms(parent: ET.Element, length: float = 0.06) -> None:
    axes = (
        ((length, 0.0, 0.0), "1 0 0 1"),
        ((0.0, length, 0.0), "0 1 0 1"),
        ((0.0, 0.0, length), "0 0 1 1"),
    )
    for endpoint, rgba in axes:
        ET.SubElement(
            parent,
            "geom",
            {
                "type": "capsule",
                "fromto": f"0 0 0 {vector_string(endpoint)}",
                "size": ".003",
                "rgba": rgba,
                "contype": "0",
                "conaffinity": "0",
                "group": "3",
            },
        )


def find_body_by_name(root: ET.Element, name: str) -> Optional[ET.Element]:
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    return None


def config_vector(config: dict, name: str, default: list[float]) -> np.ndarray:
    value = np.asarray(config.get(name, default), dtype=float).reshape(3)
    return value


def config_transform(config: dict, name: str) -> np.ndarray:
    value = np.asarray(config.get(name, np.eye(4)), dtype=float)
    return validate_transform(name, value)


def add_visual_capsule(
    parent: ET.Element,
    name: str,
    start: np.ndarray,
    end: np.ndarray,
    radius: float,
    rgba: str,
) -> None:
    if np.linalg.norm(np.asarray(end) - np.asarray(start)) < 1e-6:
        return
    ET.SubElement(parent, "geom", {
        "name": name,
        "type": "capsule",
        "fromto": f"{vector_string(start)} {vector_string(end)}",
        "size": f"{radius:.10g}",
        "rgba": rgba,
        "contype": "0",
        "conaffinity": "0",
        "group": "2",
        "density": "10",
    })


def create_scene_xml(
    panda_xml: Path,
    ee_body_name: str,
    T_EC: np.ndarray,
    T_CS: np.ndarray,
    tool_visualization: dict,
) -> Path:
    """Add a holder, side stick, two-link camera bracket, camera, and triangle."""
    tree = ET.parse(panda_xml)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"{panda_xml} has no <worldbody> element.")

    ee_body = find_body_by_name(root, ee_body_name)
    if ee_body is None:
        raise ValueError(f"EE body {ee_body_name!r} was not found.")

    # Visual-only mechanical layout. T_EC and T_CS remain authoritative for PBVS.
    T_EH = config_transform(tool_visualization, "T_EH")
    holder_half_size = config_vector(
        tool_visualization, "holder_half_size", [0.055, 0.040, 0.025]
    )
    holder_color = str(tool_visualization.get("holder_rgba", ".55 .55 .60 1"))
    rod_radius = float(tool_visualization.get("rod_radius", 0.006))
    stick_radius = float(tool_visualization.get("stick_radius", 0.006))

    # Mount points are expressed in the holder frame H.
    stick_mount_H = config_vector(
        tool_visualization, "stick_mount_H", [0.055, 0.0, 0.0]
    )
    support_1_mount_H = config_vector(
        tool_visualization,
        "support_1_mount_H",
        [0.055, -0.025, 0.0],
    )

    support_2_mount_H = config_vector(
        tool_visualization,
        "support_2_mount_H",
        [0.055, 0.025, 0.0],
    )

    support_1_top_H = config_vector(
        tool_visualization,
        "support_1_top_H",
        [0.155, -0.025, 0.0],
    )

    support_2_top_H = config_vector(
        tool_visualization,
        "support_2_top_H",
        [0.155, 0.025, 0.0],
    )

    platform_center_H = config_vector(
        tool_visualization,
        "camera_platform_center_H",
        [0.165, 0.0, 0.0],
    )

    platform_half_size = config_vector(
        tool_visualization,
        "camera_platform_half_size",
        [0.008, 0.05, 0.04],
    )

    platform_rgba = str(
        tool_visualization.get(
            "camera_platform_rgba",
            ".35 .35 .38 1",
        )
    )

    def holder_point_in_ee(point_H: np.ndarray) -> np.ndarray:
        return T_EH[:3, :3] @ point_H + T_EH[:3, 3]

    holder_body = ET.SubElement(ee_body, "body", {
        "name": "pbvs_holder",
        "pos": vector_string(T_EH[:3, 3]),
        "quat": quaternion_string(T_EH[:3, :3]),
    })
    ET.SubElement(holder_body, "geom", {
        "name": "pbvs_holder_box",
        "type": "box",
        "size": vector_string(holder_half_size),
        "rgba": holder_color,
        "contype": "0",
        "conaffinity": "0",
        "group": "2",
        "density": "10",
    })
    ET.SubElement(holder_body, "site", {
        "name": "pbvs_holder_frame", "type": "sphere", "size": ".006",
        "rgba": "1 1 1 1", "group": "3",
    })
    add_axis_geoms(holder_body, 0.06)

    camera_position_E = T_EC[:3, 3]
    stick_tip_position_E = (T_EC @ T_CS)[:3, 3]
    stick_mount_E = holder_point_in_ee(stick_mount_H)
    support_1_mount_E = holder_point_in_ee(support_1_mount_H)
    support_2_mount_E = holder_point_in_ee(support_2_mount_H)

    support_1_top_E = holder_point_in_ee(support_1_top_H)
    support_2_top_E = holder_point_in_ee(support_2_top_H)

    platform_center_E = holder_point_in_ee(platform_center_H)
    # One side: holder -> straight stick -> tip.
    add_visual_capsule(
        ee_body, "pbvs_stick_shaft", stick_mount_E, stick_tip_position_E,
        stick_radius, ".30 .30 .32 1"
    )

    add_visual_capsule(
        ee_body,
        "pbvs_camera_support_1",
        support_1_mount_E,
        support_1_top_E,
        rod_radius,
        ".25 .25 .28 1",
    )

    add_visual_capsule(
        ee_body,
        "pbvs_camera_support_2",
        support_2_mount_E,
        support_2_top_E,
        rod_radius,
        ".25 .25 .28 1",
    )

    add_visual_capsule(
        ee_body,
        "pbvs_camera_top_crossbar",
        support_1_top_E,
        support_2_top_E,
        rod_radius,
        ".25 .25 .28 1",
    )

    platform_body = ET.SubElement(
        ee_body,
        "body",
        {
            "name": "pbvs_camera_platform",
            "pos": vector_string(platform_center_E),
            "quat": quaternion_string(T_EH[:3, :3]),
        },
    )

    ET.SubElement(
        platform_body,
        "geom",
        {
            "name": "pbvs_camera_platform_box",
            "type": "box",
            "size": vector_string(platform_half_size),
            "rgba": platform_rgba,
            "contype": "0",
            "conaffinity": "0",
            "group": "2",
            "density": "10",
        },
    )

    camera_half_size = config_vector(
        tool_visualization, "camera_half_size", [0.035, 0.027, 0.020]
    )
    lens_axis = config_vector(tool_visualization, "camera_lens_axis_C", [0, 0, 1])
    lens_axis_norm = np.linalg.norm(lens_axis)
    if lens_axis_norm < 1e-9:
        lens_axis = np.array([0.0, 0.0, 1.0])
    else:
        lens_axis = lens_axis / lens_axis_norm

    camera_body = ET.SubElement(ee_body, "body", {
        "name": "pbvs_camera",
        "pos": vector_string(T_EC[:3, 3]),
        "quat": quaternion_string(T_EC[:3, :3]),
    })
    ET.SubElement(camera_body, "geom", {
        "name": "pbvs_camera_housing", "type": "box",
        "size": vector_string(camera_half_size), "rgba": ".10 .30 .90 .90",
        "contype": "0", "conaffinity": "0", "group": "2", "density": "10",
    })
    # The lens marker is a short capsule along configurable camera-frame axis.
    lens_start = lens_axis * max(camera_half_size) * 0.7
    lens_end = lens_axis * (max(camera_half_size) * 0.7 + 0.025)
    add_visual_capsule(
        camera_body, "pbvs_camera_lens", lens_start, lens_end,
        0.012, ".03 .03 .05 1"
    )
    ET.SubElement(camera_body, "site", {
        "name": "pbvs_camera_frame", "type": "sphere", "size": ".006",
        "rgba": "1 1 1 1", "group": "3",
    })
    add_axis_geoms(camera_body, 0.07)

    # Stick tip frame is derived exactly from T_ES = T_EC T_CS.
    T_ES = T_EC @ T_CS
    tip_body = ET.SubElement(ee_body, "body", {
        "name": "pbvs_stick_tip",
        "pos": vector_string(T_ES[:3, 3]),
        "quat": quaternion_string(T_ES[:3, :3]),
    })
    ET.SubElement(tip_body, "geom", {
        "name": "pbvs_stick_tip_marker", "type": "sphere", "size": ".012",
        "rgba": "1 .15 .05 1", "contype": "0", "conaffinity": "0",
        "group": "2", "density": "10",
    })
    ET.SubElement(tip_body, "site", {
        "name": "pbvs_stick_tip_frame", "type": "sphere", "size": ".006",
        "rgba": "1 1 0 1", "group": "3",
    })
    add_axis_geoms(tip_body, 0.05)

    ET.SubElement(worldbody, "light", {
        "name": "pbvs_light", "pos": "0 0 2", "dir": "0 0 -1",
    })
    ET.SubElement(worldbody, "geom", {
        "name": "pbvs_floor", "type": "plane", "size": "2 2 .1",
        "rgba": ".85 .85 .85 1",
    })

    triangle = ET.SubElement(worldbody, "body", {
        "name": "pbvs_triangle", "mocap": "true", "pos": ".5 0 .4",
    })
    for attrs in [
        {"type": "box", "size": ".12 .09 .006", "rgba": ".15 .65 .20 .45"},
        {"type": "sphere", "size": ".012", "pos": "0 .055 .01", "rgba": "1 .1 .1 1"},
        {"type": "sphere", "size": ".012", "pos": "-.05 -.035 .01", "rgba": "1 .1 .1 1"},
        {"type": "sphere", "size": ".012", "pos": ".05 -.035 .01", "rgba": "1 .1 .1 1"},
    ]:
        attrs.update({"contype": "0", "conaffinity": "0"})
        ET.SubElement(triangle, "geom", attrs)
    ET.SubElement(triangle, "site", {
        "name": "pbvs_triangle_frame", "type": "sphere", "size": ".007",
        "rgba": "1 1 1 1", "group": "3",
    })
    add_axis_geoms(triangle, 0.08)

    output = panda_xml.parent / "_pbvs_generated_panda_holder_bracket.xml"
    tree.write(output, encoding="unicode")
    return output


class LatestPoseCommand:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose: Optional[np.ndarray] = None
        self._time = 0.0

    def set(self, pose: np.ndarray) -> None:
        with self._lock:
            self._pose = pose.copy()
            self._time = time.monotonic()

    def get(self) -> tuple[Optional[np.ndarray], float]:
        with self._lock:
            return (
                None if self._pose is None else self._pose.copy(),
                self._time,
            )


class CommandReceiver(threading.Thread):
    def __init__(self, bind_ip: str, port: int, latest: LatestPoseCommand) -> None:
        super().__init__(daemon=True)
        self.bind_ip = bind_ip
        self.port = port
        self.latest = latest

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.bind_ip, self.port))
        while True:
            data, _ = sock.recvfrom(1024)
            if len(data) != POSE_SIZE:
                continue
            pose = np.asarray(struct.unpack(POSE_FORMAT, data), dtype=float)
            if np.all(np.isfinite(pose)):
                self.latest.set(pose)


@dataclass
class ArmJoint:
    joint_id: int
    qpos_address: int
    dof_address: int
    minimum: float
    maximum: float


def discover_arm_joints(model: mujoco.MjModel) -> list[ArmJoint]:
    joints: list[ArmJoint] = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or ""
        if name.endswith(tuple(str(i) for i in range(1, 8))) or "joint" in name.lower():
            qpos_address = int(model.jnt_qposadr[joint_id])
            dof_address = int(model.jnt_dofadr[joint_id])
            minimum, maximum = model.jnt_range[joint_id]
            joints.append(
                ArmJoint(
                    joint_id,
                    qpos_address,
                    dof_address,
                    float(minimum),
                    float(maximum),
                )
            )

    joints.sort(key=lambda joint: joint.dof_address)
    if len(joints) < 7:
        raise RuntimeError(
            f"Found only {len(joints)} candidate hinge joints. "
            "Run inspect_model.py and pass a compatible Panda model."
        )
    return joints[:7]


def body_transform(data: mujoco.MjData, body_id: int) -> np.ndarray:
    return make_transform(
        data.xmat[body_id].reshape(3, 3).copy(),
        data.xpos[body_id].copy(),
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "MuJoCo Panda simulator with optional physical-robot "
            "end-effector mirroring."
        )
    )
    parser.add_argument(
        "--panda-xml",
        type=Path,
        required=True,
        help="Path to mujoco_menagerie/franka_emika_panda/panda.xml.",
    )
    parser.add_argument(
        "--pbvs-config",
        type=Path,
        required=True,
        help="PBVS JSON config containing T_EC, T_CS, and T_TS_des.",
    )
    parser.add_argument("--ee-body", default="hand")

    parser.add_argument("--command-bind-ip", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=2600)
    parser.add_argument("--command-timeout", type=float, default=0.25)

    parser.add_argument("--state-ip", default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=6200)
    parser.add_argument(
        "--publish-sim-state",
        action="store_true",
        help=(
            "Also publish simulated EE state while in physical mirror mode. "
            "It is already published by default in normal simulation mode."
        ),
    )

    parser.add_argument("--tracker-ip", default="127.0.0.1")
    parser.add_argument("--tracker-port", type=int, default=6500)
    parser.add_argument(
        "--disable-tracker-output",
        action="store_true",
        help=(
            "Do not publish T_TC. Use this when an external bridge computes "
            "tracker poses directly from physical Panda and triangle streams."
        ),
    )

    parser.add_argument(
        "--triangle-bind-ip",
        default="127.0.0.1",
        help="Address receiving streamed triangle poses in Panda base frame.",
    )
    parser.add_argument(
        "--triangle-port",
        type=int,
        default=6601,
        help="UDP port receiving triangle pose as <6f>.",
    )
    parser.add_argument(
        "--triangle-timeout",
        type=float,
        default=0.5,
        help=(
            "Maximum triangle-packet age in seconds. The triangle is hidden "
            "and T_TC publication stops when the stream is stale."
        ),
    )

    parser.add_argument("--damping", type=float, default=0.05)
    parser.add_argument("--max-joint-speed", type=float, default=0.4)
    parser.add_argument("--kp-position", type=float, default=3.0)
    parser.add_argument("--kp-orientation", type=float, default=2.0)

    parser.add_argument("--triangle-step", type=float, default=0.01)
    parser.add_argument("--triangle-rotation-step-deg", type=float, default=2.0)

    parser.add_argument(
        "--real-state-bind-ip",
        default=None,
        help=(
            "Enable digital-twin mode and bind here for physical Panda "
            "EE-state packets."
        ),
    )
    parser.add_argument(
        "--real-state-port",
        type=int,
        default=6201,
        help="UDP port receiving physical Panda EE state as <6f>.",
    )
    parser.add_argument(
        "--real-state-timeout",
        type=float,
        default=0.5,
        help="Maximum age in seconds of a physical-state packet.",
    )
    args = parser.parse_args()

    if args.real_state_timeout <= 0.0:
        parser.error("--real-state-timeout must be positive.")
    if args.triangle_timeout <= 0.0:
        parser.error("--triangle-timeout must be positive.")
    mirror_mode = args.real_state_bind_ip is not None
    publish_sim_state = (not mirror_mode) or args.publish_sim_state
    panda_xml = args.panda_xml.expanduser().resolve()
    config_path = args.pbvs_config.expanduser().resolve()

    (
        T_EC,
        T_CS,
        _T_TS_des,
        _T_TC_des,
        tool_visualization,
    ) = load_transform_config(config_path)

    scene_xml = create_scene_xml(
        panda_xml,
        args.ee_body,
        T_EC,
        T_CS,
        tool_visualization,
    )

    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)

    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)

    # Preserve the existing initial joint-7 offset used by this visualization.
    joint7_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "joint7",
    )
    if joint7_id < 0:
        raise RuntimeError("MuJoCo joint 'joint7' was not found.")

    joint7_qpos_address = int(model.jnt_qposadr[joint7_id])
    print(
        "joint7 before:",
        math.degrees(data.qpos[joint7_qpos_address]),
        "deg",
    )

    data.qpos[joint7_qpos_address] += math.radians(90.0)
    joint7_minimum, joint7_maximum = model.jnt_range[joint7_id]
    data.qpos[joint7_qpos_address] = np.clip(
        data.qpos[joint7_qpos_address],
        joint7_minimum + 0.01,
        joint7_maximum - 0.01,
    )
    mujoco.mj_forward(model, data)

    print(
        "joint7 after:",
        math.degrees(data.qpos[joint7_qpos_address]),
        "deg",
    )

    ee_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        args.ee_body,
    )
    if ee_body_id < 0:
        raise ValueError(
            f"EE body {args.ee_body!r} not found. Run inspect_model.py."
        )

    triangle_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "pbvs_triangle",
    )
    if triangle_body_id < 0:
        raise RuntimeError("Generated body 'pbvs_triangle' was not found.")

    camera_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "pbvs_camera",
    )
    tip_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "pbvs_stick_tip",
    )
    if camera_body_id < 0 or tip_body_id < 0:
        raise RuntimeError("Generated camera or stick-tip body was not found.")

    joints = discover_arm_joints(model)

    latest_command = LatestPoseCommand()
    latest_real_state = LatestPoseCommand()
    latest_triangle_pose = LatestPoseCommand()

    CommandReceiver(
        args.triangle_bind_ip,
        args.triangle_port,
        latest_triangle_pose,
    ).start()
    print(
        "Receiving streamed triangle pose in Panda base frame on "
        f"{args.triangle_bind_ip}:{args.triangle_port}"
    )

    if mirror_mode:
        CommandReceiver(
            args.real_state_bind_ip,
            args.real_state_port,
            latest_real_state,
        ).start()
        print(
            "Digital-twin mode: receiving physical Panda EE state on "
            f"{args.real_state_bind_ip}:{args.real_state_port}"
        )
        print("Waiting for fresh physical Panda state.")
    else:
        CommandReceiver(
            args.command_bind_ip,
            args.command_port,
            latest_command,
        ).start()
        print(
            "Normal simulation mode: receiving Cartesian commands on "
            f"{args.command_bind_ip}:{args.command_port}"
        )

    state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tracker_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # The external triangle stream is authoritative. Keep the triangle below
    # the floor until the first fresh packet arrives.
    triangle_initialized = False
    triangle_fresh = False
    triangle_pos = np.zeros(3, dtype=float)
    triangle_rpy = np.zeros(3, dtype=float)

    data.mocap_pos[0] = np.array([0.0, 0.0, -1.0])
    data.mocap_quat[0] = np.array([1.0, 0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)

    def key_callback(_keycode: int) -> None:
        # Triangle motion is controlled exclusively by the UDP stream.
        return

    target_transform = body_transform(data, ee_body_id)
    previous_time = time.monotonic()
    last_state_send = 0.0
    last_tracker_send = 0.0
    last_status = 0.0
    last_waiting_print = 0.0

    real_state_fresh = False

    singularity_recovery_active = False
    last_ik_warning_time = 0.0

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    print("Simulation started.")
    print("Triangle is controlled by the external UDP pose stream.")
    print(f"EE body: {args.ee_body}; generated scene: {scene_xml}")
    if mirror_mode and not publish_sim_state:
        print(
            "Simulated EE-state publication is disabled in mirror mode "
            "to avoid mixing it with the physical state stream."
        )

    try:
        with mujoco.viewer.launch_passive(
            model,
            data,
            key_callback=key_callback,
        ) as viewer:
            while viewer.is_running():
                now = time.monotonic()
                dt = min(max(now - previous_time, 1e-4), 0.02)
                previous_time = now

                triangle_pose, triangle_pose_time = latest_triangle_pose.get()
                triangle_fresh = (
                    triangle_pose is not None
                    and now - triangle_pose_time <= args.triangle_timeout
                )

                if triangle_fresh:
                    triangle_pos[:] = triangle_pose[:3]
                    triangle_rpy[:] = triangle_pose[3:]
                    data.mocap_pos[0] = triangle_pos
                    data.mocap_quat[0] = quat_wxyz_from_rotation(
                        rotation_from_rpy(*triangle_rpy)
                    )
                    if not triangle_initialized:
                        print(
                            "Fresh triangle stream received; triangle is now "
                            "visible and T_TC publication is enabled."
                        )
                    triangle_initialized = True
                else:
                    if triangle_initialized:
                        print(
                            "Triangle stream is stale; hiding triangle and "
                            "pausing T_TC publication."
                        )
                    triangle_initialized = False
                    data.mocap_pos[0] = np.array([0.0, 0.0, -1.0])
                    data.mocap_quat[0] = np.array([1.0, 0.0, 0.0, 0.0])

                mujoco.mj_forward(model, data)

                current_transform = body_transform(data, ee_body_id)

                if mirror_mode:
                    real_pose, real_pose_time = latest_real_state.get()
                    real_state_fresh = (
                        real_pose is not None
                        and now - real_pose_time <= args.real_state_timeout
                    )

                    if real_state_fresh:
                        target_transform = make_transform(
                            rotation_from_rpy(
                                real_pose[3],
                                real_pose[4],
                                real_pose[5],
                            ),
                            real_pose[:3],
                        )
                    else:
                        target_transform = current_transform
                        if now - last_waiting_print >= 1.0:
                            print(
                                "Waiting for fresh physical Panda state on "
                                f"{args.real_state_bind_ip}:"
                                f"{args.real_state_port}"
                            )
                            last_waiting_print = now
                else:
                    command, command_time = latest_command.get()
                    command_is_fresh = (
                        command is not None
                        and now - command_time <= args.command_timeout
                    )

                    if command_is_fresh:
                        target_transform = make_transform(
                            rotation_from_rpy(
                                command[3],
                                command[4],
                                command[5],
                            ),
                            command[:3],
                        )
                    else:
                        target_transform = current_transform

                T_BE = current_transform
                position_error = target_transform[:3, 3] - T_BE[:3, 3]
                orientation_error = so3_log(
                    target_transform[:3, :3] @ T_BE[:3, :3].T
                )

                # Limit the Cartesian error used by IK so initialization cannot
                # generate an excessive jump from a distant starting pose.
                position_error_limited = position_error.copy()
                position_norm = np.linalg.norm(position_error_limited)
                if position_norm > 0.05:
                    position_error_limited *= 0.05 / position_norm

                orientation_error_limited = orientation_error.copy()
                orientation_norm = np.linalg.norm(
                    orientation_error_limited
                )
                if orientation_norm > math.radians(15.0):
                    orientation_error_limited *= (
                        math.radians(15.0) / orientation_norm
                    )

                twist = np.concatenate([
                    args.kp_position * position_error_limited,
                    args.kp_orientation * orientation_error_limited,
                ])

                mujoco.mj_jacBody(
                    model,
                    data,
                    jacp,
                    jacr,
                    ee_body_id,
                )
                jacobian = np.vstack([jacp, jacr])
                arm_columns = [joint.dof_address for joint in joints]
                arm_jacobian = jacobian[:, arm_columns]

                singular_values = np.linalg.svd(
                    arm_jacobian,
                    compute_uv=False,
                )
                sigma_min = float(singular_values[-1])

                sigma_threshold = 0.08
                lambda_min = max(0.001, min(args.damping, 0.30))
                lambda_max = 0.30

                if sigma_min >= sigma_threshold:
                    damping = lambda_min
                else:
                    ratio = np.clip(
                        1.0 - sigma_min / sigma_threshold,
                        0.0,
                        1.0,
                    )
                    damping = lambda_min + (
                        lambda_max - lambda_min
                    ) * ratio * ratio

                regularized = (
                    arm_jacobian @ arm_jacobian.T
                    + damping**2 * np.eye(6)
                )
                damped_inverse = arm_jacobian.T @ np.linalg.solve(
                    regularized,
                    np.eye(6),
                )

                qdot_task = damped_inverse @ twist
                q_current = np.array(
                    [
                        data.qpos[joint.qpos_address]
                        for joint in joints
                    ],
                    dtype=float,
                )
                nullspace = (
                    np.eye(7) - damped_inverse @ arm_jacobian
                )

                joint_limit_velocity = np.zeros(7)
                limit_margin = math.radians(20.0)
                limit_gain = 0.6

                for index, joint in enumerate(joints):
                    q = q_current[index]
                    distance_to_min = q - joint.minimum
                    distance_to_max = joint.maximum - q

                    if distance_to_min < limit_margin:
                        ratio = np.clip(
                            (
                                limit_margin - distance_to_min
                            ) / limit_margin,
                            0.0,
                            1.0,
                        )
                        joint_limit_velocity[index] += (
                            limit_gain * ratio * ratio
                        )

                    if distance_to_max < limit_margin:
                        ratio = np.clip(
                            (
                                limit_margin - distance_to_max
                            ) / limit_margin,
                            0.0,
                            1.0,
                        )
                        joint_limit_velocity[index] -= (
                            limit_gain * ratio * ratio
                        )

                qdot = (
                    qdot_task
                    + nullspace @ joint_limit_velocity
                )

                singularity_enter_threshold = 0.020
                singularity_exit_threshold = 0.045

                if sigma_min < singularity_enter_threshold:
                    singularity_recovery_active = True
                elif (
                    singularity_recovery_active
                    and sigma_min > singularity_exit_threshold
                ):
                    singularity_recovery_active = False

                if not np.all(np.isfinite(qdot)):
                    qdot = np.zeros(7)
                    singularity_recovery_active = True

                if singularity_recovery_active:
                    joint_centers = np.array(
                        [
                            0.5 * (joint.minimum + joint.maximum)
                            for joint in joints
                        ],
                        dtype=float,
                    )
                    qdot_recovery = 0.35 * (
                        joint_centers - q_current
                    )
                    qdot = nullspace @ qdot_recovery
                    qdot = np.clip(qdot, -0.35, 0.35)

                    if now - last_ik_warning_time > 0.5:
                        print(
                            "IK RECOVERY: "
                            f"sigma_min={sigma_min:.4f}, "
                            "|raw_recovery|="
                            f"{np.linalg.norm(qdot_recovery):.3f}, "
                            "|projected_qdot|="
                            f"{np.linalg.norm(qdot):.3f}"
                        )
                        last_ik_warning_time = now

                qdot = np.clip(
                    qdot,
                    -args.max_joint_speed,
                    args.max_joint_speed,
                )
                joint_step = np.clip(
                    qdot * dt,
                    -math.radians(0.5),
                    math.radians(0.5),
                )

                joint_margin = math.radians(3.0)
                for joint, increment in zip(joints, joint_step):
                    data.qpos[joint.qpos_address] += float(increment)
                    data.qpos[joint.qpos_address] = np.clip(
                        data.qpos[joint.qpos_address],
                        joint.minimum + joint_margin,
                        joint.maximum - joint_margin,
                    )

                mujoco.mj_forward(model, data)
                T_BE = body_transform(data, ee_body_id)

                if publish_sim_state and now - last_state_send >= 0.002:
                    rpy = rpy_from_rotation(T_BE[:3, :3])
                    state = np.concatenate([T_BE[:3, 3], rpy])
                    state_socket.sendto(
                        struct.pack(
                            POSE_FORMAT,
                            *state.astype(np.float32),
                        ),
                        (args.state_ip, args.state_port),
                    )
                    last_state_send = now

                # Do not publish a tracker pose before the triangle has been
                # initialized relative to the synchronized physical robot.
                tracker_ready = (
                    not args.disable_tracker_output
                    and triangle_initialized
                    and (not mirror_mode or real_state_fresh)
                )
                if (
                    tracker_ready
                    and now - last_tracker_send >= 0.01
                ):
                    T_BC = T_BE @ T_EC
                    T_BT = body_transform(data, triangle_body_id)
                    T_TC = np.linalg.inv(T_BT) @ T_BC

                    tracker_socket.sendto(
                        struct.pack(
                            TRACKER_FORMAT,
                            *T_TC.reshape(-1),
                        ),
                        (args.tracker_ip, args.tracker_port),
                    )
                    last_tracker_send = now

                if now - last_status >= 1.0:
                    T_BC_visual = body_transform(
                        data,
                        camera_body_id,
                    )
                    T_BS_visual = body_transform(
                        data,
                        tip_body_id,
                    )

                    mode_status = "simulation"
                    if mirror_mode:
                        mode_status = (
                            "mirror-following"
                            if real_state_fresh
                            else "mirror-waiting-for-state"
                        )

                    triangle_status = (
                        "fresh" if triangle_fresh else "waiting/stale"
                    )

                    status = (
                        f"mode={mode_status}, triangle={triangle_status}, "
                        f"current_EE_xyz="
                        f"{np.round(T_BE[:3, 3], 3)}, "
                        f"camera_xyz="
                        f"{np.round(T_BC_visual[:3, 3], 3)}, "
                        f"tip_xyz="
                        f"{np.round(T_BS_visual[:3, 3], 3)}\n"
                        "inner_EE_error: "
                        f"|e_p|={np.linalg.norm(position_error):.4f} m, "
                        "|e_R|="
                        f"{math.degrees(np.linalg.norm(orientation_error)):.2f} deg, "
                        f"sigma_min={sigma_min:.4f}"
                    )

                    if triangle_fresh:
                        T_BT_visual = body_transform(data, triangle_body_id)
                        tip_triangle_distance = float(
                            np.linalg.norm(
                                T_BT_visual[:3, 3]
                                - T_BS_visual[:3, 3]
                            )
                        )
                        camera_triangle_distance = float(
                            np.linalg.norm(
                                T_BT_visual[:3, 3]
                                - T_BC_visual[:3, 3]
                            )
                        )
                        status += (
                            "\ntriangle_xyz="
                            f"{np.round(T_BT_visual[:3, 3], 3)}, "
                            f"tip_distance={tip_triangle_distance:.4f} m, "
                            "camera_distance="
                            f"{camera_triangle_distance:.4f} m"
                        )

                    print(status)
                    last_status = now

                viewer.sync()
                time.sleep(max(model.opt.timestep, 0.001))
    finally:
        state_socket.close()
        tracker_socket.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

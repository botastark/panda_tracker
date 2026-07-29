#!/usr/bin/env python3
"""Inspect joint, body, site, actuator, keyframe, and sensor names in a MuJoCo model."""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco


def print_named_objects(model: mujoco.MjModel, title: str, object_type, count: int) -> None:
    print(f"\n{title} ({count})")
    print("-" * (len(title) + 4))
    for object_id in range(count):
        name = mujoco.mj_id2name(model, object_type, object_id)
        print(f"{object_id:3d}: {name}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "model",
        type=Path,
        help="Path to panda.xml or scene.xml.",
    )
    args = parser.parse_args()

    model_path = args.model.expanduser().resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))

    print(f"Loaded: {model_path}")
    print(f"nq={model.nq}, nv={model.nv}, nu={model.nu}, nbody={model.nbody}")

    print_named_objects(model, "Joints", mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    print_named_objects(model, "Bodies", mujoco.mjtObj.mjOBJ_BODY, model.nbody)
    print_named_objects(model, "Sites", mujoco.mjtObj.mjOBJ_SITE, model.nsite)
    print_named_objects(model, "Actuators", mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
    print_named_objects(model, "Keyframes", mujoco.mjtObj.mjOBJ_KEY, model.nkey)
    print_named_objects(model, "Sensors", mujoco.mjtObj.mjOBJ_SENSOR, model.nsensor)

    print("\nHinge joints suitable for the 7-DoF arm")
    print("----------------------------------------")
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_HINGE:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            print(
                f"{name}: qposadr={model.jnt_qposadr[joint_id]}, "
                f"dofadr={model.jnt_dofadr[joint_id]}, "
                f"range={model.jnt_range[joint_id].tolist()}"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

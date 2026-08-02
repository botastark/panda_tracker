#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from backends.mujoco_udp import MujocoUdpBackend
from backends.panda_udp import PandaUdpBackend
from common.config import load_pbvs_config
from common.geometry import invert_transform
from common.tracking_precision_logger import TrackingPrecisionLogger
from control.pbvs_controller import ControllerState, PBVSController
from perception.task_pose_udp import TaskPoseUdpSource


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="PBVS controller for simulation or Panda"
    )
    parser.add_argument(
        "--backend",
        choices=("sim", "panda"),
        required=True,
    )
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--tracker-bind-ip",
        default="0.0.0.0",
        help="Local IP for receiving T_TS UDP packets.",
    )
    parser.add_argument(
        "--tracker-port",
        type=int,
        default=6501,
        help="UDP port for receiving T_TS packets.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute commands but do not send them.",
    )
    parser.add_argument(
        "--tracking-log",
        type=Path,
        default=None,
        help=(
            "Optional CSV path for T_TS tracking precision samples. "
            "A .summary.json file is also written when the controller stops."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    raw = json.loads(args.config.read_text())
    config = load_pbvs_config(args.config)

    identity = (
        config.T_ES
        @ invert_transform(config.T_ES)
    )

    assert np.allclose(
        identity,
        np.eye(4),
        atol=1e-9,
    ), "T_ES inverse consistency check failed."

    if args.backend == "sim":
        backend = MujocoUdpBackend(
            panda_ip=raw["panda_ip"],
            command_port=int(
                raw["panda_command_port"]
            ),
            state_bind_ip=raw[
                "panda_state_bind_ip"
            ],
            state_port=int(
                raw["panda_state_port"]
            ),
        )
    else:
        backend = PandaUdpBackend(
            panda_ip=raw["panda_ip"],
            command_port=int(
                raw["panda_command_port"]
            ),
            state_bind_ip=raw[
                "panda_state_bind_ip"
            ],
            state_port=int(
                raw["panda_state_port"]
            ),
        )

    task_pose_source = TaskPoseUdpSource(
        args.tracker_bind_ip,
        args.tracker_port,
    )

    controller = PBVSController(config)
    tracking_logger = (
        TrackingPrecisionLogger(args.tracking_log)
        if args.tracking_log is not None
        else None
    )

    if tracking_logger is not None:
        print(f"Tracking precision log: {args.tracking_log}")

    period = 1.0 / config.control_rate_hz
    previous = time.monotonic()

    startup_deadline = time.monotonic() + 5.0
    seen_first_task_pose = False

    print(
        f"Backend: {args.backend}; "
        f"dry_run={args.dry_run}"
    )
    print(
        "Receiving T_TS on "
        f"{args.tracker_bind_ip}:"
        f"{args.tracker_port}"
    )
    print("Ctrl-C to stop.")

    try:
        while True:
            loop_start = time.monotonic()

            dt = loop_start - previous
            previous = loop_start

            # Protect the controller from unusually small or large dt.
            dt = max(
                min(dt, 0.1),
                1e-6,
            )

            T_BE, robot_state_age = (
                backend.get_current_pose()
            )

            # Read the latest task measurement exactly once.
            task_pose = task_pose_source.get_latest()

            if args.backend == "panda":
                if task_pose is not None:
                    seen_first_task_pose = True
                elif not seen_first_task_pose and time.monotonic() > startup_deadline:
                    print(
                        "\nERROR: No T_TS received on "
                        f"{args.tracker_bind_ip}:{args.tracker_port} "
                        "within startup timeout in Panda mode."
                    )
                    print("Check that the real tracker is running and publishing T_TS.")
                    return 1

            command, diagnostics = controller.step(
                T_BE=T_BE,
                robot_state_age=robot_state_age,
                task_pose=task_pose,
                dt=dt,
            )
            if (
                tracking_logger is not None
                and task_pose is not None
            ):
                tracking_logger.log(
                    T_TS=task_pose.T_TS,
                    T_TS_des=config.T_TS_des,
                    controller_state=diagnostics.state.name,
                    reason=diagnostics.reason,
                    robot_state_age=robot_state_age,
                    command_sent=tracking_command_sent(
                        command=command,
                        dry_run=args.dry_run,
                        controller_state=diagnostics.state,
                    ),
                )

            if (
                command is not None
                and not args.dry_run
            ):
                backend.send_target_pose(command)


            elapsed = time.monotonic() - loop_start
            time.sleep(
                max(
                    0.0,
                    period - elapsed,
                )
            )

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        if tracking_logger is not None:
            tracking_logger.close()

        task_pose_source.close()
        backend.close()

    return 0
def tracking_command_sent(
    *,
    command: np.ndarray | None,
    dry_run: bool,
    controller_state: ControllerState,
) -> bool:
    """
    Return True only when an active tracking command was transmitted.

    READY and HOLD may still transmit a pose to maintain the robot's
    current position, but those are not active tracking commands.
    """

    return (
        command is not None
        and not dry_run
        and controller_state is ControllerState.TRACKING
    )


if __name__ == "__main__":
    raise SystemExit(main())

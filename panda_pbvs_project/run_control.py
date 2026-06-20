#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from backends.mujoco_udp import MujocoUdpBackend
from backends.panda_udp import PandaUdpBackend
from common.config import load_pbvs_config
from control.pbvs_controller import PBVSController
from perception.tracker_udp import TrackerUdpSource


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PBVS controller for simulation or Panda")
    parser.add_argument("--backend", choices=("sim", "panda"), required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--tracker-bind-ip", default="0.0.0.0")
    parser.add_argument("--tracker-port", type=int, default=6500)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    raw = json.loads(args.config.read_text())
    config = load_pbvs_config(args.config)

    if args.backend == "sim":
        backend = MujocoUdpBackend(
            panda_ip=raw["panda_ip"],
            command_port=int(raw["panda_command_port"]),
            state_bind_ip=raw["panda_state_bind_ip"],
            state_port=int(raw["panda_state_port"]),
        )
    else:
        backend = PandaUdpBackend(
            panda_ip=raw["panda_ip"],
            command_port=int(raw["panda_command_port"]),
            state_bind_ip=raw["panda_state_bind_ip"],
            state_port=int(raw["panda_state_port"]),
        )

    tracker = TrackerUdpSource(args.tracker_bind_ip, args.tracker_port)
    controller = PBVSController(config)

    period = 1.0 / config.control_rate_hz
    previous = time.monotonic()
    last_print = 0.0

    print(f"Backend: {args.backend}; dry_run={args.dry_run}")
    print("Ctrl-C to stop.")

    try:
        while True:
            now = time.monotonic()
            dt = max(min(now - previous, 0.1), period)
            previous = now

            T_BE, age = backend.get_current_pose()
            command, diagnostics = controller.step(
                T_BE=T_BE,
                robot_state_age=age,
                tracker=tracker.get_latest(),
                dt=dt,
            )

            if command is not None and not args.dry_run:
                backend.send_target_pose(command)

            if now - last_print > 0.5:
                print(
                    f"state={diagnostics.state.name}, "
                    f"|e_p|={diagnostics.position_error:.4f} m, "
                    f"|e_R|={math.degrees(diagnostics.orientation_error):.2f} deg, "
                    f"reason={diagnostics.reason or '-'}"
                )
                last_print = now

            time.sleep(max(0.0, period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        tracker.close()
        backend.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

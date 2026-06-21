# Panda event-camera PBVS project

One PBVS controller is shared between simulation and the real Panda. Only the
backend and configuration file change.

## Structure

```text
panda_pbvs_project/
├── common/
├── control/
├── backends/
├── perception/
├── simulation/
├── configs/
└── run_control.py
```

## Simulation

Run the MuJoCo simulator process first. It must expose:

- command port 2600;
- EE state port 6200;
- synthetic `T_TC` port 6500.

Then run:

```bash
python run_control.py \
  --backend sim \
  --config configs/pbvs_sim.json \
  --tracker-bind-ip 127.0.0.1 \
  --tracker-port 6500
```

## Real Panda dry run

Start `explorer_safe`, but keep the robot supervised and initially use:

```bash
python run_control.py \
  --backend panda \
  --config configs/pbvs_robot.json \
  --tracker-bind-ip 0.0.0.0 \
  --tracker-port 6500 \
  --dry-run
```

Dry-run receives both Panda state and tracker data but sends no commands.

## Real Panda active run

Only after dry-run validation:

```bash
python run_control.py \
  --backend panda \
  --config configs/pbvs_robot.json \
  --tracker-bind-ip 0.0.0.0 \
  --tracker-port 6500
```

## Critical rule

Never use `configs/pbvs_sim.json` on the physical robot. The simulation speed,
error, and jump thresholds are intentionally permissive.

## Network note

`configs/pbvs_robot.json` currently sends commands to `172.16.222.48`, the
Panda PC address used in the existing setup. Change this only if
`explorer_safe` runs on a different host. Do not replace it with the robot
control-box address `172.16.0.2`; the Python client talks to the UDP bridge,
not directly to libfranka.

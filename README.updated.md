# Panda PBVS Tracker

Position-based visual servoing (PBVS) for a Franka Panda, with a MuJoCo
simulation path, a physical-robot backend, direct `T_TS` tracking, and
target-motion feedforward.

The controller expects the current stick-tip pose expressed in the triangle
frame:

```text
T_TS
```

For an eye-in-hand visual tracker that estimates the triangle pose in the
camera frame (`T_CT`), convert it before publishing:

```text
T_ET = T_EC @ T_CT
T_TS = inverse(T_ET) @ T_ES
```

`panda_pbvs_project/perception/task_pose_adapter.py` implements and tests this
conversion. The controller itself still receives `T_TS`; it does not call the
adapter automatically.

## Current safety status

The checked-in simulation and physical configurations are intentionally
different.

```text
configs/pbvs_sim.json
  target_feedforward_enabled = true
  target_velocity_filter_alpha = 0.25
  max_angular_speed_deg = 6.0
  max_target_angular_speed_deg = 6.0

configs/pbvs_robot.json
  target_feedforward_enabled = false
  max_angular_speed_deg = 2.0
```

Do not copy simulation tuning into `pbvs_robot.json`.

The physical configuration currently contains:

```text
tool_geometry_status =
nominal_mechanical_geometry_do_not_enable_robot_until_calibrated
```

Do not enable physical tracking until the tool geometry, hand-eye transform,
workspace, payload, and robot frame convention have been validated.

## Repository setup

Run commands from the repository root unless a section says otherwise:

```bash
cd ~/repos/panda_tracker
source .venv/bin/activate
```

The Panda model is a Git submodule. Initialize it once:

```bash
git submodule update --init --recursive
```

Verify the required XML before starting MuJoCo:

```bash
PANDA_XML="$PWD/mujoco_menagerie/franka_emika_panda/panda.xml"

if [[ ! -f "$PANDA_XML" ]]; then
  echo "ERROR: missing $PANDA_XML" >&2
  echo "Run: git submodule update --init --recursive" >&2
  exit 1
fi

echo "Using Panda XML: $PANDA_XML"
```

## UDP ports

### Simulation-only mode

```text
2600  MuJoCo Cartesian command input
6200  simulated Panda state output -> run_control.py
6501  simulated task-pose output -> run_control.py
6601  triangle trajectory input -> MuJoCo
```

### Physical Panda with digital twin

```text
Physical explorer
    |
    v
0.0.0.0:6200  udp_pose_fanout.py
    |---> 127.0.0.1:6201  run_control.py robot state
    `---> 127.0.0.1:6202  MuJoCo digital-twin state

External visual tracker
    `---> 127.0.0.1:6501  run_control.py task pose

run_control.py
    `---> physical Panda command port 2600
```

In mirror mode the MuJoCo process is started with
`--disable-task-pose-output`; the real visual tracker is authoritative.

Check port ownership before starting processes:

```bash
sudo ss -lunp | \
  grep -E ':(2600|6200|6201|6202|6501|6601)\b' || true
```

## Simulation-only PBVS validation

Use three terminals.

### Terminal 1: MuJoCo simulator

From the repository root:

```bash
cd ~/repos/panda_tracker
source .venv/bin/activate

PANDA_XML="$PWD/mujoco_menagerie/franka_emika_panda/panda.xml"
test -f "$PANDA_XML" || {
  echo "Missing Panda XML; initialize the submodule." >&2
  exit 1
}

python simulation/simulated_explorer_tool.py \
  --panda-xml "$PANDA_XML" \
  --pbvs-config panda_pbvs_project/configs/pbvs_sim.json
```

For a headless run, use a duration long enough to start the controller and
trajectory:

```bash
python simulation/simulated_explorer_tool.py \
  --panda-xml "$PANDA_XML" \
  --pbvs-config panda_pbvs_project/configs/pbvs_sim.json \
  --headless \
  --headless-duration 180
```

`--panda-xml` and `--pbvs-config` are required. Headless mode otherwise
defaults to a short run.

### Terminal 2: PBVS controller and tracking log

```bash
cd ~/repos/panda_tracker
source .venv/bin/activate

mkdir -p panda_pbvs_project/results

python panda_pbvs_project/run_control.py \
  --backend sim \
  --config panda_pbvs_project/configs/pbvs_sim.json \
  --tracker-bind-ip 127.0.0.1 \
  --tracker-port 6501 \
  --tracking-log panda_pbvs_project/results/sim_baseline.csv
```

When stopped with `Ctrl-C`, the controller writes:

```text
panda_pbvs_project/results/sim_baseline.csv
panda_pbvs_project/results/sim_baseline.csv.summary.json
```

### Terminal 3: predefined triangle trajectory

```bash
cd ~/repos/panda_tracker
source .venv/bin/activate

python panda_pbvs_project/tests/predefined_triangle_trajectory.py \
  --destination 127.0.0.1:6601 \
  --rate 30 \
  --repeat 1 \
  --initial-hold 15 \
  --center-pose \
    -0.000000029 \
    0.554499507 \
    0.294352422 \
    0.000005009 \
    0.0 \
    89.994374429
```

The center pose above is the validated equilibrium for the current model and
tool configuration. Recompute it after changing the model, end-effector frame,
`T_ES`, `T_EC`, `T_CS`, or `T_TS_des`.

Expected behavior:

```text
triangle stream starts       -> READY -> TRACKING
unique task poses continue   -> TRACKING
triangle stream ends         -> task-pose publication stops
tracker_timeout expires      -> HOLD / task_pose_stale
```

Stop the controller soon after the stale-data transition so repeated runs have
similar durations.

## Task-pose protocol

The minimal version-2 task-pose packet contains:

```text
magic/version
valid
sequence_id
confidence
T_TS
```

Required sender behavior:

```text
new tracker estimate       -> increment sequence_id
same estimate republished  -> keep the same sequence_id
invalid estimate           -> publish valid=false
low-quality estimate       -> publish reduced confidence
```

The receiver ignores duplicate and older version-2 sequence numbers. Ignored
packets do not refresh the local measurement timestamp, allowing the existing
PBVS controller to enter `HOLD` when the tracker freezes.

Current limitations:

- Legacy matrix-only packets remain accepted, but cannot provide duplicate
  detection.
- The minimal packet has no stream/session ID. If a tracker process restarts
  and resets `sequence_id`, restart `run_control.py` as well.
- The receiver confidence threshold currently defaults to `0.5`; it is not yet
  exposed as a JSON configuration field.
- `task_pose_adapter.py` is not wired into `run_control.py`. A visual-tracker
  bridge that starts from `T_CT` must perform the conversion before sending
  `T_TS`.

## Tests

From `panda_pbvs_project`:

```bash
cd ~/repos/panda_tracker/panda_pbvs_project
source ../.venv/bin/activate

python -m unittest discover \
  -s tests \
  -p 'test_*.py' \
  -v

python -m compileall \
  common \
  control \
  perception \
  tests \
  run_control.py

python -m py_compile \
  ../simulation/simulated_explorer_tool.py

git diff --check
```

Focused tests for the current tracker integration:

```bash
python tests/test_task_pose_adapter.py
python tests/test_task_pose_udp.py
python tests/test_target_feedforward.py
python tests/test_tracking_performance.py
python tests/test_run_control_logging.py
python tests/test_checked_in_configs.py
```

## Parameter tuning with reproducible A/B tests

Tune in simulation only. Keep `pbvs_robot.json` unchanged.

### Tuning rules

1. Change one parameter or one tightly coupled pair at a time.
2. Use the same simulator, center pose, trajectory, rate, repeat count, and
   initial hold for every run.
3. Record a baseline before creating a candidate.
4. Use a new log file for every run.
5. Compare `TRACKING` samples only.
6. Repeat the best candidate at least once before keeping it.
7. Do not relax safety thresholds merely to improve the score.

Primary metrics, all lower-is-better:

```text
position RMS
position p95
orientation RMS
orientation p95
```

Also inspect maxima, tracking sample count, and controller state/reason
distribution. Reject candidates that improve averages by producing frequent
`HOLD`, stale-state, jump-rejection, or robot-state dropouts.

### Create a candidate simulation config

Place `make_tuning_config.py` in `panda_pbvs_project/tools/`, then run:

```bash
mkdir -p panda_pbvs_project/configs/tuning

python panda_pbvs_project/tools/make_tuning_config.py \
  --base panda_pbvs_project/configs/pbvs_sim.json \
  --output panda_pbvs_project/configs/tuning/alpha_035.json \
  --set target_velocity_filter_alpha=0.35
```

The helper refuses a non-localhost base config unless explicitly overridden.

Example coupled angular-limit candidate:

```bash
python panda_pbvs_project/tools/make_tuning_config.py \
  --base panda_pbvs_project/configs/pbvs_sim.json \
  --output panda_pbvs_project/configs/tuning/angular_5deg.json \
  --set max_angular_speed_deg=5.0 \
  --set max_target_angular_speed_deg=5.0
```

### Run the baseline

Start the simulator with the baseline config:

```bash
python simulation/simulated_explorer_tool.py \
  --panda-xml "$PANDA_XML" \
  --pbvs-config panda_pbvs_project/configs/pbvs_sim.json \
  --headless \
  --headless-duration 180
```

Start the controller:

```bash
python panda_pbvs_project/run_control.py \
  --backend sim \
  --config panda_pbvs_project/configs/pbvs_sim.json \
  --tracker-bind-ip 127.0.0.1 \
  --tracker-port 6501 \
  --tracking-log panda_pbvs_project/results/baseline.csv
```

Run the predefined trajectory exactly as shown in the simulation section.

### Run the candidate

Restart the simulator and controller. Use the candidate config in both
processes:

```bash
python simulation/simulated_explorer_tool.py \
  --panda-xml "$PANDA_XML" \
  --pbvs-config panda_pbvs_project/configs/tuning/alpha_035.json \
  --headless \
  --headless-duration 180
```

```bash
python panda_pbvs_project/run_control.py \
  --backend sim \
  --config panda_pbvs_project/configs/tuning/alpha_035.json \
  --tracker-bind-ip 127.0.0.1 \
  --tracker-port 6501 \
  --tracking-log panda_pbvs_project/results/alpha_035.csv
```

Run the identical trajectory.

### Compare results

Place `compare_tracking_summaries.py` in `panda_pbvs_project/tools/`, then run:

```bash
python panda_pbvs_project/tools/compare_tracking_summaries.py \
  panda_pbvs_project/results/baseline.csv.summary.json \
  panda_pbvs_project/results/alpha_035.csv.summary.json
```

A positive improvement percentage means the candidate error is lower. The
comparison exits nonzero when the candidate has too few tracking samples or
its position/orientation RMS exceeds the allowed regression tolerance.

Useful starting parameters:

| Parameter | Purpose | Practical tuning note |
|---|---|---|
| `target_velocity_filter_alpha` | Feedforward estimate smoothing | Lower is smoother but adds lag; higher reacts faster but amplifies noise |
| `kp_position` | Translational correction | Increase gradually; watch command saturation and overshoot |
| `kp_orientation` | Rotational correction | Tune after translation is stable |
| `max_linear_speed` | Translational command clamp | Raise only when the command is consistently saturated |
| `max_angular_speed_deg` | Rotational command clamp | Simulation value is currently 6 deg/s; physical remains 2 deg/s |
| `max_target_linear_speed` | Feedforward estimate clamp | Set from plausible target motion, not from desired score |
| `max_target_angular_speed_deg` | Angular feedforward clamp | Keep consistent with the tested trajectory peak |
| `max_command_lead` | Maximum commanded pose lead | Safety limit; do not use as a general gain |
| `tracker_timeout` | Stale tracker timeout | Safety/reliability parameter, not a tracking-performance gain |
| `consecutive_valid_required` | Reacquisition confirmation | Higher is safer but delays tracking start |

Previously validated simulation tuning:

```text
target_feedforward_enabled = true
target_velocity_filter_alpha = 0.25
max_angular_speed_deg = 6.0
max_target_angular_speed_deg = 6.0
```

The 0.25 filter setting outperformed 0.50 in the tested trajectory. Increasing
the simulation angular limit from 2 deg/s to 6 deg/s substantially improved
both position and orientation tracking. These values remain simulation-only.

## Physical Panda with MuJoCo digital twin

### 1. Verify physical configuration

`panda_pbvs_project/configs/pbvs_robot.json` should retain:

```json
"panda_ip": "172.16.222.48",
"panda_state_bind_ip": "127.0.0.1",
"panda_state_port": 6201,
"target_feedforward_enabled": false,
"max_angular_speed_deg": 2.0
```

Do not continue until calibration and physical safety requirements are met.

### 2. Start the UDP state fan-out

From `panda_pbvs_project`:

```bash
cd ~/repos/panda_tracker/panda_pbvs_project
source ../.venv/bin/activate

python udp_pose_fanout.py \
  --bind-ip 0.0.0.0 \
  --bind-port 6200 \
  --destination 127.0.0.1:6201 \
  --destination 127.0.0.1:6202
```

Expected packet format:

```text
24 bytes (<6f)
```

### 3. Start the physical Panda explorer

```bash
cd /opt/libfranka/fe_panda
../build/fe_panda/explorer 1 0
```

### 4. Start the MuJoCo digital twin

From the repository root:

```bash
cd ~/repos/panda_tracker
source .venv/bin/activate

PANDA_XML="$PWD/mujoco_menagerie/franka_emika_panda/panda.xml"
test -f "$PANDA_XML" || exit 1

python simulation/simulated_explorer_tool.py \
  --panda-xml "$PANDA_XML" \
  --pbvs-config panda_pbvs_project/configs/pbvs_robot.json \
  --real-state-bind-ip 127.0.0.1 \
  --real-state-port 6202 \
  --disable-task-pose-output
```

### 5. Start the external visual tracker

The external tracker must publish valid `T_TS` packets to:

```text
127.0.0.1:6501
```

When the visual system produces `T_CT`, its bridge must first use
`task_pose_from_camera_target()` to create `T_TS`.

Start the tracker before starting `run_control.py`; Panda mode exits if no task
pose is received during its startup timeout.

### 6. Start the PBVS controller

```bash
cd ~/repos/panda_tracker
source .venv/bin/activate

python panda_pbvs_project/run_control.py \
  --backend panda \
  --config panda_pbvs_project/configs/pbvs_robot.json \
  --tracker-bind-ip 127.0.0.1 \
  --tracker-port 6501 \
  --dry-run \
  --tracking-log panda_pbvs_project/results/physical_dry_run.csv
```

Begin with `--dry-run`. Remove it only after verifying transforms, state ages,
workspace limits, tracker validity, stale-data behavior, and commanded motion
direction.

## Troubleshooting

### Simulator says required arguments are missing

Always provide:

```text
--panda-xml
--pbvs-config
```

### `IsADirectoryError` for the repository root

The Panda XML variable was empty and resolved to the current directory. Use
the fixed repository submodule path and validate it:

```bash
PANDA_XML="$PWD/mujoco_menagerie/franka_emika_panda/panda.xml"
test -f "$PANDA_XML" || exit 1
```

### Address already in use

```bash
sudo ss -lunp | \
  grep -E ':(2600|6200|6201|6202|6501|6601)\b'
```

Stop the process that owns the conflicting port. Do not start two processes
that bind the same UDP port.

### Digital twin reports infinite synchronization error

No valid physical state reached port `6202`.

```bash
sudo tcpdump -ni any 'udp dst port 6200'
sudo tcpdump -ni lo 'udp dst port 6201 or udp dst port 6202'
```

### Controller remains in `WAIT_FOR_TASK_POSE`

Check that the sender and receiver both use port `6501` and that version-2
packet sizes are accepted:

```bash
sudo tcpdump -ni lo 'udp dst port 6501'
```

### Controller never becomes stale when the tracker freezes

The sender is probably incrementing `sequence_id` while republishing the same
underlying estimate, or it is using the legacy matrix-only packet. Increment
the sequence only for a genuinely new tracker estimate.

## Before committing

```bash
cd ~/repos/panda_tracker/panda_pbvs_project

python -m unittest discover -s tests -p 'test_*.py' -v
python -m compileall common control perception tests run_control.py
python -m py_compile ../simulation/simulated_explorer_tool.py
git diff --check
git status --short
```

Do not commit generated CSV files, summary JSON files, `__pycache__`, or
temporary tuning configs unless they are intentionally part of a documented
benchmark.

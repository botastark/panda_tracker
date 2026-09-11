# Real-robot C++ PBVS migration

This migration replaces the Python `run_control.py` plus UDP `explorer`
command chain with a single C++ process that owns perception freshness,
PBVS state, safety interlocks, and the Franka control interface.

The migration is intentionally staged. A stage is not enabled until the
previous stage has been exercised on the real system and its log reviewed.

## Design reference

The implementation uses the control-flow ideas demonstrated by ViSP's
[`servoFrankaPBVS.cpp`](https://github.com/lagadic/visp/blob/master/example/servo-franka/servoFrankaPBVS.cpp):

- an explicit stopped state before control is enabled;
- zero velocity unless the operator has armed the task;
- zero velocity when target detection is lost;
- an explicit robot stop on normal exit and on exceptions;
- calibrated end-effector-to-camera geometry;
- separate translation and angle-axis PBVS errors.

No ViSP source is copied, and ViSP is not a build dependency. Stage 2 retains
an independent PBVS implementation while following the referenced control-flow
ideas. This avoids introducing a GPL dependency while the repository's license
remains unspecified.

The attached `fe_panda` project supplies the libfranka integration pattern:
standalone CMake discovery of `Franka::Franka`, direct `franka::Robot` state
access, column-major `O_T_EE`, and translation at indices 12--14. Stage 1 uses
that pattern through `Robot::read()`. It deliberately does not reuse
`explorer`'s torque-control callback or its raised collision thresholds.

## Stage 1: direct read-only observation



```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPANDA_TRACKER_BUILD_ROBOT=ON \
  -DCMAKE_PREFIX_PATH=/opt/libfranka
```

or:

```bash
find /opt/libfranka -name FrankaConfig.cmake -print

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPANDA_TRACKER_BUILD_ROBOT=ON \
  -DFranka_DIR=/directory/that/contains/FrankaConfig.cmake
```

With `PANDA_TRACKER_BUILD_ROBOT=ON`, configuration now fails immediately if
libfranka cannot be found. This prevents a successful-looking build that
silently omitted `pbvs_observer`.



### Real-robot test 1A: state and tracker observation

Assume the Panda workstation is `192.168.10.20` and the sender is
`192.168.10.30`. Replace both with the actual LAN addresses. On the Panda
workstation, stop `explorer` and any real tracker publisher, then start the
read-only observer:

```bash
./build/pbvs_observer \
  --robot-ip 172.16.0.2 \
  --tracker-bind-ip 0.0.0.0 \
  --tracker-source-ip 192.168.10.30 \
  --tracker-port 6501 \
  --tracker-timeout 0.1 \
  --duration 25 \
  --csv stage1_fake_lan.csv
```

## Stage 2: compute-only PBVS

Status: implemented.

Stage 2 extends the read-only Stage 1 observer with the PBVS calculations that
would precede a robot command. It deliberately stops at that boundary: the
program computes a candidate pose but cannot apply it to the Panda.

### What changed from Stage 1

| Area | Stage 1 | Stage 2 |
| --- | --- | --- |
| Panda access | Reads `franka::RobotState` | Still read-only |
| Tracker handling | Validates and logs PTP2 packets | Also applies readiness, freshness, jump, confidence, and enable-error gates |
| PBVS | Not calculated | Calculates goal pose, position/orientation errors, bounded velocities, and a proposed pose |
| Configuration | Observer CLI values | Loads the existing PBVS JSON with `--pbvs-config`; the JSON controls the loop rate and tracker timeout |
| Robot output | None | None; proposed values exist only in memory, console output, and CSV |

The same `pbvs_observer` executable enables this mode when passed
`--pbvs-config`. It loads the existing JSON configuration directly in C++ and
applies the existing Python controller's goal transform,

```text
T_goal = T_BE T_ES inverse(T_TS) T_TS_des inverse(T_ES)
```

and logs:

- base-frame translation error and proposed linear velocity;
- end-effector body-frame angle-axis error and proposed angular velocity;
- readiness, tracking, hold, stale, invalid, jump, and enable-threshold state;
- target-velocity estimate, speed limits, command-lead limit, and the proposed
  pose that would have been sent.

The proposed values are diagnostics only. The executable uses
`franka::Robot::read()` and still contains no `robot.control()` call and no
position, velocity, or torque command type. `control_rate_hz` and
`tracker_timeout` come from the PBVS JSON. Conflicting explicit CLI values are
rejected so that the tested configuration remains the single authority.

The C++ tests cover Python parity, frame sign, exact 180-degree angle-axis,
Franka column-major conversion, unique-sequence readiness, stale and jump
holds, target feedforward, speed clamping, and the real robot JSON config.

Acceptance criteria:

- The banner says `READ-ONLY PBVS COMPUTE OBSERVER` and `COMPUTE ONLY`.
- The physical robot never moves.
- The CSV state advances through `READY` to `TRACKING` after 20 unique packets
  (the lower-rate console output may not display every intermediate state).
- Orientation error remains close to zero and translation error stays at or
  below approximately 5 mm.
- Proposed linear speed never exceeds 0.04 m/s, proposed angular speed never
  exceeds 2 deg/s, and proposed command lead never exceeds 5 mm.
- When the sender stops, state changes to `HOLD` with
  `reason=task_pose_stale`.
- Final output reports that compute-only PBVS entered `TRACKING` and no command
  was sent.
- CSV ages are non-negative and rejected, old, duplicate, and wrong-source
  counters remain zero.

The current config reports
`nominal_mechanical_geometry_do_not_enable_robot_until_calibrated`. That is
acceptable for this compute-only comparison, but it blocks Stage 3 motion.

Before any Stage 3 implementation, complete the
[event-camera-in-hand and stick calibration procedure](event_camera_tool_calibration.md).
It starts by auditing the active libfranka `EE` against the flange and MuJoCo
Hand frames, then calibrates camera intrinsics, flange-to-camera hand-eye
geometry, the physical stick tip, and camera/stick self-view closure. The
flange-referenced results remain authoritative; robot-`EE` and simulation-hand
transforms are derived separately when their configured frames differ.

## Planned stages

### Stage 3: guarded translation-only control

Add an explicit operator arm/deadman, latched faults, source timestamps,
workspace/tool-volume checks, joint and singularity margins, acceleration and
jerk limiting, and zero velocity on every invalid/stale state. Orientation
remains disabled.

### Stage 4: full six-degree-of-freedom PBVS

Enable angle-axis orientation control only after translation-only hardware
logs pass. Validate hand-eye/tool calibration, payload, collision behavior,
rate limits, and convergence thresholds on the real system.

### Stage 5: retire the legacy runtime

Remove the Python controller and the `explorer` command bridge only after the
new C++ controller passes repeatable startup, tracking, target-loss, network
loss, operator-stop, and Franka-reflex tests.

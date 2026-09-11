# Panda position tracking

This folder contains position-tracking experiment.

## Files

```text
cpp/src/pbvs_robot_position.cpp
        |
        +-- tracker_receiver
        |      +-- task_pose_protocol
        |
        +-- tracker_position_filter
        |
        +-- position_servo
        |      +-- geometry
        |
        +-- robot_safety
        |
        +-- position_tracking_config
```

## controller

The tracker sends `T_CT`.
Only target translation is trusted:

```text
T_CT translation
 -> 5-sample median
 -> EMA alpha 0.25
 -> post-filter 50 mm sanity jump
 -> fixed desired R_CT
```

The robot side then computes:

```text
T_TS = inverse(T_CT_filtered) * T_CS
```

and the translational PBVS error is mapped from stick S to physical flange F,
then into Panda base B.

An axis mask selects the translation axes:

```yaml
control_x: true
control_y: false
control_z: false
```

The output is a Cartesian velocity command:

```text
[vx, vy, vz, 0, 0, 0]
```
Orientation is intentionally not controlled in this version.

## Build

From this folder:

```bash
cmake -S . -B build -DFranka_DIR=/opt/libfranka/build
cmake --build build -j
```

## Preflight

```bash
./build/pbvs_robot_position \
  --robot-ip 172.16.0.2 \
  --config cpp/configs/position_tracking_test.yml \
  --apply-load-model \
  --preflight-only
```

## First X-only motion test

```bash
./build/pbvs_robot_position \
  --robot-ip 172.16.0.2 \
  --tracker-bind-ip 0.0.0.0 \
  --tracker-source-ip 172.16.223.232 \
  --tracker-port 5000 \
  --config cpp/configs/position_tracking_test.yml \
  --apply-load-model \
  --enable-motion
```

## Load model

The config contains the last complete mass + COM + inertia set available in the
provided files:

```text
mass = 0.8876 kg
```

## Cartesian velocity rate limiting

The executable uses its own conservative call to `franka::limitRate()` with the
configured 0.5 mm/s, 2 mm/s^2 and 20 mm/s^3 translational limits.

The enclosing `robot.control()` therefore has libfranka's second built-in rate
limiter disabled and uses `franka::kMaxCutoffFrequency` to disable the command
low-pass filter. This avoids running a manually rate-limited signal through a
second rate limiter/filter.

The 1-kHz callback also publishes non-blocking diagnostics. The 1-Hz worker log
shows:

- `safety_scale`
- `rt_req_mmps`
- `limited_mmps`
- `O_dP_EE_c_mmps`
- `O_ddP_EE_c_mmps2`
- `cmd_success`

These distinguish the PBVS request from the velocity actually being handed to
libfranka.
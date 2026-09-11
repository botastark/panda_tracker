# Panda position tracking

Franka Panda position-based visual servoing (PBVS) experiment (for now only translation).

The current robot motion generator uses joint velocity control. A desired
Cartesian translation velocity is mapped through the Panda flange translational
Jacobian and then explicitly limited in joint velocity, joint acceleration, and
joint jerk before being sent to libfranka.


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
        +-- joint_velocity_mapper
        |
        +-- robot_safety
        |
        +-- position_tracking_config
```

## controller

The tracker sends `T_CT`.

```text
tracker T_CT
 -> Filter : N-sample median + EMA alpha 0.25 +  post 50 mm sanity jump
 -> PBVS translation error in Panda base frame
 -> Cartesian speed / acceleration / jerk limiting
 -> translation-only damped least-squares Jacobian 
 -> joint velocity / acceleration / jerk limiting
 franka::JointVelocities
```

Software safety includes:
```text
startup-relative flange travel envelope
joint speed / torque / torque-rate limits
external joint torque monitoring
external wrench monitoring
tracker freshness / tracking-loss grace
robot-state and command freshness
communication success-rate checks
```


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

## XY-only motion test with telemetry

```bash
./build/pbvs_robot_position \
  --robot-ip 172.16.0.2 \
  --tracker-bind-ip 0.0.0.0 \
  --tracker-source-ip 172.16.223.232 \
  --tracker-port 5000 \
  --config cpp/configs/position_tracking_xy.yml \
  --apply-load-model \
  --recover \
  --enable-motion \
  --telemetry-csv /tmp/pbvs_telemetry.csv \
  --telemetry-rate 50
```
second terminal 
```bash
python3 plot_pbvs_telemetry.py /tmp/pbvs_telemetry.csv
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
# Direct `T_TS` Controller Interface

Understood. Then the controller interface should be defined as:

$$
\boxed{\text{Controller measurement input}=T_{TS}}
$$

not `T_TC`.

Here:

$$
T_{TS}=\text{pose of stick-tip frame }S\text{ expressed in triangle frame }T
$$

The vision algorithm and the controller are therefore operating directly in the **task space**.

## Corrected Physical and Control Frames

```text
Physical robot chain

Panda base B
    │
    │ measured T_BE
    ▼
Robot-controlled EE frame E
    │
    │ fixed calibrated T_ES
    ▼
Stick-tip frame S



Moving triangle frame T
    │
    │ measured T_TS
    ▼
Stick-tip frame S
```

The camera frame `C` still exists physically:

```text
E ──T_EC──> C
E ──T_ES──> S
```

but it does not need to appear at the controller interface if the vision algorithm already produces `T_TS`.

## Controller Inputs

### Measured Robot State

$$
T_{BE}
$$

Pose of the robot-controlled EE frame in Panda base coordinates.

This comes from the physical Panda state stream.

### Measured Task State

$$
T_{TS}
$$

Pose of the stick tip relative to the moving triangle.

This comes directly from the vision algorithm.

### Desired Task State

$$
T_{TS}^{des}
$$

The required pose of the tip relative to the triangle.

### Fixed Robot-to-Tool Geometry

$$
T_{ES}
$$

Pose of the stick-tip frame relative to the robot-controlled EE frame.

It can be represented directly, or calculated from:

$$
T_{ES}=T_{EC}T_{CS}
$$

The current configuration already contains `T_EC`, `T_CS`, and `T_TS_des`, although the code currently builds its control interface around a tracker measurement called `T_TC`.

---

## Direct Task Error

The controller can compare the measured and desired tip poses directly.

One useful relative correction transform is:

$$
\boxed{
T_{\Delta S}
=
T_{TS}^{-1}T_{TS}^{des}
}
$$

This transform describes the correction from the current stick-tip pose to the desired stick-tip pose.

Its translational part gives the task position correction, and its rotational part gives the task orientation correction.

Alternatively, depending on whether errors are expressed in the triangle frame or tip frame, the controller may use:

$$
T_{TS}^{des}T_{TS}^{-1}
$$

The multiplication order must remain consistent with the controller’s existing spatial/body error convention.

---

## Direct Derivation of the Robot Command

The current stick-tip pose in the Panda base is:

$$
T_{BS}=T_{BE}T_{ES}
$$

From the measured vision pose:

$$
$$

we can infer the triangle pose in the Panda base:

$$
T_{BT}
=
T_{BS}T_{TS}^{-1}
$$

Then the desired stick-tip pose in the Panda base is:

$$
T_{BS}^{des}
=
T_{BT}T_{TS}^{des}
$$

The corresponding desired robot EE pose is:

$$
T_{BE}^{des}
=
T_{BS}^{des}T_{ES}^{-1}
$$

Combining these equations gives:

$$
\boxed{
T_{BE}^{des}
=
T_{BE}
T_{ES}
T_{TS}^{-1}
T_{TS}^{des}
T_{ES}^{-1}
}
$$

This is the central relationship for the direct-`T_TS` controller.

It uses:

- current robot pose `T_BE`
- fixed EE-to-tip transform `T_ES`
- measured task pose `T_TS`
- desired task pose `T_TS_des`

It does not require `T_BT`, `T_TC`, or the camera pose in the Panda base.

---

## Corrected Information Flow

```text
Physical Panda
    │
    │ measured T_BE
    │ EE E expressed in base B
    ▼
PBVS task controller
    ▲
    │ measured T_TS
    │ stick tip S expressed in triangle T
    │
Vision algorithm
    ▲
    │ camera images
    │
Camera mounted on tool


PBVS task controller also loads:

    T_ES       fixed EE-to-stick-tip transform
    T_TS_des   desired tip pose relative to triangle

Controller output:

    T_BE_cmd   desired robot EE pose in Panda base

PBVS controller
    │
    │ T_BE_cmd
    ▼
explorer_tracker.cpp
    │
    ▼
Physical Panda
```

---

## Revised Network Schema

```text
explorer_tracker.cpp
    └── UDP 6200: measured T_BE, <6f>
              │
              ▼
        udp_pose_fanout.py
              │
              ├── 6201 → run_control.py
              └── 6202 → MuJoCo mirror


Vision algorithm
    └── new tracker/task port:
        measured T_TS
              │
              ▼
        run_control.py


run_control.py
    └── UDP 2600:
        desired T_BE_cmd, <6f>
              │
              ▼
        explorer_tracker.cpp
```

The old physical-control path:

```text
T_BE + T_BT → triangle_pose_to_tracker.py → T_TC
```

should be removed from the final vision-based controller path.

`triangle_pose_to_tracker.py` can remain only for:

- testing with a synthetic absolute triangle pose
- legacy compatibility
- visualization debugging

The current bridge specifically computes `T_TC` from `T_BE`, `T_BT`, and `T_EC`, so it is not needed when `T_TS` arrives directly.

---

## What Remains Useful from the Camera Calibration

Although the controller no longer consumes `T_TC`, the vision algorithm may still internally require:

$$
T_{CS}
$$

because the camera observes the triangle and must infer the stick-tip pose.

For example, if vision internally estimates the triangle relative to the camera, it can calculate:

$$
T_{TS}=T_{TC}T_{CS}
$$

But this conversion belongs inside the vision system. The controller sees only the final `T_TS`.

Ownership should therefore be:

| Transform | Owner |
|------------|--------|
| `T_BE` | Panda state / robot backend |
| `T_ES` | Robot/tool calibration |
| `T_EC` | Camera hand-eye calibration |
| `T_CS` | Camera-to-tip calibration |
| `T_TS` | Vision algorithm output |
| `T_TS_des` | Task configuration |
| `T_BE_cmd` | Controller output |

---

## What We Have

The intended system now has:

- measured Panda pose `T_BE`
- full measured task pose `T_TS`
- desired task pose `T_TS_des`
- rigid tool geometry through `T_ES`
- Cartesian EE command output `T_BE_cmd`
- stale robot-state and stale tracker safety concepts
- compliant hold in the physical Panda controller when command packets stop

---

## What Still Must Be Defined or Verified

### Exact `T_TS` Packet

The packet should be documented explicitly as:

```text
x_T_S
y_T_S
z_T_S
roll_T_S
pitch_T_S
yaw_T_S
```

with:

$$
R_{TS}=R_z(yaw)R_y(pitch)R_x(roll)
$$

and:

- position in metres
- angles in radians
- little-endian encoding
- exact packet size

### Exact Tip Frame `S`

The origin and axes must be physically defined:

- origin at the actual contact point or tip center
- one axis along the stick
- known sign of the outward stick direction

### Exact Triangle Frame `T`

It must be established that:

- the origin is the target triangle center
- the normal direction is consistent
- in-plane axes do not randomly flip
- symmetry ambiguities are resolved

### Valid `T_ES`

The robot still needs an accurate transform from the controlled EE frame to the physical stick-tip frame.

A small angular error at the box or stick mount can produce a large position error at the end of a long stick.

### Valid `T_TS_des`

For contact, this may not be identity:

$$
T_{TS}^{des}\neq I
$$

The translation may include:

- triangle thickness
- tip radius
- stand-off distance
- desired contact compression


### Controller State Behaviour

The direct controller should have at least:

```text
WAIT_FOR_ROBOT
WAIT_FOR_VISION
TRACK
APPROACH
CONTACT/HOLD
STALE_DATA_HOLD
```

When `T_TS` becomes stale, it should stop updating the target and transition to a safe hold rather than continuing with the last relative pose.

---

## Final Core Definition

The controller contract should now be:

```text
Inputs:
    T_BE       current robot EE pose in Panda base
    T_TS       current stick-tip pose in triangle frame
    T_ES       fixed robot-EE-to-stick-tip transform
    T_TS_des   desired stick-tip pose in triangle frame

Output:
    T_BE_cmd   desired robot EE pose in Panda base
```

with:

$$
\boxed{
T_{BE}^{cmd}
=
T_{BE}
T_{ES}
T_{TS}^{-1}
T_{TS}^{des}
T_{ES}^{-1}
}
$$

That is the reference-frame equation around which the updated controller and information-flow diagram should be built.

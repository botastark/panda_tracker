# Panda PBVS simulation

These scripts test the PBVS pipeline before connecting the real Panda.

The official MuJoCo Python package supports interactive viewing, and MuJoCo
Menagerie provides a curated Franka Panda model. Install:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install mujoco numpy
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```

## Files

### `inspect_model.py`

Prints all names and addresses in an MJCF model.

```bash
python simulation/inspect_model.py \
  mujoco_menagerie/franka_emika_panda/panda.xml
```

Confirm the EE body name. The simulation defaults to `hand`; override it with
`--ee-body` if your model uses another name.

### `tracker_placeholder.py`

No MuJoCo required. Sends a manually adjustable tracker matrix:

```text
<16d, row-major T_TC
```

Run:

```bash
python simulation/tracker_placeholder.py --port 6500
```

Use it with the PBVS controller in dry-run mode to verify packet format and
error signs.

### `pure_sim_tracker.py`

Shows only a triangle and camera frame. It sends the displayed `T_TC` to port
6500. This is useful before involving Panda kinematics:

```bash
python simulation/pure_sim_tracker.py --tracker-port 6500
```

### `simulated_explorer.py`

Mirrors the real `explorer_safe` network interface:

- receives EE commands on port 2600 as `<6f`;
- sends simulated EE state on port 6200 as `<6f`;
- sends synthetic `T_TC` on port 6500 as `<16d`.

It uses damped least-squares Cartesian inverse kinematics and monitors the
smallest Jacobian singular value.

## Simulation PBVS config

Copy the real PBVS config:

```bash
cp event_pbvs/pbvs_config.json event_pbvs/pbvs_config_sim.json
```

Edit:

```json
"panda_ip": "127.0.0.1",
"panda_state_bind_ip": "127.0.0.1",
"dry_run": false
```

For the first test, keep simple placeholder transforms, for example `T_EC=I`.
Use the same config file for both `event_pbvs_tracker.py` and
`simulated_explorer.py`.

## Closed-loop test

Terminal 1:

```bash
source .venv/bin/activate
python simulation/simulated_explorer.py \
  --panda-xml mujoco_menagerie/franka_emika_panda/panda.xml \
  --pbvs-config event_pbvs/pbvs_config_sim.json \
  --ee-body hand
```

Terminal 2:

```bash
source .venv/bin/activate
python event_pbvs/event_pbvs_tracker.py \
  --config event_pbvs/pbvs_config_sim.json \
  --tracker-bind-ip 127.0.0.1 \
  --tracker-udp-port 6500
```

The simulator publishes both Panda state and synthetic tracker feedback, so
this is a true closed loop.

## Keyboard controls

Inside the MuJoCo viewer:

```text
W/S  triangle +/- x_B
A/D  triangle +/- y_B
R/F  triangle +/- z_B

I/K  triangle roll +/-
J/L  triangle pitch +/-
U/O  triangle yaw +/-

0    reset triangle
```

Test one axis at a time.

## Expected tests

1. **Zero-error:** the Panda remains still.
2. **Triangle lateral translation:** the camera follows laterally.
3. **Triangle normal translation:** the stick/camera distance returns to the
   configured target.
4. **Single-axis rotation:** camera orientation follows the triangle.
5. **Tracker/PBVS stop:** simulated explorer holds the latest EE pose after its
   command timeout.
6. **Singularity check:** `sigma_min` should not collapse toward zero during
   normal operation.

## Real tracker format test

Use the actual event tracker only in PBVS dry-run mode first. Adapt its output
to:

```python
T_TC.shape == (4, 4)
packet = struct.pack("<16d", *T_TC.reshape(-1))
```

The final row must be `[0, 0, 0, 1]`, translation must be in metres, and
rotation must represent the camera frame in the triangle frame.

## Notes

- This is a kinematic commissioning simulator, not a torque-accurate replica.
- It validates transforms, signs, UDP, rate limiting, IK feasibility, joint
  limits, and singularity trends.
- The generated `_pbvs_generated_panda.xml` is written beside `panda.xml` so
  relative mesh paths remain valid.
  
# Simulation backend

Use the previously created MuJoCo simulator as the process that:

- receives absolute EE commands on UDP port 2600;
- publishes simulated EE state on UDP port 6200;
- publishes synthetic tracker pose `T_TC` on UDP port 6500.

The high-level controller in `run_control.py` connects through
`MujocoUdpBackend`, so it uses the same protocol as the real Panda.

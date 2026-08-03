source /home/bota/panda_tracker/.venv/bin/activate

sudo ss -lunp | grep -E ':(2600|6200|6500)\b'
kill -CONT PID
kill PID

## Running PBVS Controller on Mujico simulator

### MuJoCo simulator

Start MuJoCo simulator so it publishes simulated `T_BE` and `T_TS`.

```bash
python simulation/simulated_explorer_tool.py \
  --panda-xml mujoco_menagerie/franka_emika_panda/panda.xml \
  --pbvs-config panda_pbvs_project/configs/pbvs_sim.json \
  optional:
  --headless \
  --headless-duration 180

```

### PBVS controller

```bash
python panda_pbvs_project/run_control.py \
  --backend sim \
  --config panda_pbvs_project/configs/pbvs_sim.json
```

### Test predefined Triangle motion

```bash
python tests/predefined_triangle_trajectory.py \
  --destination 127.0.0.1:6601 \
  --rate 30 \
  --repeat 1 \
  --initial-hold 15 \
  --center-pose \
    0.0 \
    0.554499507 \
    0.294352422 \
    0.0 \
    0.0\
    90.0
```

## Running the Physical Panda, Digital Twin, and PBVS Controller

This setup runs the physical Panda, the MuJoCo digital twin, and the PBVS controller on the same computer.

### UDP port layout

```text
Physical explorer
    |
    v
0.0.0.0:6200  udp_pose_fanout.py
    |---> 127.0.0.1:6201  run_control.py robot state
    `---> 127.0.0.1:6202  MuJoCo digital-twin state

MuJoCo digital twin
    `---> 127.0.0.1:6501  run_control.py tracker pose

run_control.py
    `---> Panda command port 2600
```

Only the UDP fan-out process should bind to port `6200`.

### 1. Configure the physical Panda backend

In `configs/pbvs_robot.json`, use:

```json
"panda_state_bind_ip": "127.0.0.1",
"panda_state_port": 6201
```

Keep the Panda command settings unchanged.

### 2. Start the UDP state fan-out

From `panda_pbvs_project`:

```bash
python3 udp_pose_fanout.py \
  --bind-ip 0.0.0.0 \
  --bind-port 6200 \
  --destination 127.0.0.1:6201 \
  --destination 127.0.0.1:6202
```

Expected output:

```text
Listening for Panda state on 0.0.0.0:6200
Forwarding to: 127.0.0.1:6201, 127.0.0.1:6202
Expected packet: 24 bytes (<6f)
```

Once the physical explorer starts, the relay should report a nonzero forwarding rate.

### 3. Start the physical Panda explorer

```bash
cd /opt/libfranka/fe_panda

../build/fe_panda/explorer 1 0
```

Leave this process running.

### 4. Start the MuJoCo digital twin

If running MuJoCo as a mirror/visualizer:

```bash
python simulation/simulated_explorer_tool.py \
  --panda-xml  mujoco_menagerie/franka_emika_panda/panda.xml \
  --pbvs-config panda_pbvs_project/configs/pbvs_robot.json \
  --real-state-bind-ip 127.0.0.1 \
  --real-state-port 6202 \
  --disable-task-pose-output
```

### 5. Start the PBVS controller

```bash
python panda_pbvs_project/run_control.py  \
  --backend panda \
  --config panda_pbvs_project/configs/pbvs_robot.json \
  --tracker-bind-ip 127.0.0.1 \
  --tracker-port 6501
```

The tracker port must match the simulator's `--tracker-port`.

### 6. Check port ownership

Before starting, or when debugging, run:

```bash
sudo ss -lunp | grep -E ':(2600|6200|6201|6202|6501)\b'
```

Expected listeners:

```text
6200  udp_pose_fanout.py
6201  run_control.py
6202  simulated_explorer_holder_camera_udp_triangle.py
6501  run_control.py
```

### Troubleshooting

If the simulator reports:

```text
sync_error: |e_p|=inf m, |e_R|=inf deg
```

then no valid physical-state packet has reached port `6202`.

Check the physical stream:

```bash
sudo tcpdump -ni any 'udp dst port 6200'
```

Check the forwarded streams:

```bash
sudo tcpdump -ni lo 'udp dst port 6201 or udp dst port 6202'
```

If `run_control.py` raises:

```text
OSError: [Errno 98] Address already in use
```

another process is already bound to the configured robot-state or tracker port. Stop the old process or correct the port assignments above.

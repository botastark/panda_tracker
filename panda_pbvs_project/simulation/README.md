# Simulation backend

Use the previously created MuJoCo simulator as the process that:

- receives absolute EE commands on UDP port 2600;
- publishes simulated EE state on UDP port 6200;
- publishes synthetic tracker pose `T_TC` on UDP port 6500.

The high-level controller in `run_control.py` connects through
`MujocoUdpBackend`, so it uses the same protocol as the real Panda.

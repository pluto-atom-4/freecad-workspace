# ctrl-wheel

Sim-only bridge and Webots wheel copy that relays the `WHL` wheel-velocity line from the Wokwi program `c3_ctrl_wheel` (pluto-atom-4/balancing-robot-controller) to a velocity-driven Webots wheel and logs the measured return.

## Status

- Planned, not built.
- Simulation-only.
- Not hardware-validated.
- Nothing in this directory has been run.

## Links

- Plan: pluto-atom-4/balancing-robot-controller discussions/234 (https://github.com/pluto-atom-4/balancing-robot-controller/discussions/234)
- Counterpart program: `programs/c3_ctrl_wheel` in pluto-atom-4/balancing-robot-controller
- Tracking issue: pluto-atom-4/freecad-workspace#444

## Planned files

- `ctrl_wheel_msg.py`
- `ctrl_wheel_udp.py`
- `ctrl_wheel_relay.py`
- `fake_wheel.py`
- `ctrl_wheel_log.py`
- `ctrl_wheel_bridge.py`
- `webots/controllers/ctrl_wheel/`
- `webots/worlds/ctrl_wheel.wbt`

Later tasks add these files. A later task (FRD) extends this README with the origin of each copy.

# Servo-Wheel DOF Webots POC (issue #258)

Procedural STS3032 servo-driven wheel DOF proof-of-concept in Webots; no PROTO/mesh assets, native-sensor-only, no IMU-frame emulation.

## Purpose

Establish a minimal, testable framework for:
- **Procedural STS3032 servo model**: Servo geometry and joint built entirely in code within Webots, no external PROTO files or mesh assets imported.
- **Native Webots sensor readout**: PositionSensor and rotational motor feedback only; no synthetic IMU-frame emulation layer.
- **Servo-driven wheel control**: Demonstrates end-to-end control of a wheel degree-of-freedom actuated by a procedural servo model, with sensor feedback loop closed in simulation.

Status: #258 is the umbrella issue. #259 (mamba environment recipe) is complete. #260 (this README) is in progress. Verification and usage workflows are deferred to #267.

## Scope

- **Procedural servo geometry and joints** built in Python/Webots code only; no imported PROTO workbench objects or mesh files.
- **Native Webots sensors only**: PositionSensor for joint angle readout, motor torque control; explicitly NOT an IMU-frame emulation layer or synthetic sensor fusion.
- **POC scope**: Procedural model and sensor integration only. No verification checklists, usage documentation, or manual testing procedures yet; those belong in #267.

## Env setup

Mamba environment `servo-wheel-dof` with Python 3.11, NumPy, and pytest for simulation scripting and testing. Webots is installed separately outside this environment.

### One-time installation

```bash
# Option 1: Use the pinned lock file (recommended)
mamba env create -n servo-wheel-dof -f poc/servo-wheel-dof/mamba-envs.lock.yml

# Option 2: Use the setup_command from mamba-envs.yaml
mamba create -n servo-wheel-dof -c conda-forge python=3.11 "numpy>=1.24" "pytest>=9.1" -y
```

⚠️ **Note**: This recipe uses a custom YAML schema (to match `inverted-pendulum-project`'s structure) — `mamba env create -f mamba-envs.yaml` will **fail**. Use the lock file or the manual `setup_command` above instead.

Webots' `controller` Python module ships inside the Webots install tree (`$WEBOTS_HOME/lib/controller/python`) and is added to PYTHONPATH at controller runtime — it is NOT installed into this mamba environment.

### Verification

```bash
mamba run -n servo-wheel-dof python -c "import numpy, pytest; print('OK')"
```

Should print `OK`.

## Layout

```
poc/servo-wheel-dof/
  README.md                                This file
  mamba-envs.yaml                          Custom schema env recipe (use lock or setup_command instead)
  mamba-envs.lock.yml                      Pinned/reproducible env export (use with `mamba env create -n servo-wheel-dof -f`)
  run_demo.sh                              One-command demo script (not yet implemented)
  webots/                                  Webots integration (world files, controllers)
    worlds/                                Webots world files (`.wbt`)
      servo_wheel_dof.wbt                  World: procedural STS3032 servo + wheel (not yet implemented)
    controllers/                           Webots robot controller scripts
      servo_wheel_dof/                     Controller directory
        servo_wheel_dof.py                 Controller entry point (not yet implemented)
        servo_sim.py                       Servo simulation logic (not yet implemented)
        sensor_read.py                     Native sensor readout (not yet implemented)
        test_servo_wheel_dof.py            Unit tests (not yet implemented)
```

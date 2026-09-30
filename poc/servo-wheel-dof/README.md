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
  run_demo.sh                              Headless/GUI demo launcher (--headless/--gui/DRY_RUN=1)
  webots/                                  Webots integration (world files, controllers)
    worlds/                                Webots world files (`.wbt`)
      servo_wheel_dof.wbt                  World: procedural STS3032 servo + wheel, controller wired
    controllers/                           Webots robot controller scripts
      servo_wheel_dof/                     Controller directory
        servo_wheel_dof.py                 Controller entry point (only file importing `controller`)
        servo_sim.py                       Servo motion-profile math (pure, no Webots dependency)
        sensor_read.py                     Sensor readback + validation (pure, no Webots dependency)
        test_servo_wheel_dof.py            Unit tests (22 passing)
```

## Reproducing end to end

Requires: the `servo-wheel-dof` mamba env; Webots (with `WEBOTS_HOME`/`webots` on PATH, or set `WEBOTS_BIN`) with a real X display for the GUI step.

All commands below are from the repo root unless stated.

### 1. Create the env

```bash
mamba env create -n servo-wheel-dof -f poc/servo-wheel-dof/mamba-envs.lock.yml
mamba run -n servo-wheel-dof python -c "import numpy, pytest; print('OK')"
```

`mamba env create -f mamba-envs.yaml` does NOT work (custom schema); see "Env setup".

### 2. Run the tests

```bash
cd poc/servo-wheel-dof
mamba run -n servo-wheel-dof python3 -m pytest -q webots/controllers/servo_wheel_dof
```

Expect `22 passed`. Pure-Python unit tests only (`servo_sim.py`, `sensor_read.py`, controller helpers) — no Webots, no display, no hardware.

### 3. Run the demo — GUI mode (needs a display)

```bash
cd poc/servo-wheel-dof
./run_demo.sh --gui
```

Opens Webots with `worlds/servo_wheel_dof.wbt` running, `wheel_motor`/`wheel_sensor` bound by the `servo_wheel_dof` controller. Expect the wheel to rotate smoothly and `servo_wheel_dof: ...` lines (see "Sensor output format" below) streaming in the console.

### 4. Run the demo — headless mode (no display; CI/agent-friendly)

```bash
cd poc/servo-wheel-dof
./run_demo.sh --headless
```

Runs the same world/controller in Webots batch mode with no rendering window. Proves the controller starts, binds both devices, and streams sensor lines — it does NOT prove the GUI looks right (see "What is verified" / "NOT verified").

`DRY_RUN=1 ./run_demo.sh --headless` (or `--gui`) prints what would run without launching Webots.

## Sensor output format

Each simulation step the controller logs one line to stdout:

```
servo_wheel_dof: t=0.0160 angle_rad=0.000000 velocity_rad_s=nan
servo_wheel_dof: t=0.0320 angle_rad=0.010525 velocity_rad_s=0.657835
```

Fields:
- **`t`** — simulation time in seconds (`robot.getTime()`), fixed 4 decimal places.
- **`angle_rad`** — `wheel_sensor` (PositionSensor) reading in radians, fixed 6 decimal places.
- **`velocity_rad_s`** — angle derivative between consecutive steps, in rad/s, fixed 6 decimal places; `nan` on the very first step only (no previous sample to difference against).

Only `servo_wheel_dof.py` imports the Webots `controller` module; the motion-profile math (`servo_sim.py`) and sensor-sample validation (`sensor_read.py`) are pure Python and unit-tested outside Webots.

## What is verified

Verified headlessly (no GUI, by automation), with this method:

- **Unit tests**: `cd poc/servo-wheel-dof && mamba run -n servo-wheel-dof python3 -m pytest -q webots/controllers/servo_wheel_dof` passes, 22 tests (motion profile, sensor read/validation, controller helper logic).
- **Controller log format**: `servo_wheel_dof.py`'s `_log()` produces `servo_wheel_dof: t=<4dp> angle_rad=<6dp> velocity_rad_s=<6dp|nan>` lines; confirmed against source and against a headless batch run's captured output (first line `nan` velocity, subsequent lines numeric).
- **Device binding**: the controller looks up `wheel_motor` (RotationalMotor) and `wheel_sensor` (PositionSensor) by exact name and exits with an `ERROR device '<name>' not found` stderr line + nonzero exit code if either is missing, rather than silently no-op'ing.
- **`run_demo.sh` argument parsing / dry-run path**: `bash -n run_demo.sh` and the `DRY_RUN=1` path (no Webots launch).

## NOT verified (needs a human with a display)

This POC has **not** been run through actual human-eyeball GUI verification as of writing this section — only headless batch runs and unit tests, by agents. Specifically NOT yet confirmed by a human:

- **World loads and the wheel is visibly present** in the Webots GUI (`servo_wheel_dof.wbt` geometry, camera framing, floor/wheel look correct).
- **The wheel actually rotates smoothly** when driven by the controller's velocity commands (no jitter, no snapping, matches the motion profile visually) — headless runs only confirm the numeric log stream, not the rendered motion.
- **Sensor stream sustains >= 10 Hz** in a live GUI run over a sustained period (headless log lines have been read from source/short batch captures, not timed by a human over a long run).
- **Headless batch mode runs clean end-to-end** for a full `run_demo.sh --headless` invocation (clean start, clean exit, no stray Webots processes) — this needs a human to run it, not just review the script.

Do not tick any box in the checklist below until a human has actually done it.

## Manual verification checklist (human)

Tick a box only after doing it yourself. Nothing here was done by the automation that wrote this section. Record results in the "Verification log" below and as a comment on issue #267.

- [ ] **1. World loads and wheel is visible** (needs display, no hardware)
  Do: `cd poc/servo-wheel-dof && ./run_demo.sh --gui`.
  Pass: Webots window opens, `servo_wheel_dof.wbt` loads with no console errors, the wheel geometry is visible and correctly placed (not floating/clipping), camera framing shows the whole model.
  If it fails: window doesn't open -> check `$DISPLAY`/Webots install; wheel missing/mispositioned -> fix `worlds/servo_wheel_dof.wbt`, not the controller.
- [ ] **2. Controller binds sensor and wheel rotates smoothly** (needs display)
  Do: with the world running (Play pressed if not auto-run), watch the wheel and the console.
  Pass: console shows `servo_wheel_dof: t=... angle_rad=... velocity_rad_s=...` lines starting immediately, no `ERROR device ... not found`; wheel rotation is smooth (no jumps/stutters) and visually tracks the velocity profile from `servo_sim.motion_profile()`.
  If it fails: device-not-found -> device name mismatch between `servo_wheel_dof.py` and the `.wbt` (`wheel_motor`/`wheel_sensor` must match exactly); jerky motion -> check timestep vs. motion-profile assumptions in `servo_sim.py`, open an issue.
- [ ] **3. Sensor stream sustained >= 10 Hz** (needs display or headless timing)
  Do: capture console output over at least 5 real seconds (e.g. `./run_demo.sh --gui 2>&1 | ts '%.s'` or pipe through `awk` timing the `t=` deltas), count lines per second.
  Pass: sustained rate >= 10 Hz (i.e. `t=` deltas average <= 0.1 s across the sample window); no long stalls.
  If it fails: note the observed rate and whether Webots' `basicTimeStep` in the `.wbt` needs lowering, or whether logging/flush is the bottleneck (`_log()` already uses `flush=True`).
- [ ] **4. Headless batch mode runs clean** (no display needed)
  Do: `cd poc/servo-wheel-dof && ./run_demo.sh --headless`, let it run to completion or a fixed timeout, then check exit code and `pgrep -a webots` afterward.
  Pass: exits 0 (or the script's documented timeout exit), `servo_wheel_dof: ...` lines stream throughout, no Python traceback, no leftover `webots` process after exit.
  If it fails: leftover process -> fix `run_demo.sh` cleanup/trap logic; traceback -> fix the controller, not the script.

### Verification log (fill in by hand)

| Date | Who | Item(s) | Result (pass/fail) | Notes / issue link |
|------|-----|---------|--------------------|--------------------|
|      |     |         |                    |                    |

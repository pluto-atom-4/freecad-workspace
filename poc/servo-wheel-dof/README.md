# Servo-Wheel DOF Webots POC (issue #258)

Procedural STS3032 servo + wheel in Webots. The wheel stands upright, rim on the
floor, and a supervisor controller follows UDP IMU + wheel-angle datagrams
(mock or replayed data today). No PROTO or mesh assets.

## Purpose

- **Upright follower mode**: the controller sets the Robot pose from IMU pitch and
  yaw and drives the wheel hinge to the received `wheel_angle`.
- **Hardware-free**: `monitor/mock_publisher.py` supplies synthetic or replayed data.
- **Long-term goal**: data from an XIAO ESP32-C3 + MPU6050. The MPU6050 is 6-DOF
  (accel + gyro), with no magnetometer. Today ONLY mock/replay data is supported;
  nothing in this POC reads serial.

Status: #258 is the umbrella issue. The human GUI checklist is tracked in #289.

## Env setup

Mamba environment `servo-wheel-dof` (Python 3.11, NumPy, pytest). Webots is
installed separately; its `controller` module exists only inside Webots.

```bash
# Option 1: pinned lock file (recommended)
mamba env create -n servo-wheel-dof -f poc/servo-wheel-dof/mamba-envs.lock.yml
# Option 2: manual
mamba create -n servo-wheel-dof -c conda-forge python=3.11 "numpy>=1.24" "pytest>=9.1" -y
mamba run -n servo-wheel-dof python -c "import numpy, pytest; print('OK')"
```

`mamba env create -f mamba-envs.yaml` does NOT work (custom schema); use the lock
file or the manual command. The last command should print `OK`.

## Layout

```
poc/servo-wheel-dof/
  README.md  CLAUDE.md                 This file, and rules for working here
  mamba-envs.yaml                      Custom-schema recipe (do not use with -f)
  mamba-envs.lock.yml                  Pinned env export
  run_demo.sh                          Launcher: --headless | --gui, optional --mock
  monitor/
    mock_publisher.py                  UDP publisher: --mock or --replay CSV
    test_swd_publisher.py              Publisher tests
  webots/
    worlds/servo_wheel_dof.wbt         World: upright wheel on a static supervisor Robot
    controllers/servo_wheel_dof/
      servo_wheel_dof.py               Controller (the only file importing `controller`)
      imu_wheel_msg.py                 Message parse/format/mock (copied from esp32-dof)
      imu_udp_latest.py                Drain UDP, keep latest (copied from esp32-dof)
      imu_euler_math.py                ZYX Euler to axis-angle (copied from esp32-dof)
      upright_pose.py                  Robot translation + rotation from pitch, yaw
      servo_sim.py, sensor_read.py     Older pure modules, still tested
      test_swd_*.py, test_servo_wheel_dof.py   Pure-Python tests
```

The controller imports `sensor_read.read_sensor` (for the log lines) but no longer
calls `servo_sim.py`; the old motion profile is not played.

## Pose and sign convention

- Frame: ENU (x East, y North, z Up). The wheel stands upright, rim on the floor.
- The Robot is rotated +90 deg about X, so the Robot-local hinge axis Z maps to
  world -Y. The wheel centre is at world (0, -0.026, 0.03).
- Positive motor angle / `wheel_angle` is a right-hand rotation about world -Y:
  the wheel top moves toward -X, counter-clockwise seen from the -Y camera.
- The red marker box on the rim (wheel-local +X side) rises for positive
  `wheel_angle`. Checked headlessly and by the maintainer in the live GUI (see
  the PR #296 description); the full manual checklist is issue #289.
- IMU pitch and yaw drive the Robot: rotation is Rz(yaw) * Ry(pitch) * Rx(+90 deg).
  The wheel centre stays fixed and the rim stays on the floor.
- IMU roll is ignored by design.
- The same convention is stated in the header comment of
  `webots/worlds/servo_wheel_dof.wbt`; keep the two in sync.

## UDP protocol

- Transport: UDP, one JSON datagram per message, to `127.0.0.1:5006`.
- Port: default 5006, overridden by env `SWD_UDP_PORT` (integer 1..65535).
  The esp32-dof follower uses 5005, so both can coexist.
- Schema (angles in radians; roll/pitch/yaw are ZYX Euler angles):

```json
{"seq": 0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "wheel_angle": 0.0}
```

- All four angle keys are required numbers. `seq` and extra keys are ignored.
- Rejected as invalid: missing key, string/null/bool/list values, NaN, Infinity.
  The controller counts invalid datagrams and keeps the last valid pose.
- The controller binds the port without SO_REUSEADDR. A second instance exits 1.

Mock sample (`imu_wheel_msg.mock_imu_wheel`), t in seconds, all radians:

| Field | Formula |
|-------|---------|
| roll | 0.5 sin(2 pi 0.2 t) |
| pitch | 0.3 sin(2 pi 0.1 t) |
| yaw | 0.8 sin(2 pi 0.05 t) |
| wheel_angle | 1.5 sin(2 pi 0.25 t) |

## Workflow

All commands run from `poc/servo-wheel-dof` and need the `servo-wheel-dof` env.

### One command: Webots plus mock publisher

```bash
./run_demo.sh --gui --mock        # window; publishes until Ctrl-C (exit 130 is normal)
./run_demo.sh --headless --mock   # no window; stops by itself near TIMEOUT_S
```

With `--mock`, `run_demo.sh` runs Webots in the background in its own process
group, waits until the controller binds the UDP port, then runs
`monitor/mock_publisher.py --mock`. On exit, Ctrl-C or SIGTERM it stops only that
Webots process group (never `pkill`). On a normal end the exit code is the
publisher's (0, or 2 for bad arguments); Ctrl-C exits 130 and SIGTERM exits 143
(the script's traps). In GUI mode the simulation must be RUNNING; press Play
if it starts paused.

Environment variables:

| Variable | Meaning |
|----------|---------|
| `WEBOTS_BIN` | Webots binary (default `/usr/local/bin/webots`) |
| `TIMEOUT_S` | Headless wall-clock timeout, seconds (default 30) |
| `DRY_RUN=1` | Print the resolved plan and exit 0; nothing is launched |
| `SWD_UDP_PORT` | UDP port for controller and publisher (default 5006) |
| `SWD_PYTHON` | Python for the publisher (default: python of the mamba env) |
| `SWD_WAIT_S` | Max seconds to wait for the controller to bind (default 30) |
| `MOCK_DURATION` | Publisher seconds (headless default: TIMEOUT_S minus startup, 3 s) |

Fail-fast (exit 1): world file missing, `WEBOTS_BIN` not found, `$DISPLAY` unset
with `--gui`, invalid `SWD_UDP_PORT`/`SWD_WAIT_S`/`MOCK_DURATION`, non-positive
`TIMEOUT_S` (headless `--mock`), no `setsid`, no python for the env, UDP port
already in use, Webots exits early, or the port is not bound within `SWD_WAIT_S`.
`DRY_RUN=1` still requires `WEBOTS_BIN` to exist and validates `--mock` inputs.

```bash
DRY_RUN=1 ./run_demo.sh --headless --mock    # shows the full mock plan, launches nothing
```

Without `--mock`, `./run_demo.sh --gui` or `--headless` only launches Webots; the
wheel holds the pose from the `.wbt` until the first valid datagram arrives.

### Publisher alone (Webots running in another terminal)

```bash
mamba run -n servo-wheel-dof python3 monitor/mock_publisher.py --mock --duration 5
mamba run -n servo-wheel-dof python3 monitor/mock_publisher.py --replay run.csv
```

- Exactly one of `--mock` or `--replay CSV`. Options: `--port` (default
  `SWD_UDP_PORT` or 5006), `--rate` Hz (default 50), `--duration` seconds.
- `--mock` runs until Ctrl-C or `--duration`. `--replay` stops at the end of file.
- Exit codes: 0 on normal end or Ctrl-C; 2 for bad arguments or an invalid CSV.
- It prints `published`, `errors` and `skipped` counts at the end.
- Stdlib only: no numpy, no `controller`, no pyserial.

Replay CSV columns: `roll,pitch,yaw,wheel_angle` (radians, required) and optional
`t_us` (microseconds). Column order is free, extra columns are ignored, a UTF-8
BOM is accepted. Without `t_us` rows are paced by `--rate`; with `t_us` the gaps
are followed (each clamped to 1 s). Example:

```csv
roll,pitch,yaw,wheel_angle,t_us
0.0,0.00,0.00,0.0,0
0.0,0.10,0.20,0.5,20000
0.0,0.20,0.40,1.0,40000
```

## Controller behavior

- Binds `127.0.0.1:$SWD_UDP_PORT` (non-blocking); exits 1 if the bind fails.
- Each Webots step (`basicTimeStep` 16 ms) it drains all queued datagrams and
  keeps the newest valid one.
- On a message: sets Robot translation and rotation from `upright_pose(pitch, yaw)`
  and calls `wheel_motor.setPosition(wheel_angle)`.
- Before the first valid message the `.wbt` pose is untouched.
- Stdout/stderr lines, prefixed `servo_wheel_dof: `:
  - `listening on 127.0.0.1:5006`
  - `first message received`
  - `ignored N invalid datagram(s) so far` (first 3, then every 100th)
  - every step: `t=0.0160 angle_rad=0.000000 velocity_rad_s=nan` (`nan` on the
    first step only; `angle_rad` is the `wheel_sensor` reading in radians)
- Device names are exact: `wheel_motor` (RotationalMotor), `wheel_sensor`
  (PositionSensor). A missing device prints `ERROR device '<name>' not found`.

## Run the tests

```bash
cd poc/servo-wheel-dof
mamba run -n servo-wheel-dof python3 -m pytest -q monitor webots/controllers/servo_wheel_dof
```

Pure Python: no Webots, no display, no hardware. `test_swd_world_upright.py` is a
text-level guard on the `.wbt`; set env `SWD_WORLD_PATH` to test a modified copy.

## Limits

- The Robot is static (no Physics for the Robot, no odometry): the wheel spins in
  place and does not roll or move across the floor.
- MPU6050 has no magnetometer, so real yaw would drift. Mock/replay yaw does not.
- `imu_wheel_msg.py`, `imu_udp_latest.py` and `imu_euler_math.py` are copied, not
  imported, from `poc/esp32-dof`. Fixes must be mirrored by hand (each file header
  names its source).
- The simulation must be running (not paused) for the controller to step.
- Only mock/replay data is supported; serial/hardware input is not implemented.

## Human verification

The marker direction for positive `wheel_angle` was checked by the maintainer in
the live GUI (recorded in the PR #296 description). Not yet verified by a human:
pose response to IMU pitch and yaw, and wheel jitter. The manual checklist and its
result log live in issue #289. Only a human ticks those boxes; nothing here is
ticked.

## Superseded: velocity-demo checklist

The earlier checklist (issue #267) covered a self-driven velocity profile
(`servo_sim.motion_profile()`) that the controller no longer plays. It is
superseded by the UDP follower described above and by issue #289. Its items and
log table were removed from this file; see git history for the old text.

<!-- end of servo-wheel-dof README -->

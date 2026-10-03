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
    telemetry_summary.py               Offline summary of a telemetry CSV (stdlib only)
    test_swd_telemetry_summary.py      Summary tests
  webots/
    worlds/servo_wheel_dof.wbt         World: upright wheel on a static supervisor Robot
    controllers/servo_wheel_dof/
      servo_wheel_dof.py               Controller (the only file importing `controller`)
      imu_wheel_msg.py                 Message parse/format/mock (copied from esp32-dof)
      imu_udp_latest.py                Drain UDP, keep latest (copied from esp32-dof)
      imu_euler_math.py                ZYX Euler to axis-angle (copied from esp32-dof)
      upright_pose.py                  Robot translation + rotation from pitch, yaw
      telemetry.py                     Opt-in CSV telemetry: COLUMNS, env parsing, fail-soft writer
      alignment_metrics.py             Anchor deviation, axis dot, wheel position in Robot frame
      overlay_text.py                  Opt-in GUI overlay text, colour and update rate
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
- Telemetry and overlay are opt-in, read-only and off by default; with them off the
  output is unchanged. See "Telemetry" below.
- Stdout/stderr lines, prefixed `servo_wheel_dof: `:
  - `listening on 127.0.0.1:5006`
  - `first message received`
  - `ignored N invalid datagram(s) so far` (first 3, then every 100th)
  - every step: `t=0.0160 angle_rad=0.000000 velocity_rad_s=nan` (`nan` on the
    first step only; `angle_rad` is the `wheel_sensor` reading in radians)
- Device names are exact: `wheel_motor` (RotationalMotor), `wheel_sensor`
  (PositionSensor). A missing device prints `ERROR device '<name>' not found`.

## Telemetry

Opt-in, read-only and fail-soft: the controller only reads poses and writes text. It
never sets a pose, never calls `resetPhysics`, and any telemetry or overlay error prints
one stderr line, turns that feature off and lets the simulation keep running. Unset
`SWD_TELEMETRY` leaves all output unchanged. Code lives in the pure modules
`telemetry.py`, `alignment_metrics.py` and `overlay_text.py`.

### Turn it on

| Variable | Meaning |
|----------|---------|
| `SWD_TELEMETRY` | On: `1`, `true`, `yes`, `on`. Off: empty, `0`, `false`, `no`, `off` (case-insensitive). Anything else: off plus one stderr warning. Default off. |
| `SWD_TELEMETRY_FILE` | CSV path; the file is opened with "w" (overwritten). Alone it does NOT enable telemetry. A relative path resolves against the controller process working directory (the controller directory under Webots); use an absolute path. Unusable path: one stderr `ERROR`, telemetry off. Unset or empty: stdout mode. |
| `SWD_OVERLAY` | Same on/off values. Live alignment label in the 3D view (see Overlay). Default off. |

```bash
# file mode (advised, especially in the GUI)
SWD_TELEMETRY=1 SWD_TELEMETRY_FILE=$PWD/run1.csv ./run_demo.sh --headless --mock
# stdout mode: every line is prefixed "telemetry: " (doubles the log volume)
SWD_TELEMETRY=1 ./run_demo.sh --headless --mock
```

`run_demo.sh` needs no change: the variables pass through to Webots. The controller logs
`telemetry -> <absolute path>` (or `telemetry -> stdout`) at start. There is no rotation:
the file grows for as long as the run lasts. One row is written per Webots step.

### Columns (equal to `telemetry.COLUMNS`, 17, in this order)

`t_s,msgs,age_s,wheel_cmd,wheel_sensor,wheel_err,vel_rad_s,robot_x,robot_y,robot_z,wheel_z,rel_x,rel_y,rel_z,anchor_dev_m,axis_dot,warn`

| Column | Unit | Meaning and healthy value |
|--------|------|---------------------------|
| `t_s` | s | Simulation time. |
| `msgs` | count | Valid datagrams so far (integer). There is no `seq` column. |
| `age_s` | s | Sim time since the last valid datagram; `nan` before the first one. |
| `wheel_cmd` | rad | Last commanded `wheel_angle`; `nan` before the first message. |
| `wheel_sensor` | rad | `wheel_sensor` reading. |
| `wheel_err` | rad | `wheel_cmd - wheel_sensor`; `nan` before the first message. |
| `vel_rad_s` | rad/s | Sensor velocity; `nan` on the first step. |
| `robot_x`, `robot_y`, `robot_z` | m | Robot world position. |
| `wheel_z` | m | Wheel centre world height. Healthy: about 0.03 (the wheel radius, rim on the floor). |
| `rel_x`, `rel_y` | m | Wheel centre in the Robot frame: radial drift. Healthy: about 0. |
| `rel_z` | m | Wheel centre in the Robot frame: axial drift. Healthy: 0.026 (the hinge anchor offset). |
| `anchor_dev_m` | m | Distance from the wheel centre to the hinge anchor. Healthy: about 0 (a healthy run showed a max near 8e-5 m). |
| `axis_dot` | none | Dot product of the wheel and Robot local Z axes, clamped to [-1, 1]. Healthy: about 1.0. |
| `warn` | 0/1 | 1 when `anchor_dev_m` is above the limit or `axis_dot` is below the limit (below). |

Formatting: floats are `%.6f`; `msgs` is an integer; any missing or non-finite value is
the text `nan`. Without the wheel node (or if a pose read fails) the nine pose columns
(`robot_x` to `axis_dot`) are `nan` and `warn` is 0; the controller prints one
`WARN: wheel node not found, alignment metrics off` line at start.

Timing: poses are read after the step and BEFORE the new command is applied, so a row
shows the effect of the PREVIOUS step's command. `age_s` and `t_s` are simulation time.

### WARN thresholds

`warn` = 1 when `anchor_dev_m > 0.001` (1 mm) or `axis_dot < 0.9999`. Non-finite pose
input also gives `warn` = 1 with `nan` metrics. A warn row flushes the file at once
(otherwise flush every 62 rows). stderr gets `servo_wheel_dof: WARN t=... dev_m=... dot=...`
for the first 3 warn steps, then every 100th warn step; the CSV still has every row.

### Overlay (`SWD_OVERLAY=1`, GUI only)

Needs `SWD_TELEMETRY=1` and the wheel node; otherwise one stderr `WARN` and overlay off.
It draws a 4-line label via `Supervisor.setLabel` about 4 times per second, green when OK
and red on warn, and clears it at exit:

```
servo-wheel OK|WARN
dev=<mm> mm
axis=<dot>
rel=(<x>, <y>, <z>) mm
```

Headless runs do not show it (checking it needs a human with a display; nothing here is
ticked). Values are in mm, rel as x, y, z in the Robot frame.

### Read a CSV: `monitor/telemetry_summary.py`

```bash
python3 monitor/telemetry_summary.py run1.csv              # window default 0.5 s
python3 monitor/telemetry_summary.py run1.csv --window 1.0
```

Stdlib only; also accepts a stdout capture (only lines starting `telemetry:` are used,
prefix stripped). Rows with a wrong cell count are skipped and counted; `nan` cells are
ignored by every statistic. Output is `key : value` lines (illustrative values, `<..>` varies):

```
file              : run1.csv
rows              : <n>
skipped           : 0
t_start_s         : <s>
t_end_s           : <s>
duration_s        : <s>
msgs_last         : <n>
wheel_err_max_abs : <rad>
wheel_err_mean_abs: <rad>
anchor_dev_m_max  : <healthy: ~0.00008>
axis_dot_min      : <healthy: ~1.0>
warn_rows         : 0
first_warn_t_s    : n/a
```

When a warn row exists, `window_s`, `window_rows` and then those rows (CSV with header)
within +-window of `first_warn_t_s` follow. A statistic with no finite value prints `n/a`.
Exit codes: 0 summary printed (also for a header-only file); 2 for bad arguments, an
unreadable file, an empty file or a missing required column (`t_s`, `msgs`, `wheel_err`,
`anchor_dev_m`, `axis_dot`, `warn`).

### Diagnosing the #302 wheel detach

1. Capture a healthy headless baseline, then the same run in the GUI (issue #310 holds the
   exact capture steps; a human with a display does it):
   `SWD_TELEMETRY=1 SWD_TELEMETRY_FILE=$PWD/run1.csv ./run_demo.sh --headless --mock`.
2. Run `telemetry_summary.py` on each CSV. Healthy: `warn_rows` 0, `anchor_dev_m_max` far
   below 0.001, `axis_dot_min` about 1.0.
3. If `warn_rows` > 0, read `first_warn_t_s` and the window rows. Radial drift shows in
   `rel_x`/`rel_y`, axial drift in `rel_z` (nominal 0.026), tilt in `axis_dot`; compare
   `age_s`/`msgs` and `wheel_err` at that time to see whether it followed a command.
4. Attach the CSV and summary to #302 (GUI) or #303 (healthy baseline), per #310.

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

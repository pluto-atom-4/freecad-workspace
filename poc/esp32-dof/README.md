# ESP32 6-DOF IMU POC (issue #227)

Proof-of-concept integration of an external **XIAO ESP32-S3 Sense** microcontroller with a 6-DOF inertial measurement unit (IMU): firmware streams accelerometer + gyroscope frames over USB serial, a host-side Python monitor parses and fuses them into roll/pitch/yaw, and a Webots world shows a box that follows the orientation live. See "Reproducing end to end" to run it and "Manual verification checklist (human)" for what still needs a person with a display and the board.

## Purpose

Establish a minimal, testable framework for:
- **Firmware communication**: Serial protocol over USB/UART to stream IMU data (6-DOF: 3-axis accelerometer + 3-axis gyroscope) from the ESP32 to the host PC.
- **Host-side monitor**: Python script that decodes incoming IMU packets, validates data integrity, and logs telemetry.
- **Webots integration**: A Webots supervisor controller rotates a box (`ESP32_BODY`) to follow live, mock, or replayed IMU orientation received over localhost UDP, to visually confirm orientation tracking.

Status: firmware (#233), monitor (#234-#238), Webots world and controller (#231, #232, #236), and run scripts (#239) are all implemented. Hardware and GUI behavior are NOT yet human-verified; see "What is verified" below.

## Hardware

- **Microcontroller**: XIAO ESP32-S3 Sense (Seeed Studio)
  - ⚠️ **IMPORTANT**: The Sense variant does *not* include an onboard IMU. Do NOT assume one is present.
- **IMU Sensor**: MPU-6050 (InvenSense, e.g. GY-521 module), interfaced externally via I2C
  - 3-axis accelerometer (firmware configures +/-4 g)
  - 3-axis gyroscope (firmware configures +/-500 dps)
  - Connected via I2C to D4 (GPIO5, SDA) + D5 (GPIO6, SCL) on the XIAO ESP32-S3 Sense; address 0x68 (0x69 if AD0 high)
  - The MPU-6050 gyro has a zero-rate offset that varies with temperature, so yaw drift can be noticeable (yaw drifts anyway: no magnetometer). The firmware averages the gyro for about 1 s at boot to remove the bias; keep the board still and flat for that first second after power-up or reset.
  - Mounting: lay the module flat, component side up, with its printed X arrow toward the box's nose (+X). No axis remap is applied; +1 g must read on Z when flat.
  - MPU-6050 support (#252) is **compile-checked only**. It has NOT been flashed or tested on hardware and needs human verification (`0x68` found, frames stream, tilt drives the box).

## Env setup

This project provides a mamba environment (`esp32-dof`) with Python 3.11, NumPy, PySerial, and pytest.

### One-time installation

```bash
# From the repo root, use the pinned lock file:
mamba env create -n esp32-dof -f poc/esp32-dof/mamba-envs.lock.yml

# OR use the setup_command from mamba-envs.yaml:
mamba create -n esp32-dof -c conda-forge python=3.11 "numpy>=1.24" "pyserial>=3.5" "pytest>=9.1" -y
```

⚠️ **Note**: This recipe uses a custom YAML schema (to match `inverted-pendulum-project`'s structure) — `mamba env create -f mamba-envs.yaml` will **fail**. Use the lock file or the manual `setup_command` above instead.

### Verification

```bash
mamba run -n esp32-dof python -c "import serial, numpy, pytest; print('OK')"
```

Should print `OK`.

### Serial port access (Linux)

On Linux, serial access to `/dev/ttyACM*` requires the `dialout` group:

```bash
# Check current groups:
groups

# If 'dialout' is not listed, add yourself:
sudo usermod -aG dialout $USER

# Log out and back in for the group change to take effect.
```

Do NOT run this command as part of automated setup — it requires an interactive login afterward and may not succeed in a headless environment.

If `serial_lines` raises `DofSerialError` ("cannot open serial port ..."), the device is missing (see `dof_monitor.py --list-ports`), held by another program (e.g. ModemManager, Arduino serial monitor), or you are not in the dialout group (see above). More in "Troubleshooting".

## Layout

```
poc/esp32-dof/
  mamba-envs.yaml         Custom schema env recipe (use lock or setup_command instead)
  mamba-envs.lock.yml     Pinned/reproducible env export (use this with `mamba env create -n esp32-dof -f`)
  README.md               This file
  CLAUDE.md               Per-POC guidance for Claude Code
  run_mock_demo.sh        One-command demo: Webots GUI + mock monitor publishing over UDP (#239)
  firmware/               Arduino sketch and documentation (build artifacts gitignored)
    README.md             Firmware setup, compilation, and testing guide
    esp32_dof/            Firmware sketch directory (folder name matches .ino basename)
      esp32_dof.ino       XIAO ESP32-S3 + MPU-6050 frame streamer
  monitor/                Host-side Python monitor script + tests
    dof_frame.py          Frame protocol parser and formatter
    test_esp32dof_frame.py Frame protocol unit tests
    dof_fusion.py         Complementary filter, radians out
    test_esp32dof_fusion.py Complementary filter unit tests
    dof_sources.py        Mock and CSV-replay frame line sources (Iterator[str])
    test_esp32dof_sources.py Source unit tests
    dof_serial.py         pyserial line source (serial_lines, list_ports; serial:// and loop:// URLs)
    test_esp32dof_serial.py Serial source unit tests (fake port + loop://, no hardware)
    dof_stats.py          StreamStats: received/dropped/out_of_order/resets/bad_frames, windowed rate (pure)
    test_esp32dof_stats.py StreamStats unit tests
    dof_monitor.py        Monitor CLI: --port | --mock | --replay, table + summary, optional --fuse / --publish (#237, #238)
    test_esp32dof_monitor.py Monitor CLI/run() unit tests (sources monkeypatched, no hardware)
    dof_publisher.py      OrientationPublisher: localhost UDP JSON sender to the Webots follower (#238)
    test_esp32dof_publisher.py Publisher + fuse/publish integration + UDP contract tests (no hardware)
  webots/                 Webots integration (world files, controllers)
    run_gui.sh            Launch the world in the Webots GUI, realtime (#239)
    worlds/               Webots world files (`.wbt`); temporary `.wbproj` is gitignored
      esp32_dof.wbt       World: floor + supervisor box ESP32_BODY (#231)
    controllers/          Webots robot controller scripts
      esp32_dof_follower/ Webots supervisor controller (UDP -> ESP32_BODY rotation)
        esp32_dof_follower.py Controller entry point (only file importing `controller`)
        dof_udp_latest.py Non-blocking UDP drain: latest valid orientation + counts
        test_esp32dof_udp_latest.py UDP drain unit tests (real localhost sockets)
        dof_webots_math.py Orientation parsing + Euler-to-axis-angle helpers
        test_esp32dof_webots_math.py Controller-side unit tests
```

## Frame protocol

Binary telemetry from the ESP32 is encoded as ASCII frames, streamed over serial at 50 Hz (20 ms per frame), one frame per line:

```
IMU,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>*<HH>\n
```

### Format specification

- **Prefix**: `IMU` (literal ASCII)
- **seq**: Sequence number (uint16, 0–65535); wraps at 65536
- **t_us**: Device microseconds (uint32, 0–4,294,967,295); capped at 2^32−1
- **ax, ay, az**: Accelerometer in g (3 × float); formatted as `%.6f` (6 decimal places, fixed-point, no exponent)
- **gx, gy, gz**: Gyroscope in deg/s (3 × float); formatted as `%.6f`
- **HH**: 2-digit UPPERCASE hexadecimal XOR checksum of every character from `IMU` to `*` (exclusive)
- **Terminator**: `\n` (ASCII 10); receiver also accepts `\r\n` (CRLF)

### Float formatting

**Sender (firmware / format_frame):** MUST emit `%.6f` (exactly 6 decimal places, fixed-point, **no exponent notation**).
  - Valid examples: `0.500000`, `-9.806650`, `1.250000`
  - Special case: "-0.000000" is normalized to "0.000000" before checksum

**Receiver (parse_line):** Tolerates any finite decimal float token matching Python's `float()` parser, including exponent notation and leading `+` sign. Rejects:
  - Embedded whitespace (e.g., `1 .0`)
  - Underscores (e.g., `1_000`)
  - Non-finite values (`nan`, `inf`)
  
  Examples of tolerated input: `1e-3`, `+2.5`, `2E+2` (parser lenient for compatibility; format_frame still emits `%.6f` only).

### Checksum computation

The checksum is the bitwise XOR of the ASCII ord value of every character in the frame body (from `I` in `IMU` to the character before `*`). Expressed in pseudocode:

```c
// C/C++ (firmware)
uint8_t checksum = 0;
for (const char* p = body; p < end; p++) {
    checksum ^= (uint8_t)*p;
}
printf("%02X", checksum);  // 2-digit UPPERCASE hex
```

### Example

Golden frame (firmware parity vector):

```
IMU,12345,1234567890,0.500000,-9.806650,1.250000,10.500000,-5.500000,0.000000*52
```

Body (checksum input): `IMU,12345,1234567890,0.500000,-9.806650,1.250000,10.500000,-5.500000,0.000000`  
Checksum: `0x52` (82 decimal)

*Note: Example values are illustrative. Accelerometer and gyroscope values are in units of g and deg/s respectively; the specific values shown are not necessarily representative of a typical physical device configuration.*

## Fusion

6-DOF complementary filter for orientation estimation (roll, pitch, yaw) from combined accelerometer and gyroscope data.

**Inputs:**
- Accelerometer: 3 values in units of *g* (Earth's gravity ≈ 9.81 m/s²)
- Gyroscope: 3 values in units of *degrees per second* (dps)
- Time step: in *seconds*

**Output:**
- Roll, pitch, yaw: all in *radians* (ZYX convention: R = Rz(yaw) Ry(pitch) Rx(roll))

**Limitations:**
- Yaw drifts unbounded over time without an external reference (no magnetometer).
- Roll and pitch are ill-conditioned near pitch ≈ ±90° (gimbal lock region).
- No free-fall or linear acceleration rejection; assumes +1 g on Z-axis when device is stationary and flat.
- Euler-angle rate coupling ignored; simple gyro integration without accounting for Euler-rate transformation (POC trade-off; production systems should use quaternions).

**Usage:**
```python
from dof_fusion import ComplementaryFilter

fuse = ComplementaryFilter(alpha=0.98)  # 98% gyro, 2% accel correction
accel_g = (ax, ay, az)                 # accelerometer in g
gyro_dps = (gx, gy, gz)                # gyroscope in deg/s
dt_s = 0.02                            # time step in seconds

roll, pitch, yaw = fuse.update(accel_g, gyro_dps, dt_s)
```

## Sources

A **source** is an `Iterator[str]` of raw frame lines (non-empty, stripped of `\r\n` trailing whitespace; consumers pass each to `dof_frame.parse_line()`). The serial source (`dof_serial.serial_lines`) follows the same contract.

**Built-in sources:**

- **`mock_lines(rate_hz=50.0, duration_s=None, realtime=True, drop_every=0) → Iterator[str]`**
  - Deterministic synthetic motion: roll = 0.5·sin(2π·0.2·t), pitch = 0.3·sin(2π·0.1·t) radians.
  - Accelerometer: derived from gravity + roll/pitch rotations (no linear acceleration).
  - Gyroscope: gx = d(roll)/dt, gy = d(pitch)/dt (degrees/s), gz = 0 (matching the fusion model's simple axis-rate integration).
  - `duration_s=None` yields infinite frames; use `itertools.islice()` to limit iteration.
  - `drop_every=N` skips every Nth frame slot (N > 0): creates detectable seq gaps while time marches on; `drop_every=1` yields nothing (acceptable).
  - `realtime=False` yields as fast as possible (useful for testing); `realtime=True` sleeps to match frame spacing (wall-clock replay).
  - Argument errors (invalid rate_hz, duration_s, drop_every) are **eager** — raised at call time, before any frame is generated.

- **`csv_replay_lines(path, realtime=True) → Iterator[str]`**
  - Replays frame lines from a CSV file. Header row required with columns (any order, extras ignored): `seq`, `t_us`, `ax`, `ay`, `az`, `gx`, `gy`, `gz`.
  - UTF-8 BOM (if present) is automatically stripped.
  - Blank rows skipped by the CSV parser; row errors include file line number for debugging.
  - `realtime=True`: sleeps by inter-frame t_us deltas (clamped to 1.0 s max); no sleep on first row or negative/zero deltas.
  - `realtime=False`: yields as fast as possible.
  - Argument errors (invalid path type, missing columns) are **eager**; row format/value errors are **lazy** (raised during iteration).

- **`dof_serial.serial_lines(port, baud=115200, timeout_s=1.0, reconnect=True, *, max_duration_s=None) → Iterator[str]`**
  - Reads a real device (/dev/ttyACM0) or any pyserial URL (loop://) via serial.serial_for_url
  - Yields decoded (UTF-8, errors="replace") non-empty lines with \r\n stripped
  - Does NOT parse frames (pass each line to dof_frame.parse_line; garbled lines raise FrameError there)
  - port opens on the first next() (lazy); argument errors (ValueError) eager
  - Failed first open raises DofSerialError (a serial.SerialException) immediately even with reconnect=True; message names the dialout group
  - Read timeouts don't end the iterator
  - A partial line is held and completed when the rest arrives
  - The first line after each (re)open is discarded (may be a fragment)
  - A run of >4096 bytes with no newline is dropped
  - If the device disappears after a successful open: reconnect=True logs to stderr, sleeps 1 s and reopens (retrying indefinitely); reconnect=False re-raises the SerialException
  - max_duration_s (optional) ends the iterator cleanly once elapsed (checked at least every timeout_s) so a silent device cannot block a duration limit
  - Ctrl-C propagates and closes the port

- **`dof_serial.list_ports() → list[str]`**
  - Sorted device names from serial.tools.list_ports.comports()

**Usage example:**

```python
from itertools import islice
from dof_frame import parse_line
from dof_sources import mock_lines

# Synthetic stream: 100 frames at 50 Hz, no wall-clock delay
for line in islice(mock_lines(rate_hz=50.0, realtime=False), 100):
    frame = parse_line(line)
    print(f"seq={frame.seq}, roll/pitch gyro rates: {frame.gyro[0]:.1f}, {frame.gyro[1]:.1f} deg/s")
```

**Consumer notes for drift handling:**

Consumers should be aware that both `seq` and `t_us` wrap:
- `seq` wraps at 65536 (wraparound every ~21.8 min at 50 Hz). Compute differences as `(seq2 - seq1) % 65536`.
- `t_us` wraps at 2³²−1 microseconds (~71.6 min at 50 Hz). Compute deltas as `(t_us2 - t_us1) & 0xFFFFFFFF`.

## Monitor

**Live monitoring CLI for IMU streams** — prints a real-time table of frames and a final summary with stream quality metrics.

**Usage:**

```bash
# Mock (synthetic) 50 Hz stream for 3 seconds
mamba run -n esp32-dof python3 monitor/dof_monitor.py --mock --duration 3

# With fused orientation (complementary filter, adds columns)
mamba run -n esp32-dof python3 monitor/dof_monitor.py --mock --fuse --duration 3

# With UDP publisher to Webots (drives the box; world running with Play pressed; mock wobble roll 0.5 sin(2π0.2t), pitch 0.3 sin(2π0.1t))
mamba run -n esp32-dof python3 monitor/dof_monitor.py --mock --publish --duration 10

# Real device (auto-reconnect on disconnect)
mamba run -n esp32-dof python3 monitor/dof_monitor.py --port /dev/ttyACM0

# Replay a CSV capture (non-realtime, one row per line)
mamba run -n esp32-dof python3 monitor/dof_monitor.py --replay capture.csv --quiet-every 1

# List available serial ports
mamba run -n esp32-dof python3 monitor/dof_monitor.py --list-ports
```

**Flags:**

| Flag | Description |
|------|-------------|
| `--port PATH` | Serial device or pyserial URL (e.g. `/dev/ttyACM0`, `loop://`, `socket://...`) |
| `--mock` | Synthetic 50 Hz frames at real-time speed (deterministic motion, 0 drops) |
| `--replay CSV` | Replay a CSV file with columns `seq,t_us,ax,ay,az,gx,gy,gz` |
| `--baud N` | Serial baud rate (default 115200); ignored for replay/mock |
| `--list-ports` | List available serial ports and exit |
| `--duration SECONDS` | Stop after N seconds of wall-clock time (for mock/replay); for `--port`, stops cleanly if the device is silent |
| `--quiet-every N` | Print every Nth frame to stdout (default 5); **all frames are still counted in statistics** |
| `--fuse` | Complementary-filter fusion; adds roll/pitch/yaw (deg) columns |
| `--publish` | Send fused orientation to Webots over localhost UDP (implies --fuse); harmless if Webots is not running |
| `--udp-port PORT` | UDP destination port for --publish (default 5005; 1..65535; ignored without --publish) |
| `--alpha A` | Filter gyro weight in [0,1] (default 0.98; ignored without --fuse/--publish) |

**Output:**

Rows printed to stdout (without --fuse):
```
    seq    t(ms)    ax(g)    ay(g)    az(g)  gx(dps)  gy(dps)  gz(dps)  rate  drops
      0       0.0    0.000   -0.063    0.998     0.00     0.00     0.00   0.0      0
      1      20.0   -0.000   -0.062    0.998     0.05     0.00     0.00  50.0      0
      2      40.0    0.002   -0.060    0.998     0.11     0.00     0.00  50.0      0
    ...
```

With `--fuse`, three columns are appended (roll/pitch/yaw in degrees); rate/drops columns unchanged. Out-of-order frames are not fused or published and their row repeats the previous angles. After a device reset the filter is reset (roll/pitch re-init from accel, yaw restarts at 0). Yaw drifts (no magnetometer).

Summary (always printed at end):
```
--- summary ---
received         : 150
dropped          : 0
out_of_order     : 0
resets           : 0
bad_frames       : 0
avg rate (Hz)    : 50.0 (device time, 3.00 s)
```

With `--publish`, the summary gets `published : N` (successful sends only).

**Exit codes:**
- `0` — normal completion or Ctrl-C (summary still printed)
- `2` — argument error (e.g. invalid `--duration`, missing `--port`) or source error (e.g. `cannot open serial port /dev/ttyACM0: Permission denied` → check dialout group)

Errors go to stderr; rows + summary to stdout. No Traceback on expected errors.

**Stream quality metrics:**

| Metric | Meaning |
|--------|---------|
| `received` | Total frames parsed (all sources, including garbled/out-of-order/duplicates) |
| `dropped` | Gap in sequence number (modulo 65536 wraparound): counted as `(seq2-seq1) % 65536 - 1` for forward gaps |
| `out_of_order` | Duplicates OR small backward jumps (seq decreases by ≤50); same stream continues (not a reset) |
| `resets` | Large backward jump (seq decreases by >50) OR device time went backwards; stream re-baselined from this point (no drops charged retroactively) |
| `bad_frames` | Lines that failed to parse (checksum error, malformed, etc.); ignored in rate/stats |
| `rate (Hz)` | Instantaneous frame rate from last ≤50 accepted frames; based on device t_us deltas, not wall clock |
| `avg rate (Hz)` | Average rate: `(accepted intervals) / (total device time)` since the last reset or start |

**Known limitations:**

- A dropped frame at the very **end** of a stream (after final line received) is **undetectable** — the monitor cannot distinguish "final frame N" from "final frame N+1 was dropped". This is inherent to the protocol.
- `out_of_order` increments for small reversals (b ≤ 50), but the stream does **not** rebase — the next frame must be > the "high water mark" to resume. Frames between the reversal and the high water mark are counted as out_of_order, not reset.
- Fusion dt uses the real t_us delta, so a dropped slot integrates over 40 ms; --publish sends only accepted (in-order/forward/reset) frames at the source rate (about 50 Hz); the Webots controller applies the latest per step.

## Webots IPC contract

**Simulation integration**: Webots controller receives orientation updates from the host monitor over UDP.

**Protocol:**
- Address: `127.0.0.1:5005` (localhost UDP only; no remote network transport)
- Payload: One JSON datagram per UDP packet
- Format: `{"seq": <uint>, "roll": <float>, "pitch": <float>, "yaw": <float>}`
- Units: All angles in radians (ZYX Euler convention: R = Rz(yaw) Ry(pitch) Rx(roll))
- Notes:
  - `seq` (sequence number) is optional in the JSON; receiver ignores it if present.
  - Extra keys are silently ignored by the parser.
  - JSON parser rejects NaN, ±Infinity, and non-finite values.

**Rotation convention:**
- **ZYX Euler angles** (standard aviation convention):
  - Roll: rotation about X-axis (±π)
  - Pitch: rotation about Y-axis [−π/2, π/2]
  - Yaw: rotation about Z-axis (−π, π]
  - Composed as: R = Rz(yaw) · Ry(pitch) · Rx(roll)

- **Webots representation**: Output of `euler_to_axis_angle()` is a unit axis-angle tuple (x, y, z, angle) compatible with Webots rotation fields.

**Controller (`esp32_dof_follower`, issue #236):**
- Supervisor controller of `DEF ESP32_BODY Robot` in `webots/worlds/esp32_dof.wbt`; binds a non-blocking UDP socket on `127.0.0.1:5005` (no `SO_REUSEADDR`: a second instance fails loudly with a bind error instead of silently splitting datagrams).
- Each simulation step it drains all queued datagrams, keeps the latest valid one and sets the node's `rotation` from `euler_to_axis_angle()`. Invalid datagrams are ignored (counted; first 3, then every 100th logged). Before the first message the box is left unchanged; if the sender stops, the box holds its last pose.
- **The simulation must be RUNNING (press Play, or use `--mode=realtime`/`fast`); while paused the controller is not stepped and the box will not move.**
- Console output: `esp32_dof_follower: listening on 127.0.0.1:5005`, then `first orientation message received`. Set `DOF_FOLLOWER_DEBUG=1` in Webots' environment to also print the applied rotation.
- Manual smoke test (needs a display): open `poc/esp32-dof/webots/worlds/esp32_dof.wbt`, press Play, then
  `printf '{"seq":1,"roll":0.5,"pitch":0.0,"yaw":0.0}' | nc -u -w1 127.0.0.1 5005` -- the box tilts about x.
  Or use ./webots/run_gui.sh (from poc/esp32-dof) which starts the world in realtime mode.
- Unit tests: `cd poc/esp32-dof/webots/controllers/esp32_dof_follower && mamba run -n esp32-dof python3 -m pytest -q`.

**Rotation handedness is not GUI-verified yet.** The sign conventions between the ZYX Euler model and Webots' rendering are covered by unit tests only. Expected in the live GUI: roll = π/2 tilts the box about +X with the orange nose staying on +X; yaw = π/2 turns the nose to +Y (North, ENU). Check item 2 of the "Manual verification checklist (human)". If a sign is off, fix `euler_to_axis_angle` (#232) or the `setSFRotation` call (#236); do not tweak by guesswork.

## Data flow

```
 MPU-6050 --I2C--> ESP32-S3 --USB serial 115200, 50 Hz ASCII frames--> monitor (parse + fuse)
   (external IMU)      (esp32_dof.ino)     IMU,seq,t_us,ax..gz*HH        dof_monitor.py
                                                                             |
                                                          UDP JSON {"seq","roll","pitch","yaw"} rad
                                                                             v
                                       127.0.0.1:5005  -->  Webots supervisor (esp32_dof_follower)  -->  ESP32_BODY (box rotation)
```

Mock mode replaces the first three stages with a synthetic generator (`--mock`); replay mode reads a CSV (`--replay`).

## Reproducing end to end

Requires: the `esp32-dof` mamba env; Webots R2025a (`/usr/local/bin/webots`, or set `WEBOTS_BIN`) with a real X display for the GUI steps; for real hardware also a XIAO ESP32-S3 (Sense) with an external MPU-6050 (GY-521) wired as in `firmware/README.md`, and `arduino-cli` (or Arduino IDE).

All commands below are from the repo root unless stated.

### 1. Create the env

```bash
mamba env create -n esp32-dof -f poc/esp32-dof/mamba-envs.lock.yml
mamba run -n esp32-dof python -c "import serial, numpy, pytest; print('OK')"
```

`mamba env create -f mamba-envs.yaml` does NOT work (custom schema); see "Env setup".

### 2. Run the tests

```bash
cd poc/esp32-dof
mamba run -n esp32-dof python3 -m pytest -q monitor webots/controllers/esp32_dof_follower
```

Pure-Python only: no hardware, no Webots, no display. They do not cover the firmware, the world file or the GUI.

### 3. Mock demo (no hardware; needs a display)

```bash
cd poc/esp32-dof
./run_mock_demo.sh              # runs until Ctrl-C
./run_mock_demo.sh --duration 30
```

This starts Webots (`webots/run_gui.sh`, realtime), waits until the controller listens on UDP 5005, then runs `dof_monitor.py --mock --publish` in the `esp32-dof` env and closes Webots when it exits. Expect the blue box to wobble smoothly (mock: roll 0.5*sin(2*pi*0.2*t), pitch 0.3*sin(2*pi*0.1*t) rad, 50 Hz). `--mock` runs at real time forever unless `--duration` is given; Ctrl-C prints a summary (`published : N`). `DOF_DEMO_DRY_RUN=1 ./run_mock_demo.sh` prints what it would do without starting anything. See the checklist for the pass criteria.

Manual equivalent (two terminals):

```bash
cd poc/esp32-dof
./webots/run_gui.sh                                                      # terminal 1
mamba run -n esp32-dof python3 monitor/dof_monitor.py --mock --publish   # terminal 2
```

To see the controller's applied rotations, start Webots with debug on (must be in Webots' OWN environment):

```bash
DOF_FOLLOWER_DEBUG=1 ./webots/run_gui.sh
```

Headless smoke (no window; proves the controller runs, not that it looks right; use a display or `xvfb-run` if the batch run complains about one):

```bash
cd poc/esp32-dof
DOF_FOLLOWER_DEBUG=1 timeout 20 webots --batch --mode=realtime --no-rendering --minimize --stdout --stderr webots/worlds/esp32_dof.wbt &
sleep 8; mamba run -n esp32-dof python3 monitor/dof_monitor.py --mock --publish --duration 8; wait
```

Expect `esp32_dof_follower: listening on 127.0.0.1:5005`, `first orientation message received`, and `rotation ...` lines whose angle changes.

### 4. Real hardware

Flash the sketch (details, wiring, and library versions in `firmware/README.md`; `arduino-cli` must be on your PATH):

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:CDCOnBoot=cdc poc/esp32-dof/firmware/esp32_dof
arduino-cli board list                      # find the port, usually /dev/ttyACM0
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:XIAO_ESP32S3:CDCOnBoot=cdc poc/esp32-dof/firmware/esp32_dof
```

Find the port, look at the raw stream, then drive Webots:

```bash
cd poc/esp32-dof
mamba run -n esp32-dof python3 monitor/dof_monitor.py --list-ports
mamba run -n esp32-dof python3 monitor/dof_monitor.py --port /dev/ttyACM0            # table only; expect rate ~50, drops 0
./webots/run_gui.sh                                                                    # terminal 1
mamba run -n esp32-dof python3 monitor/dof_monitor.py --port /dev/ttyACM0 --publish   # terminal 2
```

Add `--fuse` (implied by `--publish`) to see roll/pitch/yaw columns in degrees. Keep the board still and flat for the first second so roll/pitch initialize from gravity. If opening the port fails you get exit code 2 and a hint about the `dialout` group (see "Serial access (Linux)").

## What is verified

Verified headlessly (no GUI, no hardware), with this method:

- **Unit tests**: `cd poc/esp32-dof && mamba run -n esp32-dof python3 -m pytest -q monitor webots/controllers/esp32_dof_follower` passes (frame parser, fusion, sources, serial with fake port/`loop://`, stats, monitor CLI, publisher, UDP drain, Euler-to-axis-angle math).
- **Mock -> monitor -> UDP -> real Webots controller**: a reviewer ran Webots in headless batch mode (`webots --batch --mode=realtime --no-rendering --minimize --stdout --stderr <world>` under `timeout`) with `DOF_FOLLOWER_DEBUG=1` and `dof_monitor.py --mock --publish`; the controller logged `listening`, `first orientation message received`, and `rotation` lines whose angle varied over time and matched `euler_to_axis_angle(mock_angles(t))`.
- **Firmware compiles**: arduino-cli 1.5.1, esp32:esp32 3.3.12, Adafruit MPU6050 2.2.9 with BusIO 1.17.4 and Unified Sensor 1.1.15 (never flashed).
- **Parser/checksum parity**: the golden frame `IMU,12345,1234567890,0.500000,-9.806650,1.250000,10.500000,-5.500000,0.000000*52` parses and re-formats byte-for-byte in the Python parser, and the firmware source uses the same format/XOR.
- **Run scripts**: `bash -n`, the `DOF_DEMO_DRY_RUN=1` dry run, and the DISPLAY / WEBOTS_BIN / port-in-use failure paths (no GUI launched).

## NOT verified (needs a human with a display and/or the board)

- Webots world look (#231): box reads level with z up, the camera framing is sensible, the orange +x nose marker is visible.
- Rotation handedness in the GUI (#232, #236): roll = pi/2 tilts about +x with the nose staying on +x; yaw = pi/2 points the nose to +y (North, ENU).
- Simulation running vs paused: the controller is not stepped while paused (box will not move). `run_gui.sh` uses `--mode=realtime` so it should start running; not yet seen in the GUI.
- Firmware on real hardware (#233, #252): I2C comms with the MPU-6050 (WHO_AM_I `0x68`, library `begin()` accepting the chip); gyro-bias calibration; sign and axis of the mounting; the DTR/RTS reset on port open printing ROM boot text (harmless: those lines fail parsing and are counted as `bad_frames`); the golden-frame self-test (`#define DOF_SELFTEST 1`, first line must end `*52`) and the Python parity snippet in `firmware/README.md`.
- Serial reconnect (#235) with a real USB unplug; ModemManager grabbing `/dev/ttyACM*`; the `dialout` group on the test machine.
- `run_gui.sh` and `run_mock_demo.sh` in a real GUI session (the Webots process-group cleanup on exit and the bind timing when Webots starts paused are unobserved).

## Manual verification checklist (human)

Tick a box only after doing it yourself. Nothing here was done by the automation that wrote this section. Record results in the "Verification log" below and as a comment on issue #227.

- [ ] **1. Mock demo tilts the box smoothly** (needs display, no hardware)
  Do: `cd poc/esp32-dof && ./run_mock_demo.sh` (add `DOF_FOLLOWER_DEBUG=1` in front to see rotation logs).
  Pass: Webots window opens with a level blue box (top face up, z up) with a visible orange nose block at the +x end, floor grey, camera framing shows the whole box; console shows `esp32_dof_follower: listening on 127.0.0.1:5005` then `first orientation message received`; box tilts smoothly (no jumps) about roughly ±29° roll (period 5 s) and ±17° pitch (period 10 s); Ctrl-C prints the summary (`bad_frames 0`, `published` roughly 50/s) and the Webots window closes (check `pgrep -a webots` afterwards: no leftover process).
  If it fails: box static -> simulation paused (press Play) or controller not bound (check `ss -uln | grep 5005`, port in use, Webots console errors); world look wrong -> fix `webots/worlds/esp32_dof.wbt` (#231), not the controller; jerky -> note timing and open an issue.
- [ ] **2. Rotation handedness is correct** (needs display; can use mock or `nc`)
  Do: with the world running, send single poses:
  `printf '{"seq":1,"roll":1.5708,"pitch":0.0,"yaw":0.0}' | nc -u -w1 127.0.0.1 5005` then the same with roll 0 and yaw 1.5708.
  Pass: roll = pi/2 -> box rotates about its long +x axis (rolls over onto its side) and the orange nose stays at the +x end; yaw = pi/2 -> nose points along +y (the world's North/left axis), box stays level.
  If it fails: do NOT flip signs by guesswork. Open an issue with which axis/sign is wrong and fix `euler_to_axis_angle` (#232) or the `setSFRotation` call (#236) with a unit test.
- [ ] **3. Real board: tilting the board tilts the box** (needs the board flashed and wired)
  Do: flash the sketch, `dof_monitor.py --list-ports`, then `dof_monitor.py --port /dev/ttyACM0` (no publish): expect ~50 Hz, `drops 0`; then run with `--publish` alongside `./webots/run_gui.sh`; hold the board flat and still for a second, then tilt it.
  Pass: box follows the same direction and magnitude of roll and pitch; yaw drift over minutes is expected and NOT a failure.
  If it fails: IMU not found -> `ERR,imu_init` every second (wiring, address 0x68/0x69, 3V3, or the library rejecting the chip; a following `ERR,imu_whoami,0xNN` line reports what the chip answered); garbled frames -> use the golden-frame self-test and the parity snippet in `firmware/README.md`; no port -> Troubleshooting.
- [ ] **4. Unplug USB, monitor reconnects** (needs the board)
  Do: run `dof_monitor.py --port /dev/ttyACM0 --publish`, unplug the USB cable, wait ~5 s, plug it back in.
  Pass: stderr shows reconnect messages while unplugged (retries every 1 s, no traceback); after replugging, rows resume and the box follows again; the summary reports a `resets` count if the sequence restarted.
  If it fails: if the port name changes (e.g. `/dev/ttyACM1`) that is a known limitation, restart with the new port; otherwise open an issue with stderr output.

### Verification log (fill in by hand)

| Date | Who | Item(s) | Result (pass/fail) | Notes / issue link |
|------|-----|---------|--------------------|--------------------|
|      |     |         |                    |                    |

## Troubleshooting

- **`FATAL: $DISPLAY is not set`** (from `run_gui.sh` / `run_mock_demo.sh`): no X session. Export your display (`export DISPLAY=:1`) and re-run. There is no xvfb fallback for the GUI scripts. For a window-less smoke test use the headless command in "Reproducing end to end".
- **`cannot open serial port ... Permission denied`, monitor exit code 2**: you are not in the `dialout` group (`groups`; `sudo usermod -aG dialout $USER`, then log out and back in). Or the port is missing (`--list-ports`) or held by another program.
- **ModemManager grabs `/dev/ttyACM*`** (port opens then vanishes/garbles): stop it while testing (`sudo systemctl stop ModemManager`) or add a udev rule to ignore the board.
- **Boot text / bad_frames right after opening the port**: opening the port toggles DTR/RTS and resets the ESP32, which prints ROM boot text. Those lines fail parsing and are counted in `bad_frames`; the first line after each (re)open is discarded by design. Harmless.
- **Webots does not react**: (1) the simulation must be RUNNING (press Play; `--mode=realtime` starts running); (2) check that UDP 5005 is bound: `ss -uln | grep 5005`; (3) the controller log must show `listening on 127.0.0.1:5005`; (4) sender and controller must agree on port (`--udp-port`, default 5005); (5) the monitor `published : N` count in the summary must be > 0; (6) run Webots with `DOF_FOLLOWER_DEBUG=1` to see applied rotations (must be in Webots' own environment, e.g. `DOF_FOLLOWER_DEBUG=1 ./webots/run_gui.sh`).
- **Controller logs `cannot bind UDP 127.0.0.1:5005` and exits**: port already in use (a second Webots or another program). There is no SO_REUSEADDR by design. Close the other instance (`ss -uanp | grep :5005`).
- **`run_mock_demo.sh` says it cannot find python for env 'esp32-dof'**: create the env (step 1) or set `DOF_PYTHON=/path/to/python`.
- **Publisher with Webots closed**: harmless; UDP datagrams are dropped, monitor keeps running.
- **`ERR,imu_init` every second on the serial stream**: IMU not found; see `firmware/README.md`. For an MPU-6050 it is followed by `ERR,imu_whoami,0xNN` when the chip answered with an unexpected ID.

## Known limitations

- **Yaw drifts without bound** over time: there is no magnetometer, so heading has no absolute reference.
- **6-DOF only**: accelerometer + gyroscope; no position, no magnetometer, no linear-acceleration or free-fall rejection.
- Roll/pitch degrade near pitch = ±90 degrees (gimbal region, Euler angles).
- Serial reconnect assumes the device comes back under the same port name.
- Hardware, GUI look, and rotation handedness are not human-verified yet (see checklist).
- Also see the Fusion and Monitor sections' limitation lists.

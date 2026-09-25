# ESP32 6-DOF IMU POC (issue #227)

Proof-of-concept integration of an external **XIAO ESP32-S3 Sense** microcontroller with a 6-DOF inertial measurement unit (IMU) for orientation tracking and control feedback in the inverted pendulum project. This scaffold provides the host-side monitoring, testing, and simulation integration framework; firmware authoring and hardware assembly remain external.

## Purpose

Establish a minimal, testable framework for:
- **Firmware communication**: Serial protocol over USB/UART to stream IMU data (6-DOF: 3-axis accelerometer + 3-axis gyroscope) from the ESP32 to the host PC.
- **Host-side monitor**: Python script that decodes incoming IMU packets, validates data integrity, and logs telemetry.
- **Webots integration**: Optional simulation validation — drive a Webots robot model with recorded or live IMU data to visually confirm orientation tracking.

This POC is scaffolded as an empty skeleton. All three components (firmware, monitor, Webots) will be populated in sub-issue #239.

## Hardware

- **Microcontroller**: XIAO ESP32-S3 Sense (Seeed Studio)
  - ⚠️ **IMPORTANT**: The Sense variant does *not* include an onboard IMU. Do NOT assume one is present.
- **IMU Sensor**: LSM6DS3TR-C (STMicroelectronics), interfaced externally via I2C
  - 3-axis accelerometer (±2/4/8/16 g configurable)
  - 3-axis gyroscope (±125/250/500/1000/2000 dps configurable)
  - Connected via I2C to D4 (GPIO5, SDA) + D5 (GPIO6, SCL) on the XIAO ESP32-S3 Sense; address 0x6A (0x6B if SDO high)

## Env setup

This project provides a mamba environment (`esp32-dof`) with Python 3.11, NumPy, PySerial, and pytest.

### One-time installation

```bash
# Use the mamba-envs.lock.yml (pinned/reproducible):
mamba env create -f mamba-envs.lock.yml

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

## Layout

```
poc/esp32-dof/
  mamba-envs.yaml         Custom schema env recipe (use lock or setup_command instead)
  mamba-envs.lock.yml     Pinned/reproducible env export (use this with `mamba env create -f`)
  README.md               This file
  firmware/               Arduino sketch and documentation (build artifacts gitignored)
    README.md             Firmware setup, compilation, and testing guide
    esp32_dof/            Firmware sketch directory (folder name matches .ino basename)
      esp32_dof.ino       XIAO ESP32-S3 + LSM6DS3TR-C frame streamer
  monitor/                Host-side Python monitor script + tests
    dof_frame.py          Frame protocol parser and formatter
    test_esp32dof_frame.py Frame protocol unit tests
    dof_fusion.py         Complementary filter, radians out
    test_esp32dof_fusion.py Complementary filter unit tests
    dof_sources.py        Mock and CSV-replay frame line sources (Iterator[str])
    test_esp32dof_sources.py Source unit tests
  webots/                 Webots integration (world files, controllers)
    worlds/               Webots world files (`.wbt`) and temporary `.wbproj` (gitignored)
      .gitkeep            Placeholder for initial commit
    controllers/          Webots robot controller scripts
      .gitkeep            Placeholder for initial commit
      esp32_dof_follower/ Webots controller for ESP32 DOF tracking
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

A **source** is an `Iterator[str]` of raw frame lines (non-empty, stripped of `\r\n` trailing whitespace; consumers pass each to `dof_frame.parse_line()`). This contract matches the serial source planned for issue #235.

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

**⚠️ GUI verification pending (#239):** Sign conventions between this ZYX Euler model and Webots' visual rendering have NOT yet been verified in the live simulation GUI. Expected behavior:
  - roll=π/2 → body tilts about +X with nose staying on +X axis
  - yaw=π/2 → nose points +Y (North, ENU frame)

Once verified in the Webots GUI, these conventions will be either confirmed or corrected (see issue #239).

## TODO: run steps

See sub-issue #239 for the implementation plan:
- Firmware: see firmware/README.md (issue #233)
- Monitor: Python script to decode packets, validate checksums, and log telemetry.
- Webots: Minimal world + controller to visualize IMU orientation in simulation.

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
  - Connected via I2C to GPIO pins 1 (SDA) + 0 (SCL) on the XIAO ESP32-S3 Sense

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
  firmware/               Arduino/PlatformIO sketches and build artifacts (gitignored)
    .gitkeep              Placeholder for initial commit
  monitor/                Host-side Python monitor script + tests
    dof_frame.py          Frame protocol parser and formatter
    test_esp32dof_frame.py Frame protocol unit tests
  webots/                 Webots integration (world files, controllers)
    worlds/               Webots world files (`.wbt`) and temporary `.wbproj` (gitignored)
      .gitkeep            Placeholder for initial commit
    controllers/          Webots robot controller scripts
      .gitkeep            Placeholder for initial commit
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

## TODO: run steps

See sub-issue #239 for the implementation plan:
- Firmware: Arduino sketch to read LSM6DS3TR-C and stream IMU data over serial.
- Monitor: Python script to decode packets, validate checksums, and log telemetry.
- Webots: Minimal world + controller to visualize IMU orientation in simulation.

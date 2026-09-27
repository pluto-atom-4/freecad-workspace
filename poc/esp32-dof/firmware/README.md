# esp32-dof firmware (issue #233)

Arduino sketch `esp32_dof/esp32_dof.ino`: reads an EXTERNAL MPU-6050 (e.g. GY-521, I2C) on a Seeed XIAO ESP32-S3 (Sense) and streams 50 Hz ASCII frames over USB CDC serial:

    IMU,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>*<HH>\n

Protocol: see `../README.md` (Frame protocol). Camera, mic, SD and WiFi are NOT used.
The Sense board has NO onboard IMU.

## Wiring

| IMU module | XIAO ESP32-S3 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | D4 (GPIO5) |
| SCL | D5 (GPIO6) |

I2C address: `0x68` (AD0 low/floating, sketch default) or `0x69` (AD0 high). If your module
uses 0x69, edit `IMU_I2C_ADDR` in the sketch. An I2C scan should find `0x68`.
Power the module from 3V3.

## Status

Compiled with arduino-cli 1.5.1, esp32:esp32 3.3.12, Adafruit MPU6050 2.2.9 (with Adafruit BusIO 1.17.4 and Adafruit Unified Sensor 1.1.15): the sketch uses 309581 bytes (9%) of program storage and 23176 bytes (7%) of dynamic memory. NOT flashed or tested on hardware. MPU-6050 support is compile-checked only and needs human hardware verification.

## Setup (arduino-cli)

```bash
arduino-cli config init   # once, if no config exists
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install "Adafruit MPU6050"
```

This also installs its dependencies (Adafruit BusIO, Adafruit Unified Sensor) automatically. Its declared dependency list also names Adafruit GFX Library and Adafruit SSD1306, which arduino-cli installs too; the sketch does not need them (only the library's OLED example does).

Compile (USB CDC On Boot must be enabled):

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:CDCOnBoot=cdc poc/esp32-dof/firmware/esp32_dof
```

Upload (find port with `arduino-cli board list`; usually /dev/ttyACM0):

```bash
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:XIAO_ESP32S3:CDCOnBoot=cdc poc/esp32-dof/firmware/esp32_dof
```

If upload fails, hold BOOT, tap RESET, release BOOT, then retry.
Do not pass `--output-dir` inside the repo.

## Setup (Arduino IDE)

1. Boards Manager: install "esp32 by Espressif Systems".
2. Library Manager: install "Adafruit MPU6050" (accept installing its dependencies).
3. Tools > Board: "XIAO_ESP32S3"; Tools > USB CDC On Boot: "Enabled".
4. Open `esp32_dof/esp32_dof.ino`, select the port, Upload.

## Check the stream

```bash
mamba run -n esp32-dof python3 -m serial.tools.miniterm /dev/ttyACM0 115200
```

Expect ~50 lines/s like `IMU,12,240000,0.012000,...*3F`. If you see `ERR,imu_init`
every second, the IMU was not found (check wiring, address 0x68/0x69, 3V3). A following `ERR,imu_whoami,0xNN` line means the chip answered with a different ID (a clone or a wrong part; the library only accepts 0x68); no such line means no answer on I2C.
Linux needs the `dialout` group (see `../README.md`).

## Verify frames against the Python parser

Run from repo root; parses 20 lines, checks checksum + byte-for-byte round trip:

```bash
mamba run -n esp32-dof python3 -c "
import sys, serial
sys.path.insert(0, 'poc/esp32-dof/monitor')
from dof_frame import parse_line, format_frame
s = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
s.readline()  # discard possible partial first line
for _ in range(20):
    line = s.readline().decode('ascii')
    f = parse_line(line)
    assert format_frame(f) == line, (line, format_frame(f))
    print(f.seq, f.t_us, f.accel, f.gyro)
print('OK')
"
```

## Golden-frame self-test

Set `#define DOF_SELFTEST 1` in the sketch (or build with `-DDOF_SELFTEST=1`), flash, and read the first line. It must be exactly:

    IMU,12345,1234567890,0.500000,-9.806650,1.250000,10.500000,-5.500000,0.000000*52

Set it back to 0 afterwards.

## Defaults

Accel +/-4 g, gyro +/-500 dps, DLPF about 44 Hz (`MPU6050_BAND_44_HZ`), internal rate 100 Hz
(sample-rate divisor 9). The library defaults (2 g, 260 Hz, divisor 0) are overridden. A gyro-bias
calibration averages 100 samples (about 1 s) at boot, before the first frame: keep the board still
and flat. If the board moves (any axis spans more than 5 dps) it is skipped and the line
`INFO,gyro_cal_skipped` is printed once (harmless). Build with `-DIMU_GYRO_CALIB=0` to disable it:

    arduino-cli compile ... --build-property "compiler.cpp.extra_flags=-DIMU_GYRO_CALIB=0" ...

The MPU-6050 gyro has a temperature-dependent zero-rate offset, so yaw drift may be faster than expected.
Axes: +1 g on Z when the module lies flat, component side up; mount the X arrow toward the box's
nose (+X); no remapping is done.

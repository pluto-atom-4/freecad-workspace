/*
 * esp32_dof.ino -- XIAO ESP32-S3 (Sense) + external MPU-6050 IMU frame streamer
 * (issue #233, part of #227; switched to the MPU-6050 in issue #252)
 *
 * Emits one ASCII frame per line at 50 Hz over USB CDC serial (115200):
 *   IMU,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>*<HH>\n
 * Protocol is defined in poc/esp32-dof/README.md (Frame protocol) and implemented
 * host-side by poc/esp32-dof/monitor/dof_frame.py. Keep them in sync.
 *
 * Board:   Seeed XIAO ESP32S3 (FQBN esp32:esp32:XIAO_ESP32S3)
 * Setting: USB CDC On Boot = Enabled (FQBN option CDCOnBoot=cdc)
 * Library: "Adafruit MPU6050" (header <Adafruit_MPU6050.h>; needs Adafruit BusIO
 *          and Adafruit Unified Sensor)
 * NOTE: the XIAO ESP32-S3 Sense has NO onboard IMU. The IMU is an EXTERNAL module.
 *
 * Wiring (external MPU-6050 module, e.g. GY-521, I2C):
 *   Module VCC -> XIAO 3V3
 *   Module GND -> XIAO GND
 *   Module SDA -> XIAO D4 (GPIO5)
 *   Module SCL -> XIAO D5 (GPIO6)
 *
 * I2C address: 0x68 (AD0 low or floating, default)
 *              0x69 (AD0 tied high)  -> change IMU_I2C_ADDR below.
 *
 * Axes: accel reports specific force (+1 g on Z when the module lies flat,
 * component side up); gyro follows the right-hand rule about the axes printed on
 * the module. No remapping is done; mount the X arrow toward the "nose".
 *
 * Camera, microphone, SD card and WiFi are NOT used.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Set to 1 to print the golden parity frame once at boot (should end with *52).
// May also be given as -DDOF_SELFTEST=1.
#ifndef DOF_SELFTEST
#define DOF_SELFTEST 0
#endif

// 1 = average the gyro for ~1 s at boot and subtract that bias (board must be
// still; skipped automatically if it moves). 0 = off (-DIMU_GYRO_CALIB=0).
#ifndef IMU_GYRO_CALIB
#define IMU_GYRO_CALIB 1
#endif

static const uint8_t  IMU_I2C_ADDR = 0x68;   // 0x69 if AD0 is high
static const uint32_t FRAME_PERIOD_MS = 20;  // 50 Hz

Adafruit_MPU6050 mpu;

static uint16_t seq = 0;          // wraps at 65536 naturally
static uint32_t nextDue = 0;
static float gyroBias[3] = {0.0f, 0.0f, 0.0f};   // deg/s, set by calibration

// ---------------------------------------------------------------------------
// Chip layer: imuBegin() / imuRead(v). v = {ax,ay,az (g), gx,gy,gz (deg/s)}.
// The Adafruit library returns acceleration in m/s^2 and gyro in rad/s.
// ---------------------------------------------------------------------------

// Read WHO_AM_I (reg 0x75) straight over Wire, only for the failure hint.
// Returns -1 if the device does not answer. (A genuine MPU-6050 reads 0x68.)
static int readWhoAmI() {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write((uint8_t)0x75);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(IMU_I2C_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

static bool imuBegin() {
  // begin() checks WHO_AM_I == 0x68, resets the chip, and sets defaults; the
  // Wire bus is already started with our pins/clock (begin() on a running bus is
  // a no-op on this core), so pins and 400 kHz are kept.
  if (!mpu.begin(IMU_I2C_ADDR, &Wire)) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);     // +/-4 g
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);          // +/-500 dps
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);       // DLPF_CFG=3, ~44 Hz
  mpu.setSampleRateDivisor(9);                      // 1 kHz / (1+9) = 100 Hz
  return true;
}

// Raw reading in g and deg/s, no bias applied. getEvent() always returns true
// (the library ignores I2C errors), so a false return is not expected here.
static bool imuReadRaw(float v[6]) {
  sensors_event_t a, g, t;
  if (!mpu.getEvent(&a, &g, &t)) return false;
  v[0] = a.acceleration.x / SENSORS_GRAVITY_STANDARD;   // m/s^2 -> g
  v[1] = a.acceleration.y / SENSORS_GRAVITY_STANDARD;
  v[2] = a.acceleration.z / SENSORS_GRAVITY_STANDARD;
  v[3] = g.gyro.x * SENSORS_RADS_TO_DPS;                // rad/s -> deg/s
  v[4] = g.gyro.y * SENSORS_RADS_TO_DPS;
  v[5] = g.gyro.z * SENSORS_RADS_TO_DPS;
  return true;
}

static bool imuRead(float v[6]) {
  if (!imuReadRaw(v)) return false;
  v[3] -= gyroBias[0];
  v[4] -= gyroBias[1];
  v[5] -= gyroBias[2];
  return true;
}

#if IMU_GYRO_CALIB
// Average ~1 s (100 samples, 10 ms apart) of gyro. Skipped (bias stays 0) if too
// few valid reads or if any axis spans more than 5 dps (board is moving).
static void imuCalibrateGyro() {
  const int N = 100;
  const float MAX_SPAN_DPS = 5.0f;
  float sum[3] = {0, 0, 0};
  float mn[3] = {1e9f, 1e9f, 1e9f};
  float mx[3] = {-1e9f, -1e9f, -1e9f};
  int good = 0;
  for (int i = 0; i < N; i++) {
    float v[6];
    if (imuReadRaw(v)) {
      for (int a = 0; a < 3; a++) {
        float g = v[3 + a];
        sum[a] += g;
        if (g < mn[a]) mn[a] = g;
        if (g > mx[a]) mx[a] = g;
      }
      good++;
    }
    delay(10);
  }
  bool ok = (good >= 80);
  for (int a = 0; a < 3; a++) {
    if (mx[a] - mn[a] > MAX_SPAN_DPS) ok = false;
  }
  if (!ok) {
    Serial.print("INFO,gyro_cal_skipped\n");
    return;
  }
  for (int a = 0; a < 3; a++) gyroBias[a] = sum[a] / (float)good;
}
#endif

// ---------------------------------------------------------------------------
// Frame layer (unchanged)
// ---------------------------------------------------------------------------

// Format one float as %.6f (fixed, no exponent); "-0.000000" -> "0.000000"
// (must match dof_frame.format_frame). Non-finite values are sent as 0.
static void fmtFloat(char *out, size_t cap, float v) {
  if (!isfinite(v)) v = 0.0f;
  snprintf(out, cap, "%.6f", (double)v);
  if (strcmp(out, "-0.000000") == 0) {
    strcpy(out, "0.000000");
  }
}

// Build a full frame (including '*HH\n') into buf. Returns length, or 0 on overflow.
static size_t buildFrame(char *buf, size_t cap, uint16_t s, uint32_t t_us, const float v[6]) {
  char f[6][24];
  for (int i = 0; i < 6; i++) fmtFloat(f[i], sizeof(f[i]), v[i]);

  int n = snprintf(buf, cap, "IMU,%u,%lu,%s,%s,%s,%s,%s,%s",
                   (unsigned)s, (unsigned long)t_us,
                   f[0], f[1], f[2], f[3], f[4], f[5]);
  if (n <= 0 || (size_t)n + 5 > cap) return 0;   // need "*HH\n" + NUL

  uint8_t cs = 0;
  for (int i = 0; i < n; i++) cs ^= (uint8_t)buf[i];   // 'I' .. char before '*'

  int m = snprintf(buf + n, cap - (size_t)n, "*%02X\n", (unsigned)cs);
  if (m != 4) return 0;
  return (size_t)(n + m);
}

static void haltWithError() {
  for (;;) {
    Serial.print("ERR,imu_init\n");
    int id = readWhoAmI();          // extra hint: what the chip answered (if anything)
    if (id >= 0) {
      char m[40];
      snprintf(m, sizeof(m), "ERR,imu_whoami,0x%02X\n", (unsigned)id);
      Serial.print(m);
    }
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  // Do not block forever: board must run without a host attached.
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { delay(10); }

  Wire.begin(SDA, SCL);   // XIAO ESP32S3: SDA=D4/GPIO5, SCL=D5/GPIO6
  Wire.setClock(400000);

  if (!imuBegin()) {
    haltWithError();
  }

#if DOF_SELFTEST
  {
    const float g[6] = {0.5f, -9.80665f, 1.25f, 10.5f, -5.5f, 0.0f};
    char b[128];
    size_t len = buildFrame(b, sizeof(b), 12345, 1234567890UL, g);
    if (len) Serial.write((const uint8_t *)b, len);   // expect ...0.000000*52
  }
#endif

#if IMU_GYRO_CALIB
  imuCalibrateGyro();     // ~1 s, board must be still; before the first frame
#endif

  nextDue = millis() + FRAME_PERIOD_MS;
}

void loop() {
  uint32_t now = millis();
  if ((int32_t)(now - nextDue) < 0) return;

  nextDue += FRAME_PERIOD_MS;
  if ((int32_t)(now - nextDue) > 100) nextDue = now + FRAME_PERIOD_MS;  // resync if far behind

  float v[6];
  if (!imuRead(v)) return;        // skip this frame if the read failed (seq not consumed)

  char buf[128];
  size_t len = buildFrame(buf, sizeof(buf), seq, (uint32_t)micros(), v);
  seq++;                          // uint16_t wraps 65535 -> 0
  if (len) Serial.write((const uint8_t *)buf, len);   // single write, no interleaving
}

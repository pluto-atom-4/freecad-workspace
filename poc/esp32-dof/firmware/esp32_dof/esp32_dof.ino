/*
 * esp32_dof.ino -- XIAO ESP32-S3 (Sense) + external LSM6DS3TR-C IMU frame streamer
 * (issue #233, part of #227)
 *
 * Emits one ASCII frame per line at 50 Hz over USB CDC serial (115200):
 *   IMU,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>*<HH>\n
 * Protocol is defined in poc/esp32-dof/README.md (Frame protocol) and implemented
 * host-side by poc/esp32-dof/monitor/dof_frame.py. Keep them in sync.
 *
 * Board:   Seeed XIAO ESP32S3 (FQBN esp32:esp32:XIAO_ESP32S3)
 * Setting: USB CDC On Boot = Enabled (FQBN option CDCOnBoot=cdc)
 * Library: "Seeed Arduino LSM6DS3" (header <LSM6DS3.h>)
 * NOTE: the XIAO ESP32-S3 Sense has NO onboard IMU. The IMU is an EXTERNAL module.
 *
 * Wiring (external LSM6DS3TR-C module, I2C):
 *   Module 3V3 -> XIAO 3V3
 *   Module GND -> XIAO GND
 *   Module SDA -> XIAO D4 (GPIO5)
 *   Module SCL -> XIAO D5 (GPIO6)
 *
 * I2C address: 0x6A (SDO/SA0 low or floating, default)
 *              0x6B (SDO/SA0 tied high)  -> change IMU_I2C_ADDR below.
 *
 * Camera, microphone, SD card and WiFi are NOT used.
 */

#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Set to 1 to print the golden parity frame once at boot (should end with *52).
#define DOF_SELFTEST 0

static const uint8_t  IMU_I2C_ADDR = 0x6A;   // 0x6B if SDO is high
static const uint32_t FRAME_PERIOD_MS = 20;  // 50 Hz

LSM6DS3 imu(I2C_MODE, IMU_I2C_ADDR);

static uint16_t seq = 0;          // wraps at 65536 naturally
static uint32_t nextDue = 0;

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

  // Ranges/ODR must be set BEFORE begin(). Library defaults are 16 g / 2000 dps / 416 Hz.
  imu.settings.accelRange = 4;         // +/-4 g
  imu.settings.accelSampleRate = 104;  // Hz
  imu.settings.gyroRange = 500;        // +/-500 dps
  imu.settings.gyroSampleRate = 104;   // Hz

  if (imu.begin() != 0) {              // 0 == IMU_SUCCESS
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

  nextDue = millis() + FRAME_PERIOD_MS;
}

void loop() {
  uint32_t now = millis();
  if ((int32_t)(now - nextDue) < 0) return;

  nextDue += FRAME_PERIOD_MS;
  if ((int32_t)(now - nextDue) > 100) nextDue = now + FRAME_PERIOD_MS;  // resync if far behind

  float v[6];
  v[0] = imu.readFloatAccelX();   // g
  v[1] = imu.readFloatAccelY();
  v[2] = imu.readFloatAccelZ();
  v[3] = imu.readFloatGyroX();    // dps
  v[4] = imu.readFloatGyroY();
  v[5] = imu.readFloatGyroZ();

  char buf[128];
  size_t len = buildFrame(buf, sizeof(buf), seq, (uint32_t)micros(), v);
  seq++;                          // uint16_t wraps 65535 -> 0
  if (len) Serial.write((const uint8_t *)buf, len);   // single write, no interleaving
}

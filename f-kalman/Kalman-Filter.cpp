/*
  # USED CLAUDE PRO FOR THIS SHIT WE STILL NEED TO UNDERSTAND HOW THIS WORKS  #
  
 * ESP32 + MPU9250 (6-axis use) + Kalman Filter -> Pitch / Roll / Yaw
 * Serial output only.
 * ------------------------------------------------------------------
 * - Pitch & Roll: gyro + accelerometer fused with a 2-state Kalman
 *   filter (angle + gyro bias). Drift-free and stable.
 * - Yaw: gyro Z integration only (magnetometer NOT used here), so it
 *   WILL drift slowly over time.
 *
 * Open the Serial Monitor at 115200 baud to see the angles.
 *
 * MPU9250 wiring (I2C):
 *   VCC -> 3V3      GND -> GND
 *   SDA -> GPIO21   SCL -> GPIO22
 *   NCS -> 3V3      (MANDATORY: pulls the chip into I2C mode)
 *   ADO -> GND      (address 0x68; tie to 3V3 for 0x69)
 *   EDA / ECL / INT / FSYNC -> leave unconnected
 */

#include <Wire.h>

// ---------- MPU9250 (MPU6050-register-compatible for accel/gyro) ----------
#define MPU_ADDR        0x68
#define PWR_MGMT_1      0x6B
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_XOUT_H    0x3B
#define WHO_AM_I        0x75    // 0x71 MPU9250 / 0x73 MPU9255 / 0x70 MPU6500

// Scale factors for the ranges set below:
//   Accel +/-2g       -> 16384 LSB/g
//   Gyro  +/-250 dps  -> 131 LSB/(deg/s)
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE  = 131.0f;

const int SDA_PIN = 21;
const int SCL_PIN = 22;

// ---------- Kalman filter (one instance per axis) ----------
struct Kalman {
  float Q_angle   = 0.001f;
  float Q_bias    = 0.003f;
  float R_measure = 0.03f;
  float angle     = 0.0f;
  float bias      = 0.0f;
  float P[2][2]   = {{0,0},{0,0}};
};

float kalmanUpdate(Kalman &k, float newAngle, float newRate, float dt) {
  float rate = newRate - k.bias;
  k.angle += dt * rate;

  k.P[0][0] += dt * (dt * k.P[1][1] - k.P[0][1] - k.P[1][0] + k.Q_angle);
  k.P[0][1] -= dt * k.P[1][1];
  k.P[1][0] -= dt * k.P[1][1];
  k.P[1][1] += k.Q_bias * dt;

  float S = k.P[0][0] + k.R_measure;
  float K0 = k.P[0][0] / S;
  float K1 = k.P[1][0] / S;

  float y = newAngle - k.angle;
  k.angle += K0 * y;
  k.bias  += K1 * y;

  float P00 = k.P[0][0];
  float P01 = k.P[0][1];
  k.P[0][0] -= K0 * P00;
  k.P[0][1] -= K0 * P01;
  k.P[1][0] -= K1 * P00;
  k.P[1][1] -= K1 * P01;

  return k.angle;
}

Kalman kalRoll;
Kalman kalPitch;

// ---------- State ----------
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float yaw = 0.0f;
uint32_t lastMicros = 0;
uint32_t lastPrint  = 0;

// ---------- MPU helpers ----------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();          // skip temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

void mpuInit() {
  mpuWrite(PWR_MGMT_1, 0x00);        // wake up
  delay(100);
  mpuWrite(GYRO_CONFIG,  0x00);      // +/-250 dps
  mpuWrite(ACCEL_CONFIG, 0x00);      // +/-2g
  delay(100);
}

bool mpuConnected() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  uint8_t id = Wire.available() ? Wire.read() : 0x00;
  Serial.printf("WHO_AM_I = 0x%02X\n", id);
  return true;
}

void calibrateGyro(int samples = 1500) {
  long sx = 0, sy = 0, sz = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < samples; i++) {
    mpuReadRaw(ax, ay, az, gx, gy, gz);
    sx += gx; sy += gy; sz += gz;
    delay(2);
  }
  gyroBiasX = (sx / (float)samples) / GYRO_SCALE;
  gyroBiasY = (sy / (float)samples) / GYRO_SCALE;
  gyroBiasZ = (sz / (float)samples) / GYRO_SCALE;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(50);               // don't block forever if the bus stalls

  if (!mpuConnected()) {
    Serial.println("ERROR: no I2C response at 0x68.");
    Serial.println("Check NCS->3V3 (I2C mode), SDA/SCL, ADO->GND, 3V3 power.");
    while (true) delay(1000);        // halt cleanly, no reboot loop
  }

  mpuInit();

  Serial.println("Calibrating gyro -- keep the board still...");
  calibrateGyro();
  Serial.println("Done. Streaming angles:");

  // Seed the Kalman filters from the initial accelerometer angles
  int16_t ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);
  float axg = ax / ACCEL_SCALE;
  float ayg = ay / ACCEL_SCALE;
  float azg = az / ACCEL_SCALE;
  kalRoll.angle  = atan2(ayg, azg) * RAD_TO_DEG;
  kalPitch.angle = atan2(-axg, sqrt(ayg * ayg + azg * azg)) * RAD_TO_DEG;

  lastMicros = micros();
}

// ---------- Main loop ----------
void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);

  float axg = ax / ACCEL_SCALE;
  float ayg = ay / ACCEL_SCALE;
  float azg = az / ACCEL_SCALE;

  float gxr = gx / GYRO_SCALE - gyroBiasX;   // deg/s
  float gyr = gy / GYRO_SCALE - gyroBiasY;
  float gzr = gz / GYRO_SCALE - gyroBiasZ;

  float accRoll  = atan2(ayg, azg) * RAD_TO_DEG;
  float accPitch = atan2(-axg, sqrt(ayg * ayg + azg * azg)) * RAD_TO_DEG;

  uint32_t now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;

  // Roll: handle the +/-180 wrap
  if ((accRoll < -90 && kalRoll.angle > 90) ||
      (accRoll >  90 && kalRoll.angle < -90)) {
    kalRoll.angle = accRoll;
  }
  float roll = kalmanUpdate(kalRoll, accRoll, gxr, dt);

  // Pitch: keep rate consistent past +/-90
  if (abs(roll) > 90) gyr = -gyr;
  float pitch = kalmanUpdate(kalPitch, accPitch, gyr, dt);

  // Yaw: gyro integration only (drifts)
  yaw += gzr * dt;
  if (yaw >= 360.0f) yaw -= 360.0f;
  if (yaw <  0.0f)   yaw += 360.0f;

  // Print at ~20 Hz (filter still runs every loop)
  if (now - lastPrint > 50000) {
    lastPrint = now;
    Serial.printf("Pitch: %6.1f   Roll: %6.1f   Yaw: %6.1f\n", pitch, roll, yaw);
  }
}

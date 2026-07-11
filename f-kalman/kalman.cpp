#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu; // Reverted to default 0x68 since communication is working

float gxo_off = 0, gyo_off = 0, gzo_off = 0;
float axo_off = 0, ayo_off = 0, azo_off = 0;
int samples = 2000;
const float ACCEL_SCALE = 16384.0f;

//for the pid//////////////
float roll, pitch, yawr, rollr, pitchr,dt;
uint32_t now;
//////////////////////////

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
//kalman khatam


uint32_t lastMicros = 0;
uint32_t lastPrint  = 0;


void setup() {
    Wire.begin();
    Serial.begin(115200);

    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    int32_t sumGXO = 0, sumGYO = 0, sumGZO = 0;
    for (int i = 0; i < samples; i++) {
        int16_t gx,gy,gz;
        mpu.getRotation(&gx, &gy, &gz);
        sumGXO += gx;
        sumGYO += gy;
        sumGZO += gz;   
    }

    // 3. Assign true offsets
    gxo_off = (float)sumGXO / samples;
    gyo_off = (float)sumGYO / samples;
    gzo_off = (float)sumGZO / samples;
  lastMicros = micros();
  }


void loop() {
      int16_t ax, ay, az, gx, gy, gz;

    mpu.getAcceleration(&ax, &ay, &az);

      float axg = ax / ACCEL_SCALE;
  float ayg = ay / ACCEL_SCALE;
  float azg = az / ACCEL_SCALE;

    mpu.getRotation(&gx, &gy, &gz);

    // Apply offset corrections
    float gyroPitchRate = gx/131.0f - gxo_off/131.0f;
    float gyroRollRate = gy/131.0f - gyo_off/131.0f;
    float gyroYawRate = gz/131.0f - gzo_off/131.0f;

    // Pitch and Roll calculation
    float ePitch = atan2(ayg, azg) * 180.0f / PI;
    float eRoll  = atan2(-axg, sqrt(ayg*ayg + azg*azg)) * 180.0f / PI;


      now = micros();
      dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;
  if ((eRoll < -90 && kalRoll.angle > 90) ||
      (eRoll >  90 && kalRoll.angle < -90)) {
    kalRoll.angle = eRoll;
  }if (abs(roll) > 90) gyroRollRate = -gyroRollRate;
        roll = kalmanUpdate(kalRoll, eRoll, gyroRollRate, dt);
  
        pitch = kalmanUpdate(kalPitch, ePitch, gyroPitchRate, dt);
        yawr = gyroYawRate;
        rollr = gyroRollRate;
        pitchr = gyroPitchRate;

if (now - lastPrint > 50000) {
    lastPrint = now;
    Serial.print(pitch);Serial.print(",");
    Serial.print(roll);Serial.print(",");
    Serial.println(gyroYawRate);Serial.print(",");
  }
}

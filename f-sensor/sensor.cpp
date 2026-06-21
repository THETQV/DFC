#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu; // Reverted to default 0x68 since communication is working

float gxo_off = 0, gyo_off = 0, gzo_off = 0;
float axo_off = 0, ayo_off = 0, azo_off = 0;

void setup() {
    Serial1.begin(115200); 
    
    
    Wire.begin();   
    
    
    mpu.initialize();

    int samples = 2000;
    int i = 0;
    
    // CHANGED TO int32_t TO PREVENT INTEGER OVERFLOW DURING CALIBRATION
    int32_t sumAXO = 0, sumAYO = 0, sumAZO = 0;
    int32_t sumGXO = 0, sumGYO = 0, sumGZO = 0;

    Serial1.println("Calibrating... Keep IMU perfectly still.");

    while(i < samples) {
        int16_t ax_raw, ay_raw, az_raw;
        mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
        sumAXO += ax_raw;
        sumAYO += ay_raw;
        sumAZO += (az_raw - 16384); // 16384 LSB = 1g at default 2g sensitivity

        delay(1);
        
        int16_t gx_raw, gy_raw, gz_raw;
        mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);
        sumGXO += gx_raw;
        sumGYO += gy_raw;
        sumGZO += gz_raw;   
        
        i++;
        delay(1);
    }
    
    axo_off = (float)sumAXO / samples;
    ayo_off = (float)sumAYO / samples;
    azo_off = (float)sumAZO / samples;
    gxo_off = (float)sumGXO / samples;
    gyo_off = (float)sumGYO / samples;
    gzo_off = (float)sumGZO / samples;
    
    Serial1.println("Calibration complete!");
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    // Apply offset corrections
    float ax_e = ax - axo_off;
    float ay_e = ay - ayo_off;
    float az_e = az - azo_off;
    float gx_e = gx - gxo_off;
    float gy_e = gy - gyo_off;
    float gz_e = gz - gzo_off;

    // Pitch and Roll calculation
    float ePitch = atan2(ay_e, az_e) * 180.0f / PI;
    float eRoll  = atan2(-ax_e, sqrt(ay_e*ay_e + az_e*az_e)) * 180.0f / PI;
float dt = 0.01; // 10ms loop
    // Scale raw values to degrees per second based on standard FS_SEL multiplier
    float gyroPitchRate = gx_e / 131.0f;
    float gyroRollRate  = gy_e / 131.0f;
    float gyroYawRate   = gz_e / 131.0f;


    Serial1.print(ePitch);Serial1.print(",");
    Serial1.print(eRoll);Serial1.print(",");
    Serial1.println(gyroYawRate);Serial1.print(",");

    delay(80);
}
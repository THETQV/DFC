#include <Wire.h>
#include <MPU6050.h>



    
MPU6050 mpu;
float gxo_off = 0, gyo_off = 0, gzo_off = 0;
    float axo_off = 0, ayo_off = 0, azo_off = 0;
void setup() {
    Serial.begin(115200);
    delay(10);

      
      

    
   Wire.begin(); 
   delay(5);
   mpu.initialize();
   int samples = 2000;
   int i = 0;
    int sumAXO = 0, sumAYO = 0, sumAZO = 0;
   int sumGXO = 0, sumGYO = 0, sumGZO = 0;

   while(i < samples) {
    
    //for acceleration
    int16_t ax_raw, ay_raw, az_raw;
    mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
    sumAXO += ax_raw;
    sumAYO += ay_raw;
    sumAZO += (az_raw - 16384);
    //for gyroscope
    delay(1);
    int16_t gx_raw, gy_raw, gz_raw;
    mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);
    sumGXO += gx_raw;
    sumGYO += gy_raw;
    sumGZO += gz_raw;   
    //get offsets
    
    i++;
    delay(5);

    
}
axo_off = sumAXO / samples;
    ayo_off = sumAYO / samples;
    azo_off = sumAZO / samples;
    gxo_off = sumGXO / samples;
    gyo_off = sumGYO / samples;
    gzo_off = sumGZO / samples;

}
void loop(){
//get estimated values final
int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

float ax_e = ax - axo_off;
float ay_e = ay - ayo_off;
float az_e = az - azo_off;
float gx_e = gx - gxo_off;
float gy_e = gy - gyo_off;
float gz_e = gz - gzo_off;



//now p,y,r
    float ePitch = atan2(ay_e, az_e) * 180.0f / PI;
    float eRoll  = atan2(-ax_e, sqrt(ay_e*ay_e + az_e*az_e)) * 180.0f / PI;
    float gyroPitchRate = gx_e / 131.0f;
    float gyroRollRate  = gy_e / 131.0f;
    float gyroYawRate   = gz_e / 131.0f;

  
    Serial.print("Estimated Pitch: ");
    Serial.print(ePitch);
    Serial.print(" | Estimated Roll: ");
    Serial.print(eRoll);      
    Serial.print(" | Gyro Pitch Rate: ");
    Serial.print(gyroPitchRate);
    Serial.print(" | Gyro Roll Rate: ");
    Serial.print(gyroRollRate);
    Serial.print(" | Gyro Yaw Rate: ");
    Serial.println(gyroYawRate);
    delay(100);

}

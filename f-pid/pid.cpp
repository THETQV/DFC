#include <Arduino.h>

float roll, pitch, yawr, pitchr, rollr;
int rolld, pitchd, yawd;

float Kp = 5, Ki = 0.5, Kd = 2;
float kp = 0.0, kd = 0.0, ki = 0.0 ;
uint32_t lastMicros = 0;
float dt;
int roll_out, pitch_out, yawr_out;

void setup(){lastMicros = micros();}
//for x = 1, Kp,Ki,Kd
// for x = 0, kp,ki,kd
int pid(float err, float lerr, int x){
    float ir, dr;int total;
      ir += err * dt;
      dr = (err - lerr) / dt;
      if(x==1){
        total = (Kp * err) + (Ki * ir) + (Kd * dr);
      }
      if(x==0){
        total = (kp * err) + (ki * ir) + (kd * dr);
      }
      else{total = 0;}
      return total;




}





float Rlerror = 0;
float Plerror = 0;
float Ylerror = 0;




void loop(){
      now = micros();
      dt = (now - lastMicros) / 1000000.0f;
      lastMicros = now;
      if (dt <= 0 || dt > 0.5f) dt = 0.01f;

      
      
      float Rerror =  rolld - roll;
      int roll_out = pid(Rerror,Rlerror,1);
      float Rlerror = Rerror;



      float Perror = pitchd - pitch;
      int pitch_out = pid(Perror,Plerror,1);
      float Plerror = Perror;



      float Yerror = yawrd - yawr;
      int yaw_out = pid(Yerror,Ylerror,0);
      float Ylerror = Yerror;

      




    
}

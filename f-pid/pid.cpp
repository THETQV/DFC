#include <Arduino.h>

float roll, pitch, yawr, pitchr, rollr;
int rolld, pitchd, yawd;

float Kp = 5;
//1 is for roll/pitch
//2 is for yaw
float kp1 = 0.0, kd1 = 0.0, ki1 = 0.0 ;
float kp2 = 0.0, kd2 = 0.0, ki2 = 0.0 ;

uint32_t lastMicros = 0;
float dt;
int roll_out, pitch_out, yawr_out;
float  r_d = 0; float err2 = 0;

void setup(){lastMicros = micros(); }



//determine desired rate
int R_d(int angled, float anglea){
  float err1 = angled- anglea;
  r_d = err1*Kp;
}





//ek function aisa bhi
int pid(float r_d, float angler, float lerror, int mode){
   err2 = r_d - angler;
  float i += err2*dt;
    float d = (err2-lerror)/dt;
  if(mode==1){
    int total = kp1*err2+ ki1*i + kd1*d;
  }
  if(mode==2){
    int total = kp2*err2 + ki2*i + kd2*d;
  }
  return total;

}



void loop(){

float Rlerror = 0;
float Plerror = 0;
float Ylerror = 0;


      now = micros();
      dt = (now - lastMicros) / 1000000.0f;
      lastMicros = now;
      if (dt <= 0 || dt > 0.5f) dt = 0.01f;

//ROLL
      //first rate calc
      R_d(rolld,roll);
      //then actual inner pid
      roll_out = pid(r_d,rollr,Rlerror,1);
      //update last error
      Rlerror = err2;


//PITCH
       //first rate calc
      R_d(pitchd,pitch);
      //then actual inner pid
      pitch_out = pid(r_d,pitchr,Plerror,1);
      //update last error
      Plerror = err2;


//YAW
      
      //then actual inner pid
      yawr_out = pid(yawd,yawr,Ylerror,2);
      //update last error
      Ylerror = err2;


}

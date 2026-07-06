#include <Arduino.h>

float roll, pitch, yawr, pitchr, rollr;
int rolld, pitchd, yawd;

float Kp = 5;
//r,p is for roll/pitch
//y is for yaw
float kp1 = 0.0, kd1 = 0.0, ki1 = 0.0 ;
float kp2 = 0.0, kd2 = 0.0, ki2 = 0.0 ;

uint32_t lastMicros = 0, now = 0;
float dt;
float roll_out, pitch_out, yawr_out;
float  r_d = 0; float err2 = 0;

float Rlerror = 0;
float Plerror = 0;
float Ylerror = 0;

void setup(){lastMicros = micros(); }



//determine desired rate
void R_d(int angled, float anglea){
  float err1 = angled- anglea;
  r_d = err1*Kp;
  r_d = constrain(r_d, -300, 300);
}


float i1, i2, i3;


//ek function aisa bhi
float pid(float r_d, float angler, float lerror,char x){
   err2 = r_d - angler;
   float d = (err2-lerror)/dt;
   
   float total;
   if (x=='p'){i1 += err2*dt; i1 = constrain(i1, -100, 100); total = kp1*err2+ ki1*i1 + kd1*d;}
   if (x=='r'){i2 += err2*dt; i2 = constrain(i2, -100, 100);total = kp1*err2+ ki1*i2 + kd1*d;}
   if (x=='y'){i3 += err2*dt; i3 = constrain(i3, -100, 100);total = kp2*err2+ ki2*i3 + kd2*d;}
  
  return total;

}



void loop(){


      now = micros();
      dt = (now - lastMicros) / 1000000.0f;
      lastMicros = now;
      if (dt <= 0 || dt > 0.5f) dt = 0.01f;

//ROLL
      //first rate calc
      R_d(rolld,roll);
      //then actual inner pid
      roll_out = pid(r_d,rollr,Rlerror,'r');
      //update last error
      Rlerror = err2;


//PITCH
       //first rate calc
      R_d(pitchd,pitch);
      //then actual inner pid
      pitch_out = pid(r_d,pitchr,Plerror,'p');
      //update last error
      Plerror = err2;


//YAW
      
      //then actual inner pid
      yawr_out = pid(yawd,yawr,Ylerror,'y');
      //update last error
      Ylerror = err2;


}

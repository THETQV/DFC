#include <Arduino.h>
//radio wala
#define CH1 25
#define CH2 26
#define CH3 27
#define CH4 14
//max value for r,p,y.
float mr = 40.0;
float mp = 40.0;
float my = 40.0;
//define radio input default
volatile uint32_t ch1_start, ch2_start, ch3_start, ch4_start;
volatile uint16_t ch1_val = 1500;
volatile uint16_t ch2_val = 1500;
volatile uint16_t ch3_val = 1500;
volatile uint16_t ch4_val = 1500;

void IRAM_ATTR ch1_isr() {
  if (digitalRead(CH1))
    ch1_start = micros();
  else
    ch1_val = micros() - ch1_start;
}

void IRAM_ATTR ch2_isr() {
  if (digitalRead(CH2))
    ch2_start = micros();
  else
    ch2_val = micros() - ch2_start;
}

void IRAM_ATTR ch3_isr() {
  if (digitalRead(CH3))
    ch3_start = micros();
  else
    ch3_val = micros() - ch3_start;
}

void IRAM_ATTR ch4_isr() {
  if (digitalRead(CH4))
    ch4_start = micros();
  else
    ch4_val = micros() - ch4_start;
}

void setup() {
  Serial.begin(115200);

  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);

  attachInterrupt(CH1, ch1_isr, CHANGE);
  attachInterrupt(CH2, ch2_isr, CHANGE);
  attachInterrupt(CH3, ch3_isr, CHANGE);
  attachInterrupt(CH4, ch4_isr, CHANGE);


}

void loop() {
  float rolld = ((ch1_val - 1500.0) / 500.0) * mr;
  float pitchd = ((ch2_val - 1500.0) / 500.0) * mp;
  float yawd = ((ch4_val - 1500.0) / 500.0) * my;
  float throttle = ch3_val;

  Serial.print(rolld);
  Serial.print(pitchd);
  Serial.println(yawd);
  }

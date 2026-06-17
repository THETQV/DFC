
#include <Arduino.h>
#define m_1 33

// Helper function to convert microseconds to 16-bit PWM duty cycle
uint32_t usToDuty(uint32_t us) {
  return (uint32_t)((uint64_t)us * 16383UL / 2223UL);
}

void setup() {

  // Attach pin 18 with 50Hz frequency and 16-bit resolution
  ledcAttach(m_1, 450, 14);

  // --- ESC ARMING SEQUENCE ---
  // Most ESCs need to see 1000us (Low) for a few seconds to unlock
  
  ledcWrite(m_1, usToDuty(1000));
  delay(3000);

  
}

void loop() {
  // 1. Increasing from 1100 to 1600

  for (int speed = 1100; speed <= 1600; speed += 50) {
    ledcWrite(m_1, usToDuty(speed));
  delay(200); // Wait 200ms at each step so you can see/hear the change
    
    
  }

  

  
  
  for (int speed = 1600; speed >= 1100; speed -= 50) {
    ledcWrite(m_1, usToDuty(speed));
  }

  
}
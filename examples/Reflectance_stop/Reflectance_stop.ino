#include <Wire.h>
#include <ZumoShield.h>

#define REFLECTANCE_THRESHOLD  400

void setup() {
  buzzer.playOn();
  Serial.begin(9600);
  Serial.println("Zumo sample Start!");
  button.waitForPress();
}

void loop() {
  reflectances.update();

  while ((reflectances.value(3) < REFLECTANCE_THRESHOLD) && 
          (reflectances.value(4) < REFLECTANCE_THRESHOLD)) {
    motors.setSpeeds(100, 100);
    led.on();
    reflectances.update();
  }
  motors.setSpeeds(0, 0);
  led.off(); 
}

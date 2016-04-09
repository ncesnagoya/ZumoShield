#include <Wire.h>
#include <ZumoShield.h>

void setup() {
  
}

int led_state = 0;

void loop() {
  button.waitForPress();
  button.waitForRelease();
  if(led_state == 0) {
    led_state = 1;
    led.on();
  }else if(led_state == 1) {
    led_state = 0;
    led.off();    
  }
}

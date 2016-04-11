#include <Wire.h>
#include <ZumoShield.h>

void setup() {
 //一度だけ実行 
}

void loop() {
  //LEDをONに
  led.on();
  //1秒経過するのを待つ
  delay(1000);
  //LEDをOFFに
  led.off();
  delay(1000);
}

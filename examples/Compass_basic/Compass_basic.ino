#include <Wire.h>
#include <ZumoShield.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  
  compass.begin();
}

void loop() {
  float heading;

  heading = compass.averageHeading();
  Serial.print("Heading: ");
  Serial.println(heading);
}


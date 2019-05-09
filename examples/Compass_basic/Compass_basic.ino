#include <Wire.h>
#include <ZumoShield.h>

void setup() {
  Serial.begin(9600);  
  compass.begin();

  button.waitForButton();
   
  Serial.println("starting calibration");
  compass.doCalibration();
//  compass.setCalibration(, , , );
  
  Serial.print("max.x   ");
  Serial.print(compass.m_max.x);
  Serial.println();
  Serial.print("max.y   ");
  Serial.print(compass.m_max.y);
  Serial.println();
  Serial.print("min.x   ");
  Serial.print(compass.m_min.x);
  Serial.println();
  Serial.print("min.y   ");
  Serial.print(compass.m_min.y);
  Serial.println();

  button.waitForButton();
}

void loop() {
  float heading;

  heading = compass.averageHeading();
  Serial.print("Heading: ");
  Serial.println(heading);
}

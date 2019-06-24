#include <external_VL6180X.h>
#include <Wire.h>

VL6180X laser = VL6180X();

void setup(){
  Serial.begin(9600);
  Wire.begin();
  Serial.println("LiDAR initialised");
}

void loop(){
  Serial.println("entered loop");
  uint8_t range = laser.readRangeSingle();
  Serial.println(range);
  delay(1000);
}

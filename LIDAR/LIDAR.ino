#include <external_VL6180X.h>
#include <Wire.h>

VL6180X laser1;

int laser1Pin = 9;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  pinMode(laser1Pin, OUTPUT);
  digitalWrite(laser1Pin, LOW);
  delay(1000);
  digitalWrite(laser1Pin, HIGH);
  delay(50);

  Serial.println("Initialising LiDAR 1");
  laser1.init();
  laser1.setAddress(0x32);
  laser1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  laser1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  laser1.setTimeout(500);
  laser1.stopContinuous();
  laser1.setScaling(1);
  delay(300);
  laser1.startInterleavedContinuous(100);
  delay(100);
  Serial.println("LiDAR initialised");
}

void loop(){
  Serial.println("entered loop");
  uint8_t range = laser1.readRangeContinuousMillimeters();
  Serial.println(range);
  delay(1000);
}

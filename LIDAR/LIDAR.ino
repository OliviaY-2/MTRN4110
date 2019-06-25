#include <external_VL6180X.h>
#include <Wire.h>

VL6180X laser1;
VL6180X laser2;

int laser1Pin = 9; //use 28 on mega
int laser2Pin = 10; //use 29 on mega
int address1 = 0x30;
int address2 = 0x32;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  pinMode(laser1Pin, OUTPUT);
  pinMode(laser2Pin, OUTPUT);
  
  digitalWrite(laser1Pin, LOW);
  digitalWrite(laser2Pin, LOW);
  delay(1000);
  digitalWrite(laser1Pin, HIGH);
  delay(50);

  Serial.println("Initialising LiDAR 1");
  laser1.init();
  laser1.configureDefault();
  laser1.setAddress(address1);
  laser1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  laser1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  laser1.setTimeout(500);
  laser1.stopContinuous();
  laser1.setScaling(1);
  delay(300);
  laser1.startInterleavedContinuous(100);
  delay(100);
  Serial.println("LiDAR 1 initialised");

  Serial.println("Initialising LiDAR 2");
  digitalWrite(laser2Pin, HIGH);
  delay(50);
  laser2.init();
  laser2.configureDefault();
  laser2.setAddress(address2);
  laser2.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  laser2.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  laser2.setTimeout(500);
  laser2.stopContinuous();
  laser2.setScaling(1);
  delay(300);
  laser2.startInterleavedContinuous(100);
  delay(100);
  Serial.println("LiDAR 2 initialised");
}

void loop(){
  int range1 = laser1.readRangeContinuousMillimeters();
  int range2 = laser2.readRangeContinuousMillimeters();
  Serial.print("Range from LiDAR 1: ");
  Serial.print(range1);
  Serial.println(" mm");
  Serial.print("Range from LiDAR 2: ");
  Serial.print(range2);
  Serial.println(" mm");
  delay(1000);
}

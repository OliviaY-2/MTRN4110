/*
///////////////////////////////////////////////
// Laser V2.04 changed
///////////////////////////////////////////////

#include <external_VL6180X.h>
#ifndef WIRE_H_INCLUDED
#define WIRE_H_INCLUDED
#include <Wire.h>
#endif


//use 33 on mega
#define LASER1PIN 33
#define LASER1ADDRESS 0x32

//use 32 on mega
#define LASER2PIN 32
#define LASER2ADDRESS 0x30

VL6180X laser1;
VL6180X laser2;

void initLaser(VL6180X &laser, int laserPin, int address);
void testingLaser(VL6180X &laser, int laserNum);
void testLasers();

///////////////////////////////////////////////
*/

void initLaser(VL6180X &laser, int laserPin, int address) {
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);
  delay(50);
  digitalWrite(laserPin, HIGH);
  delay(50);

  laser.init();
  laser.configureDefault();
  laser.setAddress(address);
  laser.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  laser.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  laser.setTimeout(500);
  laser.stopContinuous();
  laser.setScaling(1);
  delay(300);
  laser.startInterleavedContinuous(100);
  delay(100);
}


void testingLaser(VL6180X &laser, int laserNum) {
  double distance = laser.readRangeContinuousMillimeters();

  Serial.print(" tL:");
  Serial.print(" laser "); Serial.print(laserNum);
  Serial.print(" distance mm "); Serial.print(distance);
  Serial.println();  
}


void testLasers() {
  for (int i = 0; i < 9; i++)
  {
    testingLaser(laser1,1);
    testingLaser(laser2,2);
  }
}


//

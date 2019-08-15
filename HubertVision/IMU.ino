/*
///////////////////////////////////////////////
// IMU V2.01
///////////////////////////////////////////////

#include <MPU6050_tockn.h>

#ifndef WIRE_H_INCLUDED
#define WIRE_H_INCLUDED
#include <Wire.h>
#endif

MPU6050 mpu6050(Wire);

void initIMU();
float readAngleZ();

///////////////////////////////////////////////
*/


void initIMU() {
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);
}


float readAngleZ() {
  mpu6050.update();
  return (mpu6050.getAngleZ());
}


//

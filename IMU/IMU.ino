#include <external_MPU6050_6Axis_MotionApps20.h>
#include <external_MPU6050.h>
#include <external_MPU6050_helper_3dmath.h>
#include <external_MPU6050_I2Cdev.h>
//#include <external_MPU6050.cpp>
//#include <external_MPU6050_I2Cdev.cpp>

#define INTERRUPT 2

MPU6050 IMU; //create MPU6050 object named IMU
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup(){
  Serial.begin(9600);
  pinMode(INTERRUPT, INPUT);

  Serial.println("Initialising IMU");
  IMU.initialize(); //set up IMU
  devStatus = IMU.dmpInitialize();

  if (devStatus == 0) {
    Serial.println("DMP initialisation successful");
    IMU.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT), dmpDataReady, RISING);  //set up interrupt
    mpuIntStatus = IMU.getIntStatus();
    dmpReady = true;
    packetSize = IMU.dmpGetFIFOPacketSize();
  } else {
    Serial.println("DMP initialisation failed");
  }
}

void loop() {
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = IMU.getFIFOCount();
    }      
  }
  mpuInterrupt = false; //reset interrupt flag and get INT_STATUS byte
  mpuIntStatus = IMU.getIntStatus();
  fifoCount = IMU.getFIFOCount();

}

#include <external_VL6180X.h>
#include <external_MPU6050_6Axis_MotionApps20.h>
#include <external_MPU6050.h>
#include <external_MPU6050_helper_3dmath.h>
#include <external_MPU6050_I2Cdev.h>
#include <Adafruit_VL6180X.h>
#include <Wire.h>

#define INTERRUPT 2

VL6180X laser1;
VL6180X laser2;
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
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

long receivedNum;
boolean newData = false;
int trigPin = 11; //use 30/1 on mega
int echoPin = 12; //use 30/1 on mega
int laser1Pin = 9; //use 28 on mega
int laser2Pin = 10; //use 29 on mega
int address1 = 0x30;
int address2 = 0x32;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(INTERRUPT, INPUT);
  pinMode(laser1Pin, OUTPUT);
  pinMode(laser2Pin, OUTPUT);
  
  digitalWrite(laser1Pin, LOW);
  digitalWrite(laser2Pin, LOW);
  delay(1000);

  Serial.println("Initialising LiDAR 1");
  digitalWrite(laser1Pin, HIGH);
  delay(50);
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

//  Serial.println("Initialising IMU");
//  IMU.initialize(); //set up IMU
//  Serial.println("initialising DMP");
//  devStatus = IMU.dmpInitialize();
//
//  if (devStatus == 0) {
//    Serial.println("DMP initialisation successful");
//    IMU.setDMPEnabled(true);
//    attachInterrupt(digitalPinToInterrupt(INTERRUPT), dmpDataReady, RISING);  //set up interrupt
//    mpuIntStatus = IMU.getIntStatus();
//    dmpReady = true;
//    packetSize = IMU.dmpGetFIFOPacketSize();
//  } else {
//    Serial.println("DMP initialisation failed");
//  }
  
  Serial.println("Please enter a number 1 - 4");
  
}

void loop() {
  recvNum();
  decisionTree();
}

void recvNum() {
  if (Serial.available() > 0) {
    receivedNum = Serial.parseInt();
    newData = true;
   }
}

void decisionTree() {
  if (newData == true) {
    switch (receivedNum){
      case 1:
        function1();
        break;
      case 2:
        function2();
        break;
      case 3:
        function3();
        break;
      case 4:
        function4();
        break;
      default:
        Serial.println("Please enter a number 1 - 4");
        break;
    }
    newData = false;
  }
}

void function1 () {
  Serial.println("Entered 1");

  //IMU
//  IMUmeasurement();
//  Serial.print("Roll: ");
//  Serial.println(ypr[2] * 180/M_PI);
//  Serial.print("Pitch: ");
//  Serial.println(ypr[1] * 180/M_PI);
//  Serial.print("Yaw: ");
//  Serial.println(ypr[0] * 180/M_PI);
//  Serial.print("X acceleration: ");
//  Serial.println(aaReal.x);
//  Serial.print("Y acceleration: ");
//  Serial.println(aaReal.y);
//  Serial.print("Z acceleration: ");
//  Serial.println(aaReal.z);

  //ultrasonic sensor
  double cm = ultrasonicRange();
  Serial.print("Distance (ultrasonic): ");
  Serial.print(cm);
  Serial.println(" cm");

  //LiDARs
  uint8_t range1 = laser1.readRangeContinuousMillimeters();
  uint8_t range2 = laser2.readRangeContinuousMillimeters();
  
  Serial.print("Range1 (laser): ");
  Serial.print(range1);
  Serial.println("mm");
  Serial.print("Range2 (laser): ");
  Serial.print(range2);
  Serial.println("mm");
}



void function2 () {
  Serial.println("Entered 2");

  double distance = ultrasonicRange();
  Serial.println(distance);
  while (distance > 7){ //While there is no obstruction, keep measuring the distance
    distance = ultrasonicRange();
    delay(100);
    Serial.println("no");
  } //break from loop when a distance less than 7 cm is detected

  while (distance < 7) { //while there is an obstruction, wait until it is gone
    distance = ultrasonicRange();
    Serial.println("yes");
    delay(100);
  } //break from loop when the obstruction is removed
  Serial.println("Message!");
  Serial.println(distance);
}



void function3 () {
  Serial.println("Entered 3");

  double frontDistance = ultrasonicRange();
  uint8_t leftDistance = laser1.readRangeContinuousMillimeters()/10;
  uint8_t rightDistance = laser2.readRangeContinuousMillimeters()/10;
  Serial.println(leftDistance);
  Serial.println(frontDistance);
  Serial.println(rightDistance);

  if (leftDistance > 7) {
    Serial.print("0 ");
  } else {
    Serial.print("1 ");
  }

  if (frontDistance > 7) {
    Serial.print("0 ");
  } else {
    Serial.print("1 ");
  }

  if (rightDistance > 7) {
    Serial.println("0");
  } else {
    Serial.println("1");
  }
  
}



void function4 () {
  Serial.println("Entered 4");
  Serial.println("N E S W");

  IMUmeasurement();
  float initYaw = ypr[0] * 180/M_PI; //initial yaw in degrees
  uint8_t westDistance = laser1.readRangeSingle()/10;
  double northDistance = ultrasonicRange();
  uint8_t eastDistance = laser2.readRangeSingle()/10;
  double southDistance;

  delayMicroseconds(3*10^6); //delay 3 seconds for turning purposes
  IMUmeasurement();
  float finYaw = ypr[0] * 180/M_PI; //final yaw in degrees
  float rotation = finYaw - initYaw;

  if (rotation >= 80 && rotation <= 100) { //rotated approx 90 deg
    southDistance = (double) laser2.readRangeContinuousMillimeters()/10;
  } else if (rotation >= 170 && rotation <= 190) { //rotated approx 180 deg
    southDistance = ultrasonicRange();
  } else if (rotation >= 260 && rotation <= 280) { //rotated approx 270 deg
    southDistance = (double) laser1.readRangeContinuousMillimeters()/10;
  }

  if (northDistance > 7) {
    Serial.print("0 ");
  } else {
    Serial.print("1 ");
  }
  if (eastDistance > 7) {
    Serial.print("0 ");
  } else {
    Serial.print("1 ");
  }
  if (southDistance > 7) {
    Serial.print("0 ");
  } else {
    Serial.print("1 ");
  }
  if (westDistance > 7) {
    Serial.println("0 ");
  } else {
    Serial.println("1 ");
  }
}


double ultrasonicRange() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  long duration = pulseIn(echoPin, HIGH);
  double cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  return cm;
}


void IMUmeasurement() {
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = IMU.getFIFOCount();
    }      
  }
  mpuInterrupt = false; //reset interrupt flag and get INT_STATUS byte
  mpuIntStatus = IMU.getIntStatus();
  fifoCount = IMU.getFIFOCount();
  
  if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = IMU.getFIFOCount(); // wait for correct available data length, should be a VERY short wait

    IMU.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    IMU.dmpGetQuaternion(&q, fifoBuffer);
    IMU.dmpGetGravity(&gravity, &q);
    IMU.dmpGetYawPitchRoll(ypr, &q, &gravity);
    IMU.dmpGetAccel(&aa, fifoBuffer);
    IMU.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  }
}

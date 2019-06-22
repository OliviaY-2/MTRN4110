#include <external_MPU6050_6Axis_MotionApps20.h>
#include <external_MPU6050.h>
#include <external_MPU6050_helper_3dmath.h>
#include <external_MPU6050_I2Cdev.h>
#include <Adafruit_VL6180X.h>
#include <Wire.h>

#define INTERRUPT 2

Adafruit_VL6180X laser1 = Adafruit_VL6180X(); //create object using I2C
Adafruit_VL6180X laser2 = Adafruit_VL6180X(); //create object using I2C
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
int trigPin = 11;
int echoPin = 12;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(INTERRUPT, INPUT);
  laser1.begin();
  laser2.begin();

  Serial.println("Initialising IMU");
  IMU.initialize(); //set up IMU
  Serial.println("initialising DMP");
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
    Serial.print("Roll: ");
    Serial.println(ypr[2] * 180/M_PI);
    Serial.print("Pitch: ");
    Serial.println(ypr[1] * 180/M_PI);
    Serial.print("Yaw: ");
    Serial.println(ypr[2] * 180/M_PI);
    Serial.print("X acceleration: ");
    Serial.println(aaReal.x);
    Serial.print("Y acceleration: ");
    Serial.println(aaReal.y);
    Serial.print("Z acceleration: ");
    Serial.println(aaReal.z);
  }

  //ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  long duration = pulseIn(echoPin, HIGH);
  double cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  Serial.print("Distance (ultrasonic): ");
  Serial.print(cm);
  Serial.println(" cm");

  //LiDARs
  uint8_t range2 = laser1.readRange();
  uint8_t range2 = laser2.readRange();
  
  Serial.print("Range1 (laser): ");
  Serial.print(range1);
  Serial.println("mm");
  Serial.print("Range2 (laser): ");
  Serial.print(range2);
  Serial.println("mm");
}



void function2 () {
  Serial.println("Entered 2");
}



void function3 () {
  Serial.println("Entered 3");
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  long duration = pulseIn(echoPin, HIGH);
  double frontDistance = (duration/2) / 291;
  uint8_t leftDistance = laser1.readRange()/10;
  uint8_t rightDistance = laser2.readRange()/10;

  if (leftDistance > 7) {
    Serial.print("0 ");
  } else {
    Serial.print("1 ")
  }

  if (frontDistance > 7) {
    Serial.print("0 ");
  } else {
    Serial.print("1 ")
  }

  if (rightDistance > 7) {
    Serial.println("0");
  } else {
    Serial.println("1")
  }
  
}



void function4 () {
  Serial.println("Entered 4");
}

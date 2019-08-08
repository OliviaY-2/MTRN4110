//IMU ORIENTATION
//Y axis forward
//Z axis up

#include <external_VL6180X.h>
#include <external_MPU6050_6Axis_MotionApps20.h>
#include <external_MPU6050.h>
#include <external_MPU6050_helper_3dmath.h>
#include <external_MPU6050_I2Cdev.h>
#include <Wire.h>

#define CellSize 23.
#define INTERRUPT 27
//^Only possible on Mega

bool newData = false;
long receivedNum;
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

int E1 = 5; //speed control for motor 1 (left motor)
int M1 = 4; //direction control for motor 1
int E2 = 6; //speed control for motor 2 (right motor)
int M2 = 7; //direction control for motor 2
int _Speed1 = 0; //speed for motor 1
int _Speed2 = 0;

int trigPin = 30; //use 30 on mega
int echoPin = 31; //use 31 on mega

int laser1Pin = 32; //use 32 on mega
int laser2Pin = 33; //use 33 on mega
int address1 = 0x30;
int address2 = 0x32;

const byte encoder1pinA = 2;//A pin
const byte encoder1pinB = 8;//B pin
byte encoder1PinALast;
int duration1;//the number of the pulses
boolean Direction1;//the rotation direction
const byte encoder2pinA = 3;//A pin
const byte encoder2pinB = 9;//B pin
byte encoder2PinALast;
int duration2;//the number of the pulses
boolean Direction2;//the rotation direction

float initHeading;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  Serial1.begin(9600);
  Wire.begin();
  Serial1.println("Initialising ultrasonic");
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial1.println("Initialising motors");
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(INTERRUPT, INPUT);

  pinMode(laser1Pin, OUTPUT);
  pinMode(laser2Pin, OUTPUT);
  digitalWrite(laser1Pin, LOW);
  digitalWrite(laser2Pin, LOW);
  delay(1000);

  Serial1.println("Initialising LiDAR 1");
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
  Serial1.println("LiDAR 1 initialised");

  Serial1.println("Initialising LiDAR 2");
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
  Serial1.println("LiDAR 2 initialised");

  Serial1.println("Initialising encoders");
  Encoder1Init();
  Encoder2Init();

  Serial1.println("Initialising IMU");
  IMU.initialize(); //set up IMU
  Serial1.println("initialising DMP");
  devStatus = IMU.dmpInitialize();


  if (devStatus == 0) {
    Serial1.println("DMP initialisation successful");
    IMU.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT), dmpDataReady, RISING);  //set up interrupt
    mpuIntStatus = IMU.getIntStatus();
    dmpReady = true;
    packetSize = IMU.dmpGetFIFOPacketSize();
  } else {
    Serial1.println("DMP initialisation failed");
  }

  adjustInitialHeading();
  Serial1.println("Please enter a number 1 - 4");
}

void loop() {
  // put your main code here, to run repeatedly:

}


void forward1cell(){
  IMUmeasurement();
  initHeading = ypr[0]*180/M_PI;
  double currHeading = initHeading;
  double distanceTravelled = 0;
  double angle = 0;
  int currTime = 0;
  int lastTime = 0;
  digitalWrite(M1, HIGH); //set M1 to forward
  digitalWrite(M2, LOW); //set M2 to forward
  analogWrite(E1, 255);
  analogWrite(E2, 255);
  currTime = millis();
  delay(50);

  while (distanceTravelled < CellSize) {
    lastTime = currTime;
    IMUmeasurement();
    currHeading = ypr[0]*180/M_PI; 
    angle = currHeading - initHeading;//assuming negative is to the left
    currTime = millis();
    distanceTravelled = distanceTravelled + 0.5*aa.y*pow((currTime-lastTime)/1000,2);
    if (angle < 0){ //bear right
      _Speed1 = 255;
      _Speed2 = 255 - angle*10;
    } else { //bear left
      _Speed1 = 255 - angle*10;
      _Speed2 = 255;
    }
    analogWrite(E1, _Speed1);
    analogWrite(E2, _Speed2);
  }
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}


void back1cell(){
  IMUmeasurement();
  initHeading = ypr[0]*180/M_PI;
  double currHeading = initHeading;
  double distanceTravelled = 0;
  double angle = 0;
  int currTime = 0;
  int lastTime = 0;
  digitalWrite(M1, LOW); //set M1 to backward
  digitalWrite(M2, HIGH); //set M2 to backward
  analogWrite(E1, 255);
  analogWrite(E2, 255);
  currTime = millis();
  delay(50);

  while (distanceTravelled < CellSize) {
    lastTime = currTime;
    IMUmeasurement();
    currHeading = ypr[0]*180/M_PI; 
    angle = currHeading - initHeading;//assuming negative is to the left
    currTime = millis();
    distanceTravelled = distanceTravelled + 0.5*aa.y*pow((currTime-lastTime)/1000,2);
    if (angle < 0){ //bear right
      _Speed1 = 255;
      _Speed2 = 255 - angle*10;
    } else { //bear left
      _Speed1 = 255 - angle*10;
      _Speed2 = 255;
    }
    analogWrite(E1, _Speed1);
    analogWrite(E2, _Speed2);
  }
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}


void forwardXcell(int X){
  IMUmeasurement();
  initHeading = ypr[0]*180/M_PI;
  double currHeading = initHeading;
  double distanceTravelled = 0;
  double angle = 0;
  int currTime = 0;
  int lastTime = 0;
  digitalWrite(M1, HIGH); //set M1 to forward
  digitalWrite(M2, LOW); //set M2 to forward
  analogWrite(E1, 255);
  analogWrite(E2, 255);
  currTime = millis();
  delay(50);

  while (distanceTravelled < X*CellSize) {
    lastTime = currTime;
    IMUmeasurement();
    currHeading = ypr[0]*180/M_PI; 
    angle = currHeading - initHeading;//assuming negative is to the left
    currTime = millis();
    distanceTravelled = distanceTravelled + 0.5*aa.y*pow((currTime-lastTime)/1000,2);
    if (angle < 0){ //bear right
      _Speed1 = 255;
      _Speed2 = 255 - angle*10;
    } else { //bear left
      _Speed1 = 255 - angle*10;
      _Speed2 = 255;
    }
    analogWrite(E1, _Speed1);
    analogWrite(E2, _Speed2);
  }
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}


void right90deg() {
  IMUmeasurement();
  initHeading = ypr[0] *180/M_PI;
  double currHeading = ypr[0] *180/M_PI;
  digitalWrite(M1, HIGH); //set M1 (left motor) to Backward
  digitalWrite(M2, HIGH); //set M2 (right motor) to forward
  analogWrite(E1, 255); //M1 drives at _Speed
  analogWrite(E2, 255); //M2 drives at _Speed

  while (abs(currHeading - initHeading) < 90){
    IMUmeasurement();
    currHeading = ypr[0] *180/M_PI;
  }
  analogWrite(E1, 0); //M1 drives at _Speed
  analogWrite(E2, 0); //M2 drives at _Speed
}


void rightXdeg(int X) {
  IMUmeasurement();
  initHeading = ypr[0] *180/M_PI;
  double currHeading = ypr[0] *180/M_PI;
  digitalWrite(M1, HIGH); //set M1 (left motor) to Backward
  digitalWrite(M2, HIGH); //set M2 (right motor) to forward
  analogWrite(E1, 255); //M1 drives at _Speed
  analogWrite(E2, 255); //M2 drives at _Speed

  while (abs(currHeading - initHeading) < X){
    IMUmeasurement();
    currHeading = ypr[0] *180/M_PI;
  }
  analogWrite(E1, 0); //M1 drives at _Speed
  analogWrite(E2, 0); //M2 drives at _Speed
}


void left90deg() {
  IMUmeasurement();
  initHeading = ypr[0] *180/M_PI;
  double currHeading = ypr[0] *180/M_PI;
  digitalWrite(M1, LOW); //set M1 (left motor) to Backward
  digitalWrite(M2, LOW); //set M2 (right motor) to forward
  analogWrite(E1, 255); //M1 drives at _Speed
  analogWrite(E2, 255); //M2 drives at _Speed

  while (abs(currHeading - initHeading) < 90){
    IMUmeasurement();
    currHeading = ypr[0] *180/M_PI;
  }
  analogWrite(E1, 0); //M1 drives at _Speed
  analogWrite(E2, 0); //M2 drives at _Speed
}


void leftXdeg() {
  IMUmeasurement();
  initHeading = ypr[0] *180/M_PI;
  double currHeading = ypr[0] *180/M_PI;
  digitalWrite(M1, LOW); //set M1 (left motor) to Backward
  digitalWrite(M2, LOW); //set M2 (right motor) to forward
  analogWrite(E1, 255); //M1 drives at _Speed
  analogWrite(E2, 255); //M2 drives at _Speed

  while (abs(currHeading - initHeading) < X){
    IMUmeasurement();
    currHeading = ypr[0] *180/M_PI;
  }
  analogWrite(E1, 0); //M1 drives at _Speed
  analogWrite(E2, 0); //M2 drives at _Speed
}


void right180deg() {
  IMUmeasurement();
  initHeading = ypr[0] *180/M_PI;
  double currHeading = ypr[0] *180/M_PI;
  digitalWrite(M1, HIGH); //set M1 (left motor) to Backward
  digitalWrite(M2, HIGH); //set M2 (right motor) to forward
  analogWrite(E1, 255); //M1 drives at _Speed
  analogWrite(E2, 255); //M2 drives at _Speed

  while (abs(currHeading - initHeading) < 180){
    IMUmeasurement();
    currHeading = ypr[0] *180/M_PI;
  }
  analogWrite(E1, 0); //M1 drives at _Speed
  analogWrite(E2, 0); //M2 drives at _Speed
}


void adjustInitialHeading(){
  double lastLeftDistance = 25.0;
  double lastRightDistance = 25.0;
  double distanceFromWall = ultrasonicRange();
  double currentLeftDistance = laser1.readRangeContinuousMillimeters();
  double currentRightDistance = laser2.readRangeContinuousMillimeters();

  digitalWrite(M1, HIGH); //set M1 (left motor) to Backward
  digitalWrite(M2, HIGH); //set M2 (right motor) to forward

  while ( lastLeftDistance > currentLeftDistance || lastRightDistance > currentRightDistance || distanceFromWall < 20.0) { //while values of left and right distances are shrinking and front obstruction within 20cm
    lastLeftDistance = currentLeftDistance;
    lastRightDistance = currentRightDistance;
    analogWrite(E1, 100);
    analogWrite(E2, 100); //turn right slowly
    delay(50);
    distanceFromWall = ultrasonicRange();
    currentLeftDistance = laser1.readRangeContinuousMillimeters();
    currentRightDistance = laser2.readRangeContinuousMillimeters();
  }
  analogWrite(E1, 0);
  analogWrite(E2, 0);//stop
  IMUmeasurement();
  initHeading = ypr[0] * 180/M_PI;
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

void Encoder1Init()
{
  Direction1 = true;//default -> Forward
  pinMode(encoder1pinB,INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1pinA), wheelSpeed1, CHANGE);
}

void wheelSpeed1()
{
  if (digitalRead(encoder1pinA) == digitalRead(encoder1pinB)){
    duration1 = duration1 + 1;
  } else {
    duration1 = duration1 - 1;
  }
}

void Encoder2Init()
{
  Direction2 = true;//default -> Forward
  pinMode(encoder2pinB,INPUT);
  attachInterrupt(encoder2pinA, wheelSpeed1, CHANGE);
}

void wheelSpeed2()
{
    if (digitalRead(encoder2pinA) == digitalRead(encoder2pinB)){
    duration2 = duration2 + 1;
  } else {
    duration2 = duration2 - 1;
  }
}

//Code to receive planned path via serial as a string and enact it

///////////////////////////////////////////////
// Driving V2.0...
///////////////////////////////////////////////

#define CellSize 23
#define STOPPING_DIST 5

void forward1cell ();
void forwardXcell (int X);
void back1cell ();
void right90deg ();
void rightXdeg ();
void right180deg ();
void left90deg ();
void leftXdeg (int X);
void adjustInitialHeading ();

///////////////////////////////////////////////
// IMU V1.00
///////////////////////////////////////////////

#include <external_MPU6050_6Axis_MotionApps20.h>
#include <external_MPU6050.h>
#include <external_MPU6050_helper_3dmath.h>
#include <external_MPU6050_I2Cdev.h>
#include <Wire.h>

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
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float globalHeading; 

///////////////////////////////////////////////
// LED V1.01
///////////////////////////////////////////////

// PIN numbers for each LED
#define RED_LED 36
#define GREEN_LED 37

// 0 = both lights off
// 1 = ledState :  Green light on,  Red light off
// 2 = ledState :  Green light off, Red light on  
// 3 = ledState :  Green light on,  Red light on 
int LED_State = 0; 

void initLEDS();
bool isLED_on(int colourLED);
void updateLED(int ledState);
void setLED(int colourLED, int ledState);

///////////////////////////////////////////////
// Laser V2.02
///////////////////////////////////////////////

#include <external_VL6180X.h>

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
// Ultrasonic V2.00
///////////////////////////////////////////////

#include <external_VL6180X.h>

#define TRIG_PIN 30 
#define ECHO_PIN 31

void initUltrasonic();
double ultrasonicRange();
void testUltrasonic();

///////////////////////////////////////////////
// Motor V1.00
///////////////////////////////////////////////

#define ENCODER1PIN_A 10
#define ENCODER1PIN_B  8
#define MOTOR1DIRECTION_PIN 4
#define MOTOR1SPEED_PIN 5

#define ENCODER2PIN_A 3
#define ENCODER2PIN_B  9
#define MOTOR2DIRECTION_PIN 7
#define MOTOR2SPEED_PIN 6

int _Speed1 = 0; //speed for motor 1
int _Speed2 = 0;

int duration1;//the number of the pulses
int duration2;//the number of the pulses

void Encoder1Init();
void Encoder2Init();

void wheelSpeed1();
void wheelSpeed2();

int getDirectionPin(int motor);
void setMotorForward(int motor);
void setMotorReverse(int motor);
void setMotorSpeed(int motor, int _speed);

////////////////////////////////////////
// PathToDrive V1.00
///////////////////////////////////////

char receivedNum = 'A';
boolean newData = false;

void recvNum ();
void decisionTree ();

///////////////////////////////////////



void setup() {
  Serial.begin(9600);
  pathFromSerial();
}

void loop() {

}


void recvNum() {
  if (Serial.available() > 0) {
    receivedNum = Serial.read();
    newData = true;
   }
}


void decisionTree() {
  if (newData == true) {
    switch (receivedNum){
      case '1':
        forward1cell();
        break;
      case '2':
        left90deg();
        break;
      case '3':
        right90deg();
        break;
      case '0':
        Serial.println("End of path!");
        updateLED(2);
        break;
      default:
        break;
    }
    newData = false;
  }
}


void pathFromSerial() {
  receivedNum = 'A';
  while (receivedNum != '0'){
    recvNum();
    decisionTree();
  }
}

void forward1cell(){
  Serial.println("forward");
}


void left90deg() {
  Serial.println("left");
}

void right90deg() {
  Serial.println("right");
}

// This is the code to be uploaded to the arduino for the vision run

////////////////////////////////////////
// Path Planning
///////////////////////////////////////
int map1[5][9] = { {8,0,3,3,4,8,8,8,7},
                   {5,4,2,4,6,6,8,8,7},
                   {6,3,7,1,6,6,8,8,7},
                   {8,3,1,1,6,6,6,8,7},
                   {3,3,3,3,3,4,5,5,4} };

// Initialise positions array
// 100 rows should suffice
int positions[300][5];

// Arrays for best solutions and interim solution for calculating best soln
int bestSolutionUncertain[30][5];
int bestSolutionCertain[30][5];
int currentSolution[30][5];

// String storing forward rotate etc instructions
String instructions = "";

// Some variables to store the information that will be used to confirm that
// the map has been fully explored
int uncertainSteps = 0;
int uncertainTurns = 0;
int certainSteps = 0;
int certainTurns = 0;

// initialise other globals
int id = 0;
int iterations = 0;

// Initialise a variable to make the if statements in nextMove much easier to write
int CIQ = 0;
int newXpos;
int newYpos;

// Define the target location and the starting location and starting position
int targetCell[2] = {2,4};
int startCell[2] = {4,4};
int startDir = 3;

// Initialise a variable to break the main loop
bool reachedTarget = false;

// variable for BT input, initialised to something random
char state = 'A'; 
char BTstate = 'N';
String input = "";
boolean NL = true;


void nextMove(int pos[5], int currentMap[5][9], bool includeUncertain);
bool discardMove(int newY, int newX, int numSteps);
void planPath(bool includeUncertain);
void extractSolution(bool includeUncertain);
void printSolution(bool includeUncertain);
void printMap(bool includeUncertain);
void decodeInput(String inputString);
void decodeMap(String inputString);

////////////////////////////////////////
// PathToDrive V1.00
///////////////////////////////////////

void pathFromString(String Instructions);
void decisionTree();

///////////////////////////////////////////////
// LED V1.01 changed
///////////////////////////////////////////////

// PIN numbers for each LED
#define RED_LED 36
#define GREEN_LED 37

// 0 = Green light off, Red light off
// 1 = Green light on,  Red light off
// 2 = Green light off, Red light on  
// 3 = Green light on,  Red light on 
int LED_State = 0; 

void initLEDS();
bool isLED_on(int colourLED);
void updateLED(int ledState);
void setLED(int colourLED, int ledState);

///////////////////////////////////////////////

///////////////////////////////////////////////
// Ultrasonic V2.01
///////////////////////////////////////////////

#define TRIG_PIN 30 
#define ECHO_PIN 31

void initUltrasonic();
double ultrasonicRange();
void testUltrasonic();

///////////////////////////////////////////////

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

///////////////////////////////////////////////
// Motor V1.00
///////////////////////////////////////////////

#define ENCODER1PIN_A 10
#define ENCODER1PIN_B  8
#define MOTOR1DIRECTION_PIN 4
#define MOTOR1SPEED_PIN 5

#define ENCODER2PIN_A 10
#define ENCODER2PIN_B  8
#define MOTOR2DIRECTION_PIN 4
#define MOTOR2SPEED_PIN 5

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

///////////////////////////////////////////////

///////////////////////////////////////////////
// Robot V1.02 changed
///////////////////////////////////////////////

// currentPos 
// Current position of the robot, (x,y, heading(deg)) +0 in X direction
int currentPos[3] = {1,8,0}; 

// nextPos
// Next position of the robot, (x,y, heading(deg))
int nextPos[3] = {0,0,0}; 

// Status of left or right wall
// 0 = unknown, 1 = wallSeen, 2 = noWall 

int leftWall  = 0; 
int rightWall = 0; 

// Status of the front wall 
// 0 = unknown, 1 = wall directly in front, 2 = wall in second square in front,
// 3 = wall in third square in front,       4 = wall in fourth square in front
int frontWall = 0; 


void calibrateRobot();
void askNextHeading();
void askNextX();
void askNextY();
void askNextPosition();
void askRobotFrontWall();
void askRobotLeftWall();
void askRobotRightWall();
void askUserLeftWall();
void askUserRightWall();
void askUserFrontWall();
void displayPos(int pos[3]);
int getDirection(float heading);
int getHeading();
int getXPos();
int getYPos();
void moveNextPosition();
void setCurrentPos(int x, int y, int heading);
void turnToHeading(double targetHeading);
void turnAngle(double angle);
void forwardNumCells(int numCells);

///////////////////////////////////////////////

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

void setup() {
  Serial1.begin(9600);
  initLEDS();
  initUltrasonic();
  initIMU();
  calibrateRobot();
  initLaser(laser1, LASER1PIN, LASER1ADDRESS);
  initLaser(laser2, LASER2PIN, LASER2ADDRESS);
  Encoder1Init();
  Encoder2Init();
  analogWrite(MOTOR1SPEED_PIN, 0); 
  analogWrite(MOTOR2SPEED_PIN, 0);
  updateLED(0);
}

void loop() 
{
  // Read from the Bluetooth module and send to the Arduino Serial Monitor
  if (Serial1.available())
    {
        BTstate = Serial1.read();
        input += BTstate;
        if (BTstate == '\n') // Single quotes are crucial here
        {

          decodeInput(input);

          //Use planned path to drive to the center
          updateLED(1);
          pathFromString(instructions);
          
          input = "";
          Serial1.write('o');
          Serial1.write('k');
          Serial1.write('\r');
          Serial1.write('\n');
        }
    }

}

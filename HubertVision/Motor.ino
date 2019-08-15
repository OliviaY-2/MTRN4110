/*
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

int MOTOR1SPEED_PIN = 5; //speed control for motor 1 (left motor)
int MOTOR1DIRECTION_PIN = 4; //direction control for motor 1
int MOTOR2SPEED_PIN = 6; //speed control for motor 2 (right motor)
int MOTOR2DIRECTION_PIN = 7; //direction control for motor 2
int _Speed1 = 0; //speed for motor 1
int _Speed2 = 0;


const byte ENCODER1PIN_A = 10;//A pin
const byte ENCODER1PIN_B = 8;//B pin
byte ENCODER1PIN_ALast;
int duration1;//the number of the pulses
boolean Direction1;//the rotation direction
const byte ENCODER2PIN_A = 3;//A pin
const byte ENCODER2PIN_B = 9;//B pin
byte ENCODER2PIN_ALast;
int duration2;//the number of the pulses
boolean Direction2;//the rotation direction

void Encoder1Init();
void Encoder2Init();

void wheelSpeed1();
void wheelSpeed2();

int getDirectionPin(int motor);
void setMotorForward(int motor);
void setMotorReverse(int motor);
void setMotorSpeed(int motor, int _speed);

///////////////////////////////////////////////
*/


void Encoder1Init()
{
  pinMode(ENCODER1PIN_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER1PIN_A), wheelSpeed1, CHANGE);
}

void wheelSpeed1()
{
  if (digitalRead(ENCODER1PIN_A) == digitalRead(ENCODER1PIN_B)){
    duration1 = duration1 + 1;
  } else {
    duration1 = duration1 - 1;
  }
}

void Encoder2Init()
{
  pinMode(ENCODER2PIN_B,INPUT);
  attachInterrupt(ENCODER2PIN_A, wheelSpeed1, CHANGE);
}

void wheelSpeed2()
{
    if (digitalRead(ENCODER2PIN_A) == digitalRead(ENCODER2PIN_B)){
    duration2 = duration2 + 1;
  } else {
    duration2 = duration2 - 1;
  }
}

int getDirectionPin(int motor) { // here to allow improved interations with the motor later
  return motor;
}

int getSpeedPin(int motor) { // here to allow improved interations with the motor later
  return motor;
}

void setMotorForward(int motor) { digitalWrite(getDirectionPin(motor), HIGH); }

void setMotorReverse(int motor) { digitalWrite(getDirectionPin(motor), LOW); }

void setMotorSpeed(int motor, int _speed) { analogWrite(getSpeedPin(motor), _speed); }

//

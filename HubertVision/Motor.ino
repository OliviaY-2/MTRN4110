/*
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
  int directionPin;
  if (motor == 1) {
    directionPin = MOTOR1DIRECTION_PIN;
  } else {
    directionPin = MOTOR2DIRECTION_PIN;
  }
  return directionPin;
}

int getSpeedPin(int motor) { // here to allow improved interations with the motor later
  int speedPin;
  if (motor == 1) {
    speedPin = MOTOR1SPEED_PIN;
  } else {
    speedPin = MOTOR2SPEED_PIN; 
  }
  return speedPin;
}

void setMotorForward(int motor) { digitalWrite(getDirectionPin(motor), HIGH); }

void setMotorReverse(int motor) { digitalWrite(getDirectionPin(motor), LOW); }

void setMotorSpeed(int motor, int _speed) { analogWrite(getSpeedPin(motor), _speed); }

//

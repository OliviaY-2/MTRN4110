#include <external_VL6180X.h>

#define CellSize 23.5
#define Pi 3.14159
#define rightTurnDuration 360
//^number of pulses in a 90 deg turn.

bool newData = false;
long receivedNum;
VL6180X laser1;
VL6180X laser2;
int E1 = 5; //speed control for motor 1 (left motor)
int M1 = 4; //direction control for motor 1
int E2 = 6; //speed control for motor 2 (right motor)
int M2 = 7; //direction control for motor 2
int _Speed = 0; //speed for motors
int trigPin = 11; //use 30/1 on mega
int echoPin = 12; //use 30/1 on mega
int laser1Pin = 13; //use 28 on mega
int laser2Pin = 10; //use 29 on mega
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

void setup() {
  Serial.begin(9600);
  Serial.println("Initialising ultrasonic");
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("Initialising motors");
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

//  Serial.println("Initialising LiDAR 1");
//  digitalWrite(laser1Pin, HIGH);
//  delay(50);
//  laser1.init();
//  laser1.configureDefault();
//  laser1.setAddress(address1);
//  laser1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
//  laser1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
//  laser1.setTimeout(500);
//  laser1.stopContinuous();
//  laser1.setScaling(1);
//  delay(300);
//  laser1.startInterleavedContinuous(100);
//  delay(100);
//  Serial.println("LiDAR 1 initialised");

//  Serial.println("Initialising LiDAR 2");
//  digitalWrite(laser2Pin, HIGH);
//  delay(50);
//  laser2.init();
//  laser2.configureDefault();
//  laser2.setAddress(address2);
//  laser2.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
//  laser2.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
//  laser2.setTimeout(500);
//  laser2.stopContinuous();
//  laser2.setScaling(1);
//  delay(300);
//  laser2.startInterleavedContinuous(100);
//  delay(100);
//  Serial.println("LiDAR 2 initialised");

  Encoder1Init();
  Encoder2Init();
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
    newData = false;
    switch (receivedNum){
      case 1:
        forward1cell();
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
  }
}

void forward1cell() {
  double distanceTravelled = 0;
  duration1 = 0;
  _Speed = 225;
  digitalWrite(M1, HIGH); //set M1 to forward
  digitalWrite(M2, LOW); //set M2 to forward
  analogWrite(E1, _Speed); //M1 drives at _Speed
  analogWrite(E2, _Speed); //M2 drives at _Speed
  _Speed = 0;

  while (distanceTravelled < CellSize){ //repeatedly measure the distance travelled until it is cell size
    distanceTravelled = duration1/48.00; //48 pulses to 1 cm
    Serial.println(distanceTravelled);
  }
  analogWrite(E1, _Speed); //M1 stops
  analogWrite(E2, _Speed); //M2 stops
}

void function2() {
  //READ 90 OR -90 IN TO INPUT
  Serial.println("Enter 90 to turn left or -90 to turn right");
  int input;
  while (newData == false){ //wait until intstruction is inputted
    if (Serial.available()) {
      input = Serial.parseInt();
      newData = true;
    }
    
  }
  if (input == 90){ //turn left
    turnLeft();
  } else if (input == -90) { //turn right
    turnRight();
  }
}

void function3() {
  double distanceFromWall = ultrasonicRange();
  _Speed = 255;
  digitalWrite(M1, HIGH); //set M1 to forward
  digitalWrite(M2, LOW); //set M2 to forward
  analogWrite(E1, _Speed); //M1 drives at _Speed
  analogWrite(E2, _Speed); //M2 drives at _Speed
  _Speed = 0;

  while (distanceFromWall > (CellSize/4)){ //repeatedly measure the distance until it is 1/4 of cell size
    distanceFromWall = ultrasonicRange();
  }
  analogWrite(E1, _Speed); //M1 stops
  analogWrite(E2, _Speed); //M2 stops
}

void function4() {
  double leftDistance = 25.0;
  double rightDistance = 25.0;

  while (leftDistance > 20 || rightDistance > 20) { // while there are turns to make
    forward1cell(); //drive forward 1
    leftDistance = laser1.readRangeContinuousMillimeters()/10; //measure distances to the right and left
    rightDistance = laser2.readRangeContinuousMillimeters()/10;
    if (leftDistance > 20) { //turn
      turnLeft();
    } else if (rightDistance > 20) {
      turnRight();
    }
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


void turnRight(){
  digitalWrite(M1, HIGH); //set M1 (left motor) to Backward
  digitalWrite(M2, HIGH); //set M2 (right motor) to forward

  _Speed = 255;
  duration1 = 0;
  analogWrite(E1, _Speed); //M1 drives at _Speed
  analogWrite(E2, _Speed); //M2 drives at _Speed
  _Speed = 0;
    
  while (abs(duration1) < rightTurnDuration) {
    Serial.println("do nothing");
  }
  analogWrite(E1, _Speed); //M1 drives at _Speed
  analogWrite(E2, _Speed); //M2 drives at _Speed
}


void turnLeft(){
  digitalWrite(M1, LOW); //set M1 (left motor) to forward
  digitalWrite(M2, LOW); //set M2 (right motor) to backward
  
  _Speed = 255;
  duration1 = 0;
  analogWrite(E1, _Speed); //M1 drives at _Speed
  analogWrite(E2, _Speed); //M2 drives at _Speed
  _Speed = 0;
    
  while (abs(duration1) < rightTurnDuration) {
    Serial.println(duration1);
  }
  analogWrite(E1, _Speed); //M1 drives at _Speed
  analogWrite(E2, _Speed); //M2 drives at _Speed
}

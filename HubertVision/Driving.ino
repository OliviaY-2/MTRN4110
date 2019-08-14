/*
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

//////////////////////////////////////////////
*/


void back1cell(){
  IMUmeasurement();
  double initHeading = ypr[0]*180/M_PI;
  double currHeading = initHeading;
  double distanceTravelled = 0;
  double angle = 0;
  int currTime = 0;
  int lastTime = 0;
  digitalWrite(MOTOR1DIRECTION_PIN, LOW); //set MOTOR1DIRECTION_PIN to backward
  digitalWrite(MOTOR2DIRECTION_PIN, HIGH); //set MOTOR2DIRECTION_PIN to backward
  analogWrite(MOTOR1SPEED_PIN, 255);
  analogWrite(MOTOR2SPEED_PIN, 255);
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
    analogWrite(MOTOR1SPEED_PIN, _Speed1);
    analogWrite(MOTOR2SPEED_PIN, _Speed2);
  }
  analogWrite(MOTOR1SPEED_PIN, 0);
  analogWrite(MOTOR2SPEED_PIN, 0);
}


void forwardXcell(int X){
  IMUmeasurement();
  double initHeading = ypr[0]*180/M_PI;
  double currHeading = initHeading;
  double distanceTravelled = 0;
  double angle = 0;
  int currTime = 0;
  int lastTime = 0;
  digitalWrite(MOTOR1DIRECTION_PIN, HIGH); //set MOTOR1DIRECTION_PIN to forward
  digitalWrite(MOTOR2DIRECTION_PIN, LOW); //set MOTOR2DIRECTION_PIN to forward
  analogWrite(MOTOR1SPEED_PIN, 255);
  analogWrite(MOTOR2SPEED_PIN, 255);
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
    analogWrite(MOTOR1SPEED_PIN, _Speed1);
    analogWrite(MOTOR2SPEED_PIN, _Speed2);
  }
  analogWrite(MOTOR1SPEED_PIN, 0);
  analogWrite(MOTOR2SPEED_PIN, 0);
}


void right90deg() {
  IMUmeasurement();
  double initHeading = ypr[0] *180/M_PI;
  double currHeading = initHeading;
  digitalWrite(MOTOR1DIRECTION_PIN, HIGH); //set MOTOR1DIRECTION_PIN (left motor) to Backward
  digitalWrite(MOTOR2DIRECTION_PIN, HIGH); //set MOTOR2DIRECTION_PIN (right motor) to forward
  analogWrite(MOTOR1SPEED_PIN, 180); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 180); //MOTOR2DIRECTION_PIN drives at _Speed

  while (abs(currHeading - initHeading) < 70){
    IMUmeasurement();
    currHeading = ypr[0] *180/M_PI;
  }
  analogWrite(MOTOR1SPEED_PIN, 0); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 0); //MOTOR2DIRECTION_PIN drives at _Speed
}


void rightXdeg(int X) {
  IMUmeasurement();
 double  initHeading = ypr[0] *180/M_PI;
  double currHeading = ypr[0] *180/M_PI;
  digitalWrite(MOTOR1DIRECTION_PIN, HIGH); //set MOTOR1DIRECTION_PIN (left motor) to Backward
  digitalWrite(MOTOR2DIRECTION_PIN, HIGH); //set MOTOR2DIRECTION_PIN (right motor) to forward
  analogWrite(MOTOR1SPEED_PIN, 180); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 180); //MOTOR2DIRECTION_PIN drives at _Speed

  while (abs(currHeading - initHeading) < (X-20)){
    IMUmeasurement();
    currHeading = ypr[0] *180/M_PI;
  }
  analogWrite(MOTOR1SPEED_PIN, 0); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 0); //MOTOR2DIRECTION_PIN drives at _Speed
}


void left90deg() {
  IMUmeasurement();
  double initHeading = ypr[0] *180/M_PI;
  double currHeading = ypr[0] *180/M_PI;
  digitalWrite(MOTOR1DIRECTION_PIN, LOW); //set MOTOR1DIRECTION_PIN (left motor) to Backward
  digitalWrite(MOTOR2DIRECTION_PIN, LOW); //set MOTOR2DIRECTION_PIN (right motor) to forward
  analogWrite(MOTOR1SPEED_PIN, 180); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 180); //MOTOR2DIRECTION_PIN drives at _Speed
 

  while (abs(initHeading - currHeading) < 70){
    IMUmeasurement();
    currHeading = ypr[0] *180/M_PI;
  }
  analogWrite(MOTOR1SPEED_PIN, 0); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 0); //MOTOR2DIRECTION_PIN drives at _Speed
}


void leftXdeg(int X) {
  IMUmeasurement();
  double  initHeading = ypr[0] *180/M_PI;
  double currHeading = ypr[0] *180/M_PI;
  digitalWrite(MOTOR1DIRECTION_PIN, LOW); //set MOTOR1DIRECTION_PIN (left motor) to Backward
  digitalWrite(MOTOR2DIRECTION_PIN, LOW); //set MOTOR2DIRECTION_PIN (right motor) to forward
  analogWrite(MOTOR1SPEED_PIN, 180); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 180); //MOTOR2DIRECTION_PIN drives at _Speed

  while (abs(currHeading - initHeading) < (X-20)){
    IMUmeasurement();
    currHeading = ypr[0] *180/M_PI;
  }
  analogWrite(MOTOR1SPEED_PIN, 0); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 0); //MOTOR2DIRECTION_PIN drives at _Speed
}


void right180deg() {
  IMUmeasurement();
  double initHeading = ypr[0] *180/M_PI;
  double currHeading = ypr[0] *180/M_PI;
  digitalWrite(MOTOR1DIRECTION_PIN, HIGH); //set MOTOR1DIRECTION_PIN (left motor) to Backward
  digitalWrite(MOTOR2DIRECTION_PIN, HIGH); //set MOTOR2DIRECTION_PIN (right motor) to forward
  analogWrite(MOTOR1SPEED_PIN, 180); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 180); //MOTOR2DIRECTION_PIN drives at _Speed

  while (abs(currHeading - initHeading) < 160){
    IMUmeasurement();
    currHeading = ypr[0] *180/M_PI;
  }
  analogWrite(MOTOR1SPEED_PIN, 0); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 0); //MOTOR2DIRECTION_PIN drives at _Speed
}


void adjustInitialHeading(){
  double lastLeftDistance = 25.0;
  double lastRightDistance = 25.0;
  double currentLeftDistance = laser1.readRangeContinuousMillimeters();
  double currentRightDistance = laser2.readRangeContinuousMillimeters();

  digitalWrite(MOTOR1DIRECTION_PIN, HIGH); //set MOTOR1DIRECTION_PIN (left motor) to Backward
  digitalWrite(MOTOR2DIRECTION_PIN, HIGH); //set MOTOR2DIRECTION_PIN (right motor) to forward

  while ( lastLeftDistance > currentLeftDistance || lastRightDistance > currentRightDistance) { //while values of left and right distances are shrinking and front obstruction within 20cm
    lastLeftDistance = currentLeftDistance;
    lastRightDistance = currentRightDistance;
    analogWrite(MOTOR1SPEED_PIN, 50);
    analogWrite(MOTOR2SPEED_PIN, 50); //turn right slowly
    currentLeftDistance = laser1.readRangeContinuousMillimeters();
    currentRightDistance = laser2.readRangeContinuousMillimeters();
  }
  analogWrite(MOTOR1SPEED_PIN, 0);
  analogWrite(MOTOR2SPEED_PIN, 0);//stop
}


void forward1cell() {
  double distanceFromWall = ultrasonicRange();
  double initDistance = distanceFromWall;
  double leftDistance = laser1.readRangeContinuousMillimeters()/10;
  double rightDistance = laser2.readRangeContinuousMillimeters()/10;
  _Speed1 = 255;
  _Speed2 = 255;
  setMotorForward(1);
  setMotorReverse(2);
  analogWrite(MOTOR1SPEED_PIN, _Speed1); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, _Speed2); //MOTOR2DIRECTION_PIN drives at _Speed
  updateLED(3);  
  
  while ((initDistance - distanceFromWall) < (CellSize - 2)){
    
    updateLED(0);  
    distanceFromWall = ultrasonicRange();
    leftDistance = laser1.readRangeContinuousMillimeters()/10; //measure distances to the right and left
    rightDistance = laser2.readRangeContinuousMillimeters()/10;
    if (leftDistance < 8) { //if too close to left wall, veer right
      _Speed1 = 255;
      _Speed2 = 200;
    } else if (rightDistance < 8) { //if too close to right wall, veer left
      _Speed1 = 200;
      _Speed2 = 255;
    } else {
      _Speed1 = 255;
      _Speed2 = 255;
    }
    analogWrite(MOTOR1SPEED_PIN, _Speed1); //MOTOR1DIRECTION_PIN drives at _Speed
    analogWrite(MOTOR2SPEED_PIN, _Speed2); //MOTOR2DIRECTION_PIN drives at _Speed
  }
  
  analogWrite(MOTOR1SPEED_PIN, 0); //MOTOR1DIRECTION_PIN stops
  analogWrite(MOTOR2SPEED_PIN, 0); //MOTOR2DIRECTION_PIN stops
  
  updateLED(3);  
}

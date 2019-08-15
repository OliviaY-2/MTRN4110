/*
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

///////////////////////////////////////////////
*/

#define DEBUG_ROBOT false

#define CELL_SIZE 23.0


float zeroAngleZ;
               

void calibrateRobot() {
  zeroAngleZ = readAngleZ();
}


void askNextHeading() {
  char junk = ' '; // character to hold the random values in a string after a key is pressed

  Serial.println("Enter value for the Next Heading, Press ENTER");
  while (Serial.available() == 0) ;  // Wait here until input buffer has a character
  {
    nextPos[2] = Serial.parseInt();      
    Serial.print("Heading = "); Serial.println(nextPos[2], DEC);

    while (Serial.available() > 0)  // .parseInt() can leave non-numeric characters
    { junk = Serial.read() ; }      // clear the keyboard buffer
  }
}


void askNextX()
{ 
    char junk = ' '; // character to hold the random values in a string after a key is pressed


  Serial.println("Enter value for the Next X, Press ENTER");
  while (Serial.available() == 0) ;  // Wait here until input buffer has a character
  {
    nextPos[0] = Serial.parseInt();      
    Serial.print("X = "); Serial.println(nextPos[0], DEC);

    while (Serial.available() > 0)  // .parseInt() can leave non-numeric characters
    { junk = Serial.read() ; }      // clear the keyboard buffer
  }
}


void askNextY()
{ 
    char junk = ' '; // character to hold the random values in a string after a key is pressed


  Serial.println("Enter value for the Next Y, Press ENTER");
  while (Serial.available() == 0) ;  // Wait here until input buffer has a character
  {
    nextPos[1] = Serial.parseInt();      
    Serial.print("Y = "); Serial.println(nextPos[1], DEC);

    while (Serial.available() > 0)  // .parseInt() can leave non-numeric characters
    { junk = Serial.read() ; }      // clear the keyboard buffer
  }
}


void askNextPosition()
{ 
   askNextX();
   askNextY();
   askNextHeading(); 
}


void askRobotFrontWall() {
  double distance = ultrasonicRange();

  for (int i = 1; i <= 4; i++) {
    if (distance < (CELL_SIZE * i)) { frontWall = i; return; }
  }

  frontWall = 0;
}


void askRobotLeftWall(){
  double distance = laser1.readRangeContinuousMillimeters()/10.0;

  if ((distance > 2.0) && (distance < 12.5)) {
    leftWall = 1;
  } else {
    leftWall = 2;
  }
}


void askRobotRightWall(){
  double distance = laser2.readRangeContinuousMillimeters()/10.0;

  if ((distance > 2.0) && (distance < 12.5)) {
    rightWall = 1;
  } else {
    rightWall = 2;
  }  
}


void askUserLeftWall () 
{ 
  char junk = ' '; // character to hold the random values in a string after a key is pressed

  Serial.println("Enter value for the Left Wall Status 'LW', Press ENTER");
  Serial.println(" 0 = unknown, 1 = Seen a wall, 2 = No Wall ");
  while (Serial.available() == 0) ;  // Wait here until input buffer has a character
  {
    leftWall = Serial.parseInt();  
    if( (leftWall < 1) || (leftWall > 2) )
    { 
       leftWall = 0;       
    }      
    Serial.print("LW = "); Serial.println(leftWall, DEC);

    while (Serial.available() > 0)  // .parseInt() can leave non-numeric characters
    { junk = Serial.read() ; }      // clear the keyboard buffer
  }
}


void askUserRightWall () 
{ 
  char junk = ' '; // character to hold the random values in a string after a key is pressed

  Serial.println("Enter value for the Right Wall Status 'RW', Press ENTER");
  Serial.println(" 0 = unknown, 1 = Seen a wall, 2 = No Wall ");
  while (Serial.available() == 0) ;  // Wait here until input buffer has a character
  {
    rightWall = Serial.parseInt();  
    if( (rightWall < 1) || (rightWall > 2) )
    { 
       rightWall = 0;       
    }
    
    Serial.print("RW = "); Serial.println(rightWall, DEC);

    while (Serial.available() > 0)  // .parseInt() can leave non-numeric characters
    { junk = Serial.read() ; }      // clear the keyboard buffer
  }
}


void askUserFrontWall () 
{ 
  char junk = ' '; // character to hold the random values in a string after a key is pressed

  Serial.println("Enter value for the Front Wall Status 'FW', Press ENTER");
  Serial.println("0 = unknown, 1 = wall directly in front, 2 = wall in second square in front");
  Serial.println("3 = wall in third square in front,       4 = wall in fourth square in front");
  
  while (Serial.available() == 0) ;  // Wait here until input buffer has a character
  {
    frontWall = Serial.parseInt();   
    if( (frontWall < 1) || (frontWall > 4) )
    { 
      frontWall = 0;      
    }
  
    Serial.print("FW = "); Serial.println(frontWall, DEC);

    while (Serial.available() > 0)  // .parseInt() can leave non-numeric characters
    { junk = Serial.read() ; }      // clear the keyboard buffer
  }
}


void displayPos(int pos[3])
{ 
  Serial.print("X: ");
  Serial.print(pos[0]);
  Serial.print(" Y: ");
  Serial.print(pos[1]);
  Serial.print(" Heading: ");
  Serial.print(pos[2]);
  Serial.println();
}


int getDirection(float heading)
{
  while (heading >  225) { heading = heading - 360; }
  while (heading < -135) { heading = heading + 360; }
  
  if                     (heading <= -135)  {  return 180;  }
  if((heading > -135) && (heading <=  -45)) {  return -90;  }
  if((heading >   45) && (heading <=  135)) {  return  90;  }
  if (heading >  135)                       {  return 180;  }

  return 0;
}


int getHeading()  {  return currentPos[2];  }


int getXPos()  {  return currentPos[0];  }
int getYPos()  {  return currentPos[1];  }


void moveNextPosition()
{ 

  if(DEBUG_ROBOT)
  { 
    currentPos[0] = nextPos[0];
    currentPos[1] = nextPos[1];
    currentPos[2] = nextPos[2];   
  }
  else
  {
    if (nextPos[0] == (currentPos[0] - 1)) {
      if (nextPos[1] == currentPos[1]) {
        if (currentPos[2] != 180) {
          turnToHeading(180);
          currentPos[2] = 180;
        }
        forwardNumCells(1);
        currentPos[0] = nextPos[0];
        return;
      }
    }
    if (nextPos[0] == currentPos[0]) {
      if (nextPos[1] == (currentPos[1] - 1)) {
        if (currentPos[2] != -90) {
          turnToHeading(-90);
          currentPos[2] = -90;
        }
        forwardNumCells(1);
        currentPos[1] = nextPos[1];
        return;
      }
      if (nextPos[1] == currentPos[1]) {
        if (nextPos[2] != currentPos[2]) {
          turnToHeading(nextPos[2]);
          currentPos[2] = nextPos[2];
        }
        return;
      }
      if (nextPos[1] == (currentPos[1] + 1)) {
        if (currentPos[2] != 90) {
          turnToHeading(90);
          currentPos[2] = 90;
        }
        forwardNumCells(1);
        currentPos[1] = nextPos[1];
        return;
      }
    }
    if (nextPos[0] == (currentPos[0] + 1)) {
      if (nextPos[1] == currentPos[1]) {
        if (currentPos[2] != 0) {
          turnToHeading(0);
          currentPos[2] = 0;
        }
        forwardNumCells(1);
        currentPos[0] = nextPos[0];
        return;
      }
    }
    
   }
}


void setCurrentPos(int x, int y, int heading) 
{ 
  currentPos[0] = x;
  currentPos[1] = y; 
  currentPos[2] = heading; 
}

void turnClockwiseToAngle(double targetAngle) {
  double currAngle = readAngleZ();

  // turn clockwise
  digitalWrite(MOTOR1DIRECTION_PIN, HIGH);
  digitalWrite(MOTOR2DIRECTION_PIN, HIGH);
  
  analogWrite(MOTOR1SPEED_PIN, 255); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 255); //MOTOR2DIRECTION_PIN drives at _Speed

  currAngle = readAngleZ();
  while ((targetAngle - currAngle) < 0){
    currAngle = readAngleZ();
  }
  
  analogWrite(MOTOR1SPEED_PIN, 0); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 0); //MOTOR2DIRECTION_PIN drives at _Speed

  // turn anti-clockwise
  digitalWrite(MOTOR1DIRECTION_PIN, LOW);
  digitalWrite(MOTOR2DIRECTION_PIN, LOW);

  analogWrite(MOTOR1SPEED_PIN, 125); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 125); //MOTOR2DIRECTION_PIN drives at _Speed

  currAngle = readAngleZ();
  while ((targetAngle - currAngle) > 0){
    currAngle = readAngleZ();
  }
  
  analogWrite(MOTOR1SPEED_PIN, 0); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 0); //MOTOR2DIRECTION_PIN drives at _Speed
}

void turnAntiClockwiseToAngle(double targetAngle) {
  double currAngle = readAngleZ();

  // turn anti-clockwise
  digitalWrite(MOTOR1DIRECTION_PIN, LOW);
  digitalWrite(MOTOR2DIRECTION_PIN, LOW);
  
  analogWrite(MOTOR1SPEED_PIN, 255); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 255); //MOTOR2DIRECTION_PIN drives at _Speed

  currAngle = readAngleZ();
  while ((targetAngle - currAngle) > 0){
    currAngle = readAngleZ();
  }
  
  analogWrite(MOTOR1SPEED_PIN, 0); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 0); //MOTOR2DIRECTION_PIN drives at _Speed

  // turn clockwise
  digitalWrite(MOTOR1DIRECTION_PIN, HIGH);
  digitalWrite(MOTOR2DIRECTION_PIN, HIGH);

  analogWrite(MOTOR1SPEED_PIN, 125); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 125); //MOTOR2DIRECTION_PIN drives at _Speed

  currAngle = readAngleZ();
  while ((targetAngle - currAngle) < 0){
    currAngle = readAngleZ();
  }
  
  analogWrite(MOTOR1SPEED_PIN, 0); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, 0); //MOTOR2DIRECTION_PIN drives at _Speed
}

void turnToAngle(double targetAngle) {
  double currAngle = readAngleZ();
  
  if (targetAngle > currAngle) {
    turnAntiClockwiseToAngle(targetAngle);
  } else {
    turnClockwiseToAngle(targetAngle);
  }
}


void turnToHeading(double targetHeading) {
  double targetAngle = targetHeading + zeroAngleZ;

  turnToAngle(targetAngle);
}

void turnAngle(double angle) {
  double targetAngle = readAngleZ() + angle;
  
  turnToAngle(targetAngle);
}


float getTargetHeading(float initHeading) {
  float heading = initHeading - zeroAngleZ;

  while (heading >  225) { heading = heading - 360; }
  while (heading < -135) { heading = heading + 360; }
  
  float deltaHeading = getDirection(heading) - heading;

  return (initHeading + deltaHeading);
}


void forwardNumCells(int numCells) {
  if (numCells < 0) return;
  
  double distanceFromWall = ultrasonicRange();
  double initDistance     = distanceFromWall;
  double requestDistance  = CELL_SIZE * numCells;  // distance requested to travel

  if (requestDistance > initDistance) return;
  
  double targetDistance   = initDistance - requestDistance;
  double tolerance        = 0.2 * CELL_SIZE;       //  1/5 of cell size
  double safeDistance     = 0.5 * CELL_SIZE;
  double leftDistance     = laser1.readRangeContinuousMillimeters()/10;
  double rightDistance    = laser2.readRangeContinuousMillimeters()/10;

  float initHeading = readAngleZ();
  float targetHeading = getTargetHeading(initHeading);
  float currHeading = readAngleZ();

  _Speed1 = 255;
  _Speed2 = 255;
  setMotorForward(MOTOR1DIRECTION_PIN);
  setMotorReverse(MOTOR2DIRECTION_PIN);
  analogWrite(MOTOR1SPEED_PIN, _Speed1); //MOTOR1DIRECTION_PIN drives at _Speed
  analogWrite(MOTOR2SPEED_PIN, _Speed2); //MOTOR2DIRECTION_PIN drives at _Speed

  distanceFromWall = ultrasonicRange();
  while ((distanceFromWall > safeDistance) && ((distanceFromWall - targetDistance) > tolerance)) { //repeatedly measure the distance
    distanceFromWall = ultrasonicRange();
    leftDistance  = laser1.readRangeContinuousMillimeters()/10; //measure distances to the right and left
    rightDistance = laser2.readRangeContinuousMillimeters()/10;
    currHeading = readAngleZ();
    if ((leftDistance < 8) || ((targetHeading - currHeading) < -3 )) { //if too close to left wall, veer right
      _Speed1 = 255;
      _Speed2 = 200;
    } else if ((rightDistance < 8) || ((targetHeading - currHeading) > 3)) { //if too close to right wall, veer left
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
}


//

#define CellSize 23.5

bool newData = false;
long receivedNum;
int E1 = 5; //speed control for motor 1
int M1 = 4; //direction control for motor 1
int E2 = 6; //speed control for motor 2
int M2 = 7; //direction control for motor 2
int _Speed = 0; //speed for motors
int trigPin = 11; //use 30/1 on mega
int echoPin = 12; //use 30/1 on mega
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
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

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

void function1() {
  
}

void function2() {

}

void function3() {
  double distance = ultrasonicRange();
  _Speed = 170; // (2/3)*255
  digitalWrite(M1, LOW); //set M1 to forward
  digitalWrite(M2, LOW); //set M2 to forward
  analogWrite(E1, _Speed); //M1 drives at _Speed
  analogWrite(E2, _Speed); //M2 drives at _Speed
  _Speed = 0;

  while (distance > (CellSize/3)){ //repeatedly measure the distance until it is 1/3 of cell size
    distance = ultrasonicRange();
  }
  analogWrite(E1, _Speed); //M1 stops
  analogWrite(E2, _Speed); //M2 stops
}

void function4() {
  
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
  attachInterrupt(encoder1pinA, wheelSpeed1, CHANGE);
}

void wheelSpeed1()
{
  int Lstate = digitalRead(encoder1pinA);
  if((encoder1PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder1pinB);
    if(val == LOW && Direction1)
    {
      Direction1 = false; //Reverse
    }
    else if(val == HIGH && !Direction1)
    {
      Direction1 = true;  //Forward
    }
  }
  encoder1PinALast = Lstate;

  if(!Direction1)  duration1  ;
  else  duration1--;
}

void Encoder2Init()
{
  Direction2 = true;//default -> Forward
  pinMode(encoder2pinB,INPUT);
  attachInterrupt(encoder2pinA, wheelSpeed1, CHANGE);
}

void wheelSpeed2()
{
  int Lstate = digitalRead(encoder2pinA);
  if((encoder2PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder2pinB);
    if(val == LOW && Direction2)
    {
      Direction2 = false; //Reverse
    }
    else if(val == HIGH && !Direction2)
    {
      Direction2 = true;  //Forward
    }
  }
  encoder2PinALast = Lstate;

  if(!Direction2)  duration2  ;
  else  duration2--;
}

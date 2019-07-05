int E1 = 5; //speed control for motor 1 (left motor)
int M1 = 4; //direction control for motor 1
int E2 = 6; //speed control for motor 2 (right motor)
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

int numPulses = 500;

void setup() {
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  Encoder1Init();
  Encoder2Init();
}

void loop() {
  int i = 1;
  if (i = 1){
    digitalWrite(M1, HIGH); //set M1 (left motor) to Backward
    digitalWrite(M2, LOW); //set M2 (right motor) to forward
  
    _Speed = 255;
    duration1 = 0;
    analogWrite(E1, _Speed); //M1 drives at _Speed
    analogWrite(E2, _Speed); //M2 drives at _Speed
    _Speed = 0;
      
    while (duration1 < numPulses) {
      //do nothing
    }
    analogWrite(E1, _Speed); //M1 drives at _Speed
    analogWrite(E2, _Speed); //M2 drives at _Speed
    i++;
  } else {
    //do nothing
  }
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

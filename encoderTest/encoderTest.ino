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

int numPulses = 352;

void setup() {
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  Encoder1Init();
  Encoder2Init();

    digitalWrite(M1, HIGH); //set M1 (left motor) to Backward
    digitalWrite(M2, LOW); //set M2 (right motor) to forward
  
    _Speed = 255;
    duration1 = 0;
    analogWrite(E1, _Speed); //M1 drives at _Speed
    analogWrite(E2, _Speed); //M2 drives at _Speed
    _Speed = 0;
}

void loop() {
//      Serial.println(duration1);
//if (duration1 > numPulses){
//    analogWrite(E1, _Speed); //M1 drives at _Speed
//    analogWrite(E2, _Speed); //M2 drives at _Speed
//
//}
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

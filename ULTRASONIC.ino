//This program detects the distance from an obstacle using an ultrasonic sensor
//Written by Olivia Yem

int trigPin = 11;
int echoPin = 12;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  pinMode(echoPin, INPUT);
  long duration = pulseIn(echoPin, HIGH);
 

  double cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  
  Serial.print(cm);
  Serial.println(" cm");
  
  delay(250);
}

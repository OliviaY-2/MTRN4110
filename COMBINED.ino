#include <Adafruit_VL6180X.h>
#include <Wire.h>

Adafruit_VL6180X laser = Adafruit_VL6180X(); //create object using I2C

int trigPin = 11;
int echoPin = 12;

void setup() {
  Serial.begin(9600); //initialise serial
  if (! laser.begin()){
    Serial.println("LIDAR not initialised. Check wiring");
    while(1);
  }
  Serial.println("LIDAR initialised");  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT); //initialise pins for ultrasonic
}

void loop() {
  //Measure distance using ultrasonic
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

  //Measure distance using LiDAR
  uint8_t range = laser.readRange();
  uint8_t status = laser.readRangeStatus();
  if(status == VL6180X_ERROR_NONE){ //if there was no error, print the range
    Serial.print("Range: ");
    Serial.print(range);
    Serial.println("mm");
  }
  delay(500); //delay for readability
}

/*
///////////////////////////////////////////////
// Ultrasonic V2.00
///////////////////////////////////////////////

#include <external_VL6180X.h>

#define TRIG_PIN 30 
#define ECHO_PIN 31

void initUltrasonic();
double ultrasonicRange();
void testUltrasonic();

///////////////////////////////////////////////
*/


void initUltrasonic() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}


double readUltrasonicRange() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  double cm = (double)duration * (double)(0.0343) * (double)(0.5);  // Divide by 29.1 or multiply by 0.0343
  return cm;  // return distance in cm
}

double ultrasonicRange()
{
  bool isGood = false;
  int iterations = 4;
  double tooShort = 2.0;
  double tooLong = 400.0;
  double minD = 500.0;
  double maxD = 0.0;
  double totDist;
  double maxError = 0.20;

  double averageDist, deltaDist;
  double dist;

  while (!isGood)
  {
    minD = 5000.0;
    maxD = 0.0;
    totDist = 0.0;
    for (int i = 0; i < iterations; i++) {
      dist = readUltrasonicRange();
      while ((dist < tooShort) || (dist > tooLong)) {
        dist = readUltrasonicRange();
      }
      if(dist < minD) { minD = dist; }
      if(maxD < dist) { maxD = dist; }
      totDist = totDist + dist;
    }

    averageDist = totDist / iterations;
    deltaDist = maxD - minD;

    if ((deltaDist / averageDist) < maxError) { isGood = true; }
  }

  return averageDist;
}


void testUltrasonic()
{ 
  long currTime = millis();
  double goodDist;
  
  for(int i = 0; i < 9; i++){ 

    currTime = millis();
    goodDist = ultrasonicRange();
    Serial.print("TU: ");
    Serial.print("Current Time ");
    Serial.print(currTime); 
    Serial.print(" Ultrasonic cm ");
    Serial.print(goodDist);
    Serial.println(); 
    Serial.println(); 

    delay(2000); 
  }
  
  Serial.println("TU: Finished");
}

//

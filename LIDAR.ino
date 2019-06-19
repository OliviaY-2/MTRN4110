#include <Adafruit_VL6180X.h>
#include <Wire.h>

Adafruit_VL6180X laser = Adafruit_VL6180X(); //create object using I2C

void setup() {
  Serial.begin(9600); //initialise serial
  if (! laser.begin()){
    Serial.println("LIDAR not initialised. Check wiring");
    while(1);
  }
  Serial.println("LIDAR initialised");
}

void loop() {
  uint8_t range = laser.readRange();
  uint8_t status = laser.readRangeStatus();

  if(status == VL6180X_ERROR_NONE){ //if there was no error, print the range
    Serial.print("Range: ");
    Serial.print(range);
    Serial.println("mm");
  }
  delay(500); //delay for readability
}

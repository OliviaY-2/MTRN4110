/*
///////////////////////////////////////////////
// LED V1.01 changed
///////////////////////////////////////////////

// PIN numbers for each LED
#define RED_LED 36
#define GREEN_LED 37

// 0 = Green light off, Red light off
// 1 = Green light on,  Red light off
// 2 = Green light off, Red light on  
// 3 = Green light on,  Red light on 
int LED_State = 0; 

void initLEDS();
bool isLED_on(int colourLED);
void updateLED(int ledState);
void setLED(int colourLED, int ledState);

///////////////////////////////////////////////
*/

#define DEBUG_LEDS false


void initLEDS() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  updateLED(0);
}

bool isLED_on(int colourLED)
{
  if (colourLED == GREEN_LED) { return ((LED_State     % 2) == 1); }
  if (colourLED == RED_LED)   { return ((LED_State / 2 % 2) == 1); }
}


void updateLED(int ledState) 
{ 
  switch(ledState){ 
    case 0: 
     setLED(GREEN_LED, 0); 
     setLED(RED_LED,   0);
     if (DEBUG_LEDS) Serial.println("GREEN OFF  RED OFF"); 
     break;
    
    case 1: 
     setLED(GREEN_LED, 1); 
     setLED(RED_LED,   0);
     if (DEBUG_LEDS) Serial.println("GREEN ON   RED OFF"); 
     break;

    case 2: 
     setLED(GREEN_LED, 0); 
     setLED(RED_LED,   1);
     if (DEBUG_LEDS) Serial.println("GREEN OFF  RED ON"); 
     break;

    case 3: 
     setLED(GREEN_LED, 1); 
     setLED(RED_LED,   1);
     if (DEBUG_LEDS) Serial.println("GREEN ON   RED ON"); 
     break;  
  } 
}


void setLED(int colourLED, int ledState)
{
  if (DEBUG_LEDS) {
    Serial.print("setLED: start");
    Serial.print(" LED_State: "); Serial.print(LED_State);
    Serial.print(" ledState: ");  Serial.print(ledState);
    Serial.print(" colourLED: "); Serial.println(colourLED);
  }
  
  int change;
  
  if (colourLED == GREEN_LED) { change = 1; }
  if (colourLED == RED_LED)   { change = 2; }

  if ((ledState != 0) && (!isLED_on(colourLED)))
  {
    LED_State = LED_State + change;
    digitalWrite(colourLED, HIGH);
  }
  if ((ledState == 0) && (isLED_on(colourLED)))
  {
    LED_State = LED_State - change;
    digitalWrite(colourLED, LOW);
  }

  if (DEBUG_LEDS) {
    Serial.print("setLED: end  ");
    Serial.print(" LED_State: "); Serial.print(LED_State);
    Serial.print(" ledState: ");  Serial.print(ledState);
    Serial.print(" colourLED: "); Serial.println(colourLED);
  }
}

void flashRED() {
  for (int i = 0; i < 7; i++) {
    digitalWrite(RED_LED, HIGH);
    delay(200);
    digitalWrite(RED_LED, LOW);
    delay(200);
  }
}

//

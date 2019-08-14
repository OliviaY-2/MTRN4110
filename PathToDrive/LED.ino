/*
///////////////////////////////////////////////
// LED V1.01
///////////////////////////////////////////////

// PIN numbers for each LED
#define RED_LED 36
#define GREEN_LED 37

// 0 = both lights off
// 1 = ledState :  Green light on,  Red light off
// 2 = ledState :  Green light off, Red light on  
// 3 = ledState :  Green light on,  Red light on 
int LED_State = 0; 

void initLEDS();
bool isLED_on(int colourLED);
void updateLED(int ledState);
void setLED(int colourLED, int ledState);

///////////////////////////////////////////////
*/

#define DEBUG_LEDS true


void initLEDS() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
}

bool isLED_on(int colourLED)
{
  if (colourLED == GREEN_LED) { return (LED_State     % 2); }
  if (colourLED == RED_LED)   { return (LED_State / 2 % 2); }
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


//

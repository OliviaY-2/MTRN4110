////////////////////////////////////////
// PathToDrive V1.00
///////////////////////////////////////

char receivedNum = 'A';
boolean newData = false;

void recvNum ();
void decisionTree ();

///////////////////////////////////////



void setup() {
  Serial.begin(9600);
  pathFromString(String(112113110));
}

void loop() {

}


void recvNum() {
  if (Serial.available() > 0) {
    receivedNum = Serial.read();
    newData = true;
   }
}


void decisionTree() {
  //if (newData == true) {
    switch (receivedNum){
      case '1':
        forward1cell();
        break;
      case '2':
        left90deg();
        break;
      case '3':
        right90deg();
        break;
      case '0':
        Serial.println("End of path!");
        //updateLED(2);
        break;
      default:
        break;
    }
    //newData = false;
  //}
}


void pathFromSerial() {
  receivedNum = 'A';
  while (receivedNum != '0'){
    recvNum();
    decisionTree();
  }
}

void pathFromString(String Instructions){
  for (int i = 0; i < Instructions.length(); i++) {
    receivedNum = Instructions[i];
    decisionTree();
  }
}

void forward1cell(){
  Serial.println("forward");
}


void left90deg() {
  Serial.println("left");
}

void right90deg() {
  Serial.println("right");
}

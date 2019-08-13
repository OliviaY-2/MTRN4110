//Code to receive planned path and enact it

char receivedNum;
boolean newData = false;

void setup() {
  Serial.begin(9600);
}

void loop() {
  recvNum();
  decisionTree();
}


void recvNum() {
  if (Serial.available() > 0) {
    receivedNum = Serial.read();
    newData = true;
   }
}


void decisionTree() {
  if (newData == true) {
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
        break;
      default:
        break;
    }
    newData = false;
  }
}


void forward1cell() {
  Serial.println("Forward");
}


void left90deg() {
  Serial.println("Left");
}

void right90deg() {
  Serial.println("Right");
}

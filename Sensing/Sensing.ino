
long receivedNum;
boolean newData = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Please enter a number 1 - 4");
}

void loop() {
  recvNum();
  decisionTree();
}

void recvNum() {
  if (Serial.available() > 0) {
    receivedNum = Serial.parseInt();
    newData = true;
   }
}

void decisionTree() {
  if (newData == true) {
    switch (receivedNum){
      case 1:
        function1();
        break;
      case 2:
        function2();
        break;
      case 3:
        function3();
        break;
      case 4:
        function4();
        break;
      default:
        Serial.println("Invalid input");
        break;
    }
    newData = false;
  }
}

void function1 () {
  Serial.println("Entered 1");
}

void function2 () {
  Serial.println("Entered 2");
}

void function3 () {
  Serial.println("Entered 3");
}

void function4 () {
  Serial.println("Entered 4");
}

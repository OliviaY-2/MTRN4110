#define CellSize 

bool newData = false;
long receivedNum;

void setup() {
  Serial.begin(9600);
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
        Serial.println("Please enter a number 1 - 4");
        break;
    }
    newData = false;
  }
}

void function1() {
  
}

void function2() {
  
}

void function3() {
  
}

void function4() {
  
}

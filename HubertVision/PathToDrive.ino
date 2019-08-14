/*
////////////////////////////////////////
// PathToDrive V1.00
///////////////////////////////////////

void pathFromString(String Instructions);
void decisionTree();

///////////////////////////////////////
*/

void decisionTree(char receivedNum) {
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
        updateLED(2);
        break;
      default:
        break;
    }
}

void pathFromString(String Instructions){
  char receivedNum;
  for (int i = 0; i < Instructions.length(); i++) {
    receivedNum = Instructions[i];
    decisionTree(receivedNum);
  }

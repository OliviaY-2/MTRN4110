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
        forwardNumCells(1);
        break;
      case '2':
        turnAngle(90); // left
        break;
      case '3':
        turnAngle(-90); // right
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
}

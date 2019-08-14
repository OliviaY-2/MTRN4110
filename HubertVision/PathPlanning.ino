// MTRN4110 - Phase B - Path Planning
// Hamish Morgan - Thursday Group 6

// Points are defined as:
// [Ypos,Xpos,predecessorID,numSteps,ID]

// Directions are defined as:
// 1: Right
// 2: Up
// 3: Left
// 4: Down

// Maps are encoded as arrays, with an int for each element. 
// values as follows:
// 0: No bottom wall, no right wall
// 1: No bottom wall, yes right wall
// 2: No bottom wall, uncertain right wall
// 3: Yes bottom wall, no right wall
// 4: Yes bottom wall, yes right wall
// 5: Yes bottom wall, uncertain right wall
// 6: Uncertain bottom wall, no right wall
// 7: Uncertain bottom wall, yes right wall
// 8: Uncertain bottom wall, uncertain right wall

// The design concept is to declare globals and change them when necessary, 
// rather than passing variables around functions


// THIS IS THE ORIGINAL MAP
//int map1[5][9] = { {3,0,3,3,4,8,8,8,7},
//                   {0,4,0,1,5,8,8,8,7},
//                   {0,3,1,1,0,5,8,8,7},
//                   {1,3,1,1,3,0,8,8,7},
//                   {3,3,3,3,3,4,5,5,4} };

/*
// Initialise a bogus map
int map1[5][9] = { {8,0,3,3,4,8,8,8,7},
                   {5,4,2,4,6,6,8,8,7},
                   {6,3,7,1,6,6,8,8,7},
                   {8,3,1,1,6,6,6,8,7},
                   {3,3,3,3,3,4,5,5,4} };

// Initialise positions array
// 100 rows should suffice
int positions[300][5];

// Arrays for best solutions and interim solution for calculating best soln
int bestSolutionUncertain[30][5];
int bestSolutionCertain[30][5];
int currentSolution[30][5];

// String storing forward rotate etc instructions
String instructions = "";

// Some variables to store the information that will be used to confirm that
// the map has been fully explored
int uncertainSteps = 0;
int uncertainTurns = 0;
int certainSteps = 0;
int certainTurns = 0;

// initialise other globals
int id = 0;
int iterations = 0;

// Initialise a variable to make the if statements in nextMove much easier to write
int CIQ = 0;
int newXpos;
int newYpos;

// Define the target location and the starting location and starting position
int targetCell[2] = {2,4};
int startCell[2] = {4,4};
int startDir = 3;

// Initialise a variable to break the main loop
bool reachedTarget = false;

// variable for BT input, initialised to something random
char state = 'A'; 
char BTstate = 'N';
String input = "";
boolean NL = true;
*/




//////////////////////////////
//   Function definitions   //
//////////////////////////////

void nextMove(int pos[5], int currentMap[5][9], bool includeUncertain)
{
  // Can we move up?
  if (pos[0] > 0)
  {
    CIQ = currentMap[pos[0]-1][pos[1]];
    if (CIQ == 3 || CIQ == 4 || CIQ == 5 || includeUncertain == false && (CIQ == 6 || CIQ == 7 || CIQ == 8))
    {
    }
    else
    {
      newYpos = pos[0];
      newXpos = pos[1];
      newYpos--;
      if (discardMove(newYpos,newXpos,pos[3]) == false)
      {
        positions[id][0] = pos[0]-1;
        positions[id][1] = pos[1];
        positions[id][2] = pos[4];
        positions[id][3] = pos[3];
        positions[id][3]++;
        positions[id][4] = id;
        id++;
      }
    }
  }
  
  // Can we move left?
  if (pos[1] > 0)
  {
    CIQ = currentMap[pos[0]][pos[1]-1];
    if (CIQ == 1 || CIQ == 4 || CIQ == 7 || includeUncertain == false && (CIQ == 2 || CIQ == 5 || CIQ == 8))
    {
    }
    else
    {
      newYpos = pos[0];
      newXpos = pos[1];
      newXpos--;
      if (discardMove(newYpos,newXpos,pos[3]) == false)
      {
        positions[id][0] = pos[0];
        positions[id][1] = pos[1]-1;
        positions[id][2] = pos[4];
        positions[id][3] = pos[3];
        positions[id][3]++;
        positions[id][4] = id;
        id++;
      }
    }
  }

  // Can we move down?
  if (pos[0] < 5)
  {
    CIQ = currentMap[pos[0]][pos[1]];
    if (CIQ == 3 || CIQ == 4 || CIQ == 5 || includeUncertain == false && (CIQ == 6 || CIQ == 7 || CIQ == 8))
    {
    }
    else
    {
      newYpos = pos[0];
      newXpos = pos[1];
      newYpos++;
      if (discardMove(newYpos,newXpos,pos[3]) == false)
      {
        positions[id][0] = pos[0]+1;
        positions[id][1] = pos[1];
        positions[id][2] = pos[4];
        positions[id][3] = pos[3]+1;
        positions[id][3]++;
        positions[id][4] = id;
        id++;
      }
    }
  }

  // Can we move right?
  if (pos[1] < 9)
  {
    CIQ = currentMap[pos[0]][pos[1]];
    if (CIQ == 1 || CIQ == 4 || CIQ == 7 || includeUncertain == false && (CIQ == 2 || CIQ == 5 || CIQ == 8))
    {
    }
    else
    {
      newYpos = pos[0];
      newXpos = pos[1];
      newXpos++;
      if (discardMove(newYpos,newXpos,pos[3]) == false)
      {
        positions[id][0] = pos[0];
        positions[id][1] = pos[1]+1;
        positions[id][2] = pos[4];
        positions[id][3] = pos[3];
        positions[id][3]++;
        positions[id][4] = id;
        id++;
      }
    }
  }    
}



bool discardMove(int newY, int newX, int numSteps)
{
  bool discard = false;
  for (int i = 0; i < id; i++)
  {
    if (positions[i][0] == newY && positions[i][1] == newX && positions[i][3] < numSteps)
    {
      discard = true;
    }
  }
  return discard;
}



void planPath(bool includeUncertain)
{
  
  id = 0;
  iterations = 0;
  
  // Set all values in the relevant final solution array to -1
  if (includeUncertain == true)
  {
    memset(bestSolutionUncertain,-1,sizeof(bestSolutionUncertain));
    uncertainSteps = 0;
    uncertainTurns = 0;
  }
  else
  {
    memset(bestSolutionCertain,-1,sizeof(bestSolutionCertain));
    certainSteps = 0;
    certainTurns = 0;
  }
  
  // Ensure all position array elements are initialised to 0
  memset(positions,-1,sizeof(positions));
  
  // Initialise our first point
  positions[0][0] = startCell[0];
  positions[0][1] = startCell[1];
  positions[0][2] = -1;
  positions[0][3] = 0;
  positions[0][4] = id;
  id++;
  
  // This function takes in the current position and updates the positions list with valid next moves
  // we can then filter out the next moves based on the number of steps to get there by using iterations
  nextMove(positions[0], map1, includeUncertain);
  iterations+=1;
  reachedTarget = false;

  while (reachedTarget == false)
  {
    for (int l = 0; l <= id; l++)
    {
      if (l == 99)
      {
//        Serial.println("OVERFLOWWWWWWWWWWWW");
      }
      if (positions[l][3] == iterations)
      {
        nextMove(positions[l], map1, includeUncertain);
        if (positions[l][0] == targetCell[0] && positions[l][1] == targetCell[1])
        {
          reachedTarget = true;
        }
      }
    }
    iterations+=1;
  }

  // Extract solution
  Serial.println("Extracting solution...");
  extractSolution(includeUncertain);

  // Print solution
  Serial.println("Printing solution...");
  printSolution(includeUncertain);
  
  // Print map
  Serial.println("Printing map...");
  printMap(includeUncertain);
}



void extractSolution(bool includeUncertain)
{
  // Initialise local variables which can be cleared after execution
  int bestTurns = 10000;
  int numTurns = 0;
  int prevDir = 0;
  int dir = 0;
  int prevPoint[5] = {0,0,0,0,0};
  int j = 0;
  bool foundStart = false;

  // Set all values in the currentSolution array to -1
  memset(currentSolution, -1, sizeof(currentSolution));
//  delay(100);

  // Iterate through positions list
  for (int i = id; i >= 0; i--)
  {
    // When we find the finishing tile
    if (positions[i][0] == targetCell[0] && positions[i][1] == targetCell[1])
    {
      // Reset variables
      prevDir = 0;
      dir = 0;
      prevPoint = {0,0,0,0,0};
      j = 0;
      foundStart = false;
      memset(currentSolution, -1, sizeof(currentSolution));
      
      // Set the first element of currentSolution to the finishing move specified by positions[i]
      for (int p = 0; p < 5; p++)
      {
        currentSolution[j][p] = positions[i][p];
//        Serial.print(currentSolution[j][p]);
//        Serial.print(" ");
//        delay(50);
      }
//      Serial.println();

      // Iterate back through the path until we find the start tile
      while (foundStart == false)
      {
        // Set prevPoint to the current point
        for (int p = 0; p < 5; p++)
        {
          prevPoint[p] = currentSolution[j][p];
        }

        // Iterate back through to find the predecessor point
        for (int k = id; k >= 0; k--)
        {
          // When we find the point that preceded the current point
          if (positions[k][4] == currentSolution[j][2]) 
          {
            // Increment counter and append the positions[k] to the current solution list
            j++;
            for (int p = 0; p < 5; p++)
            {
              currentSolution[j][p] = positions[k][p];
//              Serial.print(currentSolution[j][p]);
//              Serial.print(" ");
//              delay(10);
            }
//            Serial.println("");
//            delay(200);
            if (currentSolution[j][0] == startCell[0] && currentSolution[j][1] == startCell[1])
            {
//              Serial.println("Found the start");
              foundStart = true;
            }
            

            // Determine the direction
            if (prevPoint[0] == currentSolution[j][0]) // Points have the same Y
            {
              if (prevPoint[1] > currentSolution[j][1]) // to the right
              {
                dir = 1; // 1 -> right
              }
              else // to the left
              {
                dir = 3; // 3 -> left
              }
            }
            else // points have the same X
            {
              if (prevPoint[0] > currentSolution[j][0]) // below
              {
                dir = 4; // 4 -> down
              }
              else
              {
                dir = 2; // 2 -> up
              }
            }
            
            // Increment the turn counter if a turn has been made and we aren't on the first step
            if (prevDir != 0)
            {
              if (prevDir != dir)
              {
                numTurns++;
              }

              if (foundStart == true)
              {
                // If a turn has to be made at the start before moving
                if (dir != startDir)
                {
                  numTurns++;
                }
              }
            }
            prevDir = dir;
            
            break;
          }
        }
      }

      // Assign bestTurns, bestSolution etc
      if (numTurns < bestTurns)
      {
        bestTurns = numTurns;
        if (includeUncertain == true)
        {
          uncertainSteps = j;
          uncertainTurns = bestTurns;
          for (int p = 0; p < 5; p++)
          {
            for (int q = 29; q >= 0; q--)
            {
              bestSolutionUncertain[q][p] = currentSolution[q][p];
            }
          }
          
        }
        else
        {
          certainSteps = j;
          certainTurns = bestTurns;
          for (int p = 0; p < 5; p++)
          {
            for (int q = 29; q >= 0; q--)
            {
              bestSolutionCertain[q][p] = currentSolution[q][p];
            }
          }
        }
      }
    }
  }
}



void printSolution(bool includeUncertain)
{
  int dir = startDir;
  int prevDir = 0;
  int prevPoint[5] = {-1,-1,-1,-1,-1};
  String steps = "";
  
  if (includeUncertain == true)
  {
    for (int i = 29; i >=0; i--)
    {
      if (bestSolutionUncertain[i][0] != -1)
      {
        Serial.print("X: ");
        Serial.print(bestSolutionUncertain[i][1]);
        Serial.print(", Y: ");
        Serial.println(bestSolutionUncertain[i][0]);

        prevDir = dir;
        if (prevPoint[0] != -1 && prevPoint[1] != -1)
        {
          // If points have the same y
          if (prevPoint[0] == bestSolutionUncertain[i][0])
          {
            // Robot has moved right
            if (prevPoint[1] < bestSolutionUncertain[i][1])
            {
              dir = 1;
            }
            else
            {
              dir = 3;
            }
          }
          else // Points have the same X
          {
            // Robot has moved down
            if (prevPoint[0] < bestSolutionUncertain[i][0])
            {
              dir = 4;
            }
            else
            {
              dir = 2;
            }
          }

          if (dir - prevDir == 1 || prevDir == 4 && dir == 1)
          {
            steps += "Rotate left\n";
            instructions += "2";
          }
          else if (prevDir - dir == 1 || prevDir == 1 && dir == 4)
          {
            steps += "Rotate right\n";
            instructions += "3";
          }
          steps += "Drive forward\n";
          instructions += "1";
        }

        // Assign prevPoint
        for (int j = 0; j < 5; j++)
        {
          prevPoint[j] = bestSolutionUncertain[i][j];
        }
      }
    }
    instructions += "0";
    Serial.println();
    Serial.println("Non-localised steps:");
    Serial.println(steps);
    Serial.println();
  }
  else
  {
    for (int i = 29; i >=0; i--)
    {
      if (bestSolutionCertain[i][0] != -1)
      {
        Serial.print("X: ");
        Serial.print(bestSolutionCertain[i][1]);
        Serial.print(", Y: ");
        Serial.println(bestSolutionCertain[i][0]);

        prevDir = dir;
        if (prevPoint[0] != -1 && prevPoint[1] != -1)
        {
          // If points have the same y
          if (prevPoint[0] == bestSolutionCertain[i][0])
          {
            // Robot has moved right
            if (prevPoint[1] < bestSolutionCertain[i][1])
            {
              dir = 1;
            }
            else
            {
              dir = 3;
            }
          }
          else // Points have the same X
          {
            // Robot has moved down
            if (prevPoint[0] < bestSolutionCertain[i][0])
            {
              dir = 4;
            }
            else
            {
              dir = 2;
            }
          }

          if (dir - prevDir == 1 || prevDir == 4 && dir == 1)
          {
            steps += "Rotate left\n";
          }
          else if (prevDir - dir == 1 || prevDir == 1 && dir == 4)
          {
            steps += "Rotate right\n";
          }
          steps += "Drive forward\n";
        }

        // Assign prevPoint
        for (int j = 0; j < 5; j++)
        {
          prevPoint[j] = bestSolutionCertain[i][j];
        }        
      }
    }
    Serial.println();
    Serial.println("Non-localised steps:");
    Serial.println(steps);
    Serial.println();
  }
}



void printMap(bool includeUncertain)
{
  int width = 9;
  int height = 5;
  bool onMap = false;
  int num = 0;

  if (includeUncertain == true)
  {
    for (int k = 0; k < 30; k++)
    {
      if (bestSolutionUncertain[k][0] != -1)
      {
        num++;
      }
    }
  }
  else
  {
    for (int k = 0; k < 30; k++)
    {
      if (bestSolutionCertain[k][0] != -1)
      {
        num++;
      }
    }
  }

  // Print top row
  for (int i = 0; i < width; i++)
  {
    Serial.print(" --");
  }
  Serial.println();
  
  // For each row:
  for (int j = 0; j < height; j++)
  {
    // Print step positions and walls
    Serial.print("|");
    for (int i = 0; i < width; i++)
    {
      if (includeUncertain == true)
      {
        onMap = false;
        for (int k = 0; k < num-1; k++)
        {
          if (bestSolutionUncertain[k][0] == j && bestSolutionUncertain[k][1] == i)
          {
            if (num-k < 10)
            {
              Serial.print(" ");
            }
            Serial.print(num-k);
            onMap = true;
          }
        }
        if (bestSolutionUncertain[num-1][0] == j && bestSolutionUncertain[num-1][1] == i)
        {
          onMap = true;        
          switch (startDir)
          {
            case 1:
              Serial.print(" >");
              break;
            case 2:
              Serial.print(" ^");
              break;
            case 3:
              Serial.print(" <");
              break;
            case 4:
              Serial.print(" v");
              break;
            default:
              Serial.print("XX");
              break;
          }
        }
        if (onMap == false)
        {
          Serial.print("  ");
        }
      }
      else
      {
        onMap = false;
        for (int k = 0; k < num-1; k++)
        {
          if (bestSolutionCertain[k][0] == j && bestSolutionCertain[k][1] == i)
          {
            if (num-k < 10)
            {
              Serial.print(" ");
            }
            Serial.print(num-k);
            onMap = true;
          }         
        }
        if (bestSolutionCertain[num-1][0] == j && bestSolutionCertain[num-1][1] == i)
        {
          onMap = true;        
          switch (startDir)
          {
            case 1:
              Serial.print(" >");
              break;
            case 2:
              Serial.print(" ^");
              break;
            case 3:
              Serial.print(" <");
              break;
            case 4:
              Serial.print(" v");
              break;
            default:
              Serial.print("XX");
              break;
          }
        }
        if (onMap == false)
        {
          Serial.print("  ");
        }        
      }
      
      if (map1[j][i] == 0 || map1[j][i] == 3 || map1[j][i] == 6)
      {
        Serial.print(" ");
      }
      else if (map1[j][i] == 1 || map1[j][i] == 4 || map1[j][i] == 7)
      {
        Serial.print("|");
      }
      else if (map1[j][i] == 2 || map1[j][i] == 5 || map1[j][i] == 8)
      {
        Serial.print("*");
      }
      else
      {
        Serial.print("X");
      }
    }
    Serial.println();

    // Print floors
    Serial.print("|");
    for (int i = 0; i < width; i++)
    {
      if (map1[j][i] == 0 || map1[j][i] == 1 || map1[j][i] == 2)
      {
        Serial.print("  ");
      }
      else if (map1[j][i] == 3 || map1[j][i] == 4 || map1[j][i] == 5)
      {
        Serial.print("--");
      }
      else if (map1[j][i] == 6 || map1[j][i] == 7 || map1[j][i] == 8)
      {
        Serial.print("..");
      }
      else
      {
        Serial.print("X");
      }
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("\n\n");
}



void decodeInput(String inputString)
{
  if (inputString[0] == 'M' && inputString[1] == 'A' && inputString[2] == 'P')
  {
    Serial.println("I have received a map");
    Serial.println("\n");
    decodeMap(inputString);
  }
}



void decodeMap(String inputString)
{  
  // We assume that a map has been passed appropriately
  // The expected format of data is immediately after the P in MAP:
  // startCell (as specified above), startDir (as specified above), 
  // then the map, row by row

  // Start by setting starting cell and dir
  startCell[0] = inputString[3] - '0';
  startCell[1] = inputString[4] - '0';
  startDir = inputString[5] - '0';

  // Display this data
  Serial.print("Start X: ");
  Serial.print(startCell[1]);
  Serial.print(", Start Y: ");
  Serial.print(startCell[0]);
  Serial.print(", Start direction: ");
  Serial.print(startDir);
  Serial.println("\n");
  
  
  // Assign all map values
  int it = 6;
  for (int i = 0; i < 5; i++)
  {
    for (int j = 0; j < 9; j++)
    {
      map1[i][j] = inputString[it] - '0';
      it++;  
    }
  }

  // Call path planning function with and without uncertain walls
  planPath(true);
  Serial.println("\n\n");
  planPath(false);
  Serial.println("\n\n");

  // Determine whether map has been fully explored
  if (uncertainSteps < certainSteps || uncertainSteps == certainSteps && uncertainTurns < certainTurns)
  {
    Serial.println("This map HAS NOT been adequately explored to find the shortest path");
  }
  else
  {
    Serial.println("This map HAS been adequately explored to find the shortest path");
  }
}



//void flipMap()
//{
//  int map2[9][5];
//  memset(map2, 8, sizeof(map2));
//
//  for (int i = 0; i < 9; i++)
//  {
//    for (int j = 0; j < 5; j++)
//    {
//      if (j < 5)
//      {
//        map2[i][j] = map1[j][i];
//      }
//      else if (i == 8 && j < 4)
//      {
//        map2[i][j] = 5;
//      }
//      else if (j == 4 && i < 8)
//      {
//        map2[i][j] = 7;
//      }
//      else if (j == 4 && i == 8)
//      {
//        map2[i][j] = 4;
//      }
//      else
//      {
//        map2[i][j] = 8;
//      }
//    }
//  }
//  map1 = map2;  
//}
/*
///////////////
//   SETUP   //
///////////////
void setup() 
{

  // Initialise serial for debugging
  Serial.begin(9600); 
  Serial.println("Beginning path planning algorithm\n");
  Serial1.begin(9600);
}


//////////////
//   LOOP   //
//////////////
void loop() 
{
  // Read from the Bluetooth module and send to the Arduino Serial Monitor
  if (Serial1.available())
    {
        BTstate = Serial1.read();
        Serial.write(BTstate);
        input += BTstate;
        if (BTstate == '\n') // Single quotes are crucial here
        {
          Serial.println();

          decodeInput(input);
          
          input = "";
          Serial1.write('o');
          Serial1.write('k');
          Serial1.write('\r');
          Serial1.write('\n');
        }
    }
 
 
    // Read from the Serial Monitor and send to the Bluetooth module
    if (Serial.available())
    {
        state = Serial.read();

        Serial1.write(state);
        // Echo the user input to the main window. 
        // If there is a new line print the ">" character.
        if (NL) 
        { 
          Serial.print("\r\n>");  
          NL = false; 
        }
        Serial.write(state);
        
        if (state==10) 
        { 
          NL = true; 
        }
    }
}
*/

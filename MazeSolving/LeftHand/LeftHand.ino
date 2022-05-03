


bool LeftSensor (){
  // Read IR sensor 1
  //TODO: complete this function
  return 0;
}

bool StraightSensor (){
  // Read the IR sensor 2
  //TODO: complete this function
  return 0;
}

bool RightSensor (){
  // Read the IR sensor 3
  //TODO: complete this function
  return 0;
}


void GoStraight () {
  // go straight
  //TODO: complete this function
}

void TurnLeft () {
  // turn left
  //TODO: complete this function
}

void TurnRight () {
  // turn right
  //TODO: complete this function
}

void UTurn () {
  // reverse the direction
   //TODO: complete this function
}

void Stop () {
  // Stop
  //TODO: complete this function
}

bool IsEnd(bool Left , bool Straight , bool Right){
  if ((Left) && (Straight) && (Right)){
    // all pathes are allowed so it can be the end of the the maze
    GoStraight();
    //TODO: Add delay here
    Stop();

    Left = LeftSensor();
    Straight = StraightSensor ();
    Right = RightSensor();

    if ((Left) && (Straight) && (Right)){
      //still all pathes are allowed so this is the end
      return true;
    }
    else{
      return false;
    }
  }
}

bool LSRB_SolveMaze(){

  bool Left = LeftSensor();
  bool Straight = StraightSensor ();
  bool Right = RightSensor();

  if (IsEnd(Left , Straight , Right)){
      Stop(); 
      return true;
  }

  if (Left){
    //Left is allowed
    TurnLeft();
  }

  else if (Straight){
    //Straight is allowed
    GoStraight();
  }

  else if (Right){
    //Right is allowed
    TurnRight();
  }

  else {
    //No pathes are allowed
    UTurn();
  }

}

void SetupIRs(){
  //Setup IR sensors Pins
  //TODO: complete this function
}

void SetupLSRB_Outputs(){
  //Setup LSRB outputs Pins
  //TODO: complete this function

}
void setup(){
  SetupIRs();
  SetupLSRB_Outputs();
}
void loop(){
  bool solved = LSRB_SolveMaze();
}





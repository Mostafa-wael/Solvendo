
const int IRSensorL = A1; //IR left sensor
const int IRSensorR = A2; //IR right sensor
const int IRSensorS = A3; //IR straight sensor

const int RightMotorPWM = 3;;
const int LeftMotorPWM = 9;
const int RForward = 5;
const int RightBachward = 4;
const int LForward = 7;
const int LeftBachward = 6;



const int minDif = 5;
const int thr = 700;
int turnSpeed = 50;
int straightSpeed = 200;


 void followLine(){
    //Reading sensors, if HIGH (BLACK Line) or LOW (No Line), if black line on Sensor Right, we go Right and contrary
    int valueLeft= analogRead(IRSensorL);
    int valueRight = analogRead(IRSensorR); 
    int valueStraight = analogRead(IRSensorS);
    if(valueLeft > thr && valueLeft-valueStraight > minDif){     
      go_Left();      
    } else if (valueRight > thr && valueRight-valueStraight > minDif){
      go_Right();      
    } else{
      go_Advance();
    }
}

void set_Motorspeed(int valR , int valL ){ 
  analogWrite(LeftMotorPWM, valL);
  analogWrite(RightMotorPWM, valR);
}

void go_Advance(){
    digitalWrite(RForward, HIGH);
    digitalWrite(LForward, HIGH);
    digitalWrite(RightBachward, LOW);
    digitalWrite(LeftBachward, LOW);
    set_Motorspeed(straightSpeed , straightSpeed);
}

void go_Right(){ 
    digitalWrite(RForward, HIGH);
    digitalWrite(LForward,LOW);
    digitalWrite(RightBachward, LOW);
    digitalWrite(LeftBachward, HIGH);
    set_Motorspeed(turnSpeed , turnSpeed);
}

void go_Left(){ 
    digitalWrite(RForward, LOW);
    digitalWrite(LForward,HIGH);
    digitalWrite(RightBachward, HIGH);
    digitalWrite(LeftBachward, LOW);
    set_Motorspeed(turnSpeed , turnSpeed);
}
  
void stop_Stop(){
    digitalWrite(RForward, LOW);
    digitalWrite(RightBachward, LOW);
    digitalWrite(LForward, LOW);
    digitalWrite(LeftBachward, LOW);
    set_Motorspeed(0,0);
}


void setup() {
    pinMode(RightBachward, OUTPUT);
    pinMode(RForward, OUTPUT);
    pinMode(LeftMotorPWM, OUTPUT);
    pinMode(LForward, OUTPUT);
    pinMode(LeftBachward, OUTPUT);
    pinMode(RightMotorPWM, OUTPUT);
    pinMode(IRSensorL, INPUT);
    pinMode(IRSensorR, INPUT);
    pinMode(IRSensorS, INPUT);
}

void loop() {
    followLine();
}

# include "drivers.h"
/*!
 * @brief Directions
 */
#define LEFT 1
#define CENTER 2
#define RIGHT 3

/*!
 * @brief IR Sensors
 */
#define IR_L A0
#define IR_C A1
#define IR_R A2
#define IR_LL A3
#define IR_RR A4
/*!
 * @brief SWITCH Pin
 */
//#define SWITCH 11
//#define MOTOR_L_B 7
//#define MOTOR_L_F 8
//#define MOTOR_R_F 4
//#define MOTOR_R_B 3
//#define MOTOR_R_SPEED 5
//#define MOTOR_L_SPEED 6
#define SWITCH 11
#define MOTOR_L_B 3
#define MOTOR_L_F 4
#define MOTOR_R_F 8
#define MOTOR_R_B 7
#define MOTOR_R_SPEED 6
#define MOTOR_L_SPEED 5

/*!
 * @brief Thresholds
 */
#define THRESHOLD_W 500 // above it is white and below it all black
#define THRESHOLD_B 150 // above it is white and below it all black
#define ThresholdDiff 200
#define ThresholdDiff_W 150
#define SPEEDSTEERING 70 // steering speed
#define NUM_OF_ERROR 70  // thershold for accepted errors
#define SPEEDFORWARD 95// forward speed

/*!
 * @brief PID Constants
 */
#define kp 0.05
#define ki 0.0000004 
#define kd .02 //1.5

/*!
 * @brief Global variables for the PID
 */
float eCurrent=0;
float ePrev=0;
long prevT = 0;
float eIntegral = 0;

/*!
 * @brief Gllobal variables to handle the direction errors
 */
int lastDir = 0;
int error = 0;
boolean dir = false;

void moveMotor2(int speedR , int speedL, int rDir, int lDir)
{
  //rDir *= -1;
  lDir *= -1;
  if(millis()%200 > 120){
    avr_analogWrite(MOTOR_R_SPEED, 0);
    avr_analogWrite(MOTOR_L_SPEED, 0);
    return;
  }
  avr_analogWrite(MOTOR_R_SPEED, speedR);
  avr_analogWrite(MOTOR_L_SPEED, speedL);
  if(rDir == 1){
    avr_digitalWrite(MOTOR_R_F,HIGH);
    avr_digitalWrite(MOTOR_R_B,LOW);
  }
  else{
    avr_digitalWrite(MOTOR_R_F,LOW);
    avr_digitalWrite(MOTOR_R_B,HIGH);
  }
  if(lDir == 1){
    avr_digitalWrite(MOTOR_L_F,HIGH);
    avr_digitalWrite(MOTOR_L_B,LOW);
  }
  else{
    avr_digitalWrite(MOTOR_L_F,LOW);
    avr_digitalWrite(MOTOR_L_B,HIGH);
  }
}

/*!
 * @brief the setup code
 * @return void
 */
void setup() {
  // set motor pins as output
  Serial.begin(9600);
  avr_pinMode(MOTOR_R_F, OUTPUT);
  avr_pinMode(MOTOR_R_B, OUTPUT);
  avr_pinMode(MOTOR_L_F, OUTPUT);
  avr_pinMode(MOTOR_L_B, OUTPUT);

  // set switch pin as input
  avr_pinMode(SWITCH, INPUT);
  // set motor direction
  avr_digitalWrite(MOTOR_R_F, HIGH);
  avr_digitalWrite(MOTOR_L_F, HIGH);
  avr_digitalWrite(MOTOR_R_B, LOW);
  avr_digitalWrite(MOTOR_L_B, LOW);
}

void loop() {
  /////////////////////////////////////////////////////
  movecar2();
 //moveMotor(SPEEDFORWARD, SPEEDFORWARD);
 //moveMotor2(SPEEDFORWARD,SPEEDFORWARD,1,-1);
  /////////////////////////////////////////////////////
}
int lIsBlack = 0;
int rIsBlack = 0;
int rrIsBlack = 0;
int llIsBlack = 0;

int lIsWhite = 0;
int rIsWhite = 0;
int cIsWhite = 0;
int llIsWhite = 0;
int rrIsWhite = 0;

int abbas = 0;

unsigned long lastRight = 0;

int onLine(){
  return (
    avr_analogRead(IR_R) > THRESHOLD_W
    && avr_analogRead(IR_C)< THRESHOLD_W
    && avr_analogRead(IR_L) > THRESHOLD_W
    );
//    return (
//      analogRead(IR_R) < THRESHOLD_W
//      || analogRead(IR_C)< THRESHOLD_W
//      || analogRead(IR_L) < THRESHOLD_W
//      );
}
int kam2 = 10000;
void rotateRight(){
  int mincnt = kam2;
  while(mincnt > 0 || !onLine()){
    moveMotor2(SPEEDFORWARD,SPEEDFORWARD,-1,1);
    mincnt--;
  }
}
void rotateLeft(){
  moveMotor2(0,0,1,1);
  moveMotor2(SPEEDFORWARD,SPEEDFORWARD,-1,-1);
  moveMotor2(0,0,1,1);
    int mincnt = kam2;
  while(mincnt > 0 || !onLine()){
    moveMotor2(SPEEDFORWARD,0,1,-1);
    mincnt--;
  }
}
void rotate180(){
   int mincnt = kam2;
  while(mincnt > 0 || !onLine()){
    moveMotor2(SPEEDFORWARD,SPEEDFORWARD,1,-1);
    mincnt--;
  }
}
void movecar2(){
  float LL_Reading = avr_analogRead(IR_LL);
  float RR_Reading = avr_analogRead(IR_RR);
  float C_Reading = avr_analogRead(IR_C);
  float L_Reading = avr_analogRead(IR_L);
  float R_Reading = avr_analogRead(IR_R);

  int C = C_Reading > THRESHOLD_W;
  int L = L_Reading > THRESHOLD_W;
  int R = R_Reading > THRESHOLD_W;
  int LL = LL_Reading > THRESHOLD_W;
  int RR = RR_Reading > THRESHOLD_W;

  if(!LL) llIsBlack++,llIsWhite = 0;
  else llIsBlack = 0,llIsWhite++;

  if(!RR) rrIsBlack++,rrIsWhite = 0;
  else rrIsBlack = 0,rrIsWhite++;
  
  if(!L) lIsBlack++,lIsWhite = 0;
  else lIsBlack = 0,lIsWhite++;

  if(!R) rIsBlack++,rIsWhite = 0;
  else rIsBlack = 0,rIsWhite++;

  if(!C) cIsWhite = 0;
  else cIsWhite++;

  int kam = 3;
  if(llIsBlack >= kam){
    rotateLeft();
  }
  else if(cIsWhite >=kam && rIsWhite >=kam  && lIsWhite >=kam ){
    if(millis()-lastRight < 1000)
      rotateRight();
    else
      rotate180();
  }
  
  if(rrIsBlack >= kam)
    lastRight = millis();
    
  movecar();
}

float differential_steering(float left_align,float c,float right_align) 
{
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;

  eCurrent = 0 -(left_align-right_align); 
  ePrev = (eCurrent-ePrev)/deltaT;
  eIntegral += eCurrent*deltaT;

  float delta_v = kp*eCurrent + ki*eIntegral + kd*ePrev;

  ePrev = eCurrent;
  prevT=currT;
  return delta_v;
}

void movecar(){
 // read the sensors
 float C = avr_analogRead(IR_C);
 float L = avr_analogRead(IR_L);
 float R = avr_analogRead(IR_R);

 // calc the steering angle usign the PID controller
 float PIDError = differential_steering(L, C, R);

 // check the switch pin to switch between code : 1- if conditions  2- pid with if conditions 
// if(digitalRead(SWITCH) == HIGH)
// {
  PIDError = 0.0;
// }
 // check if reached the end point
//  if(C<THRESHOLD_B && L < THRESHOLD_B && R< THRESHOLD_B){
//    moveMotor(0, 0);
//    }

  // check if e have drifted out of the line
  // explore the surroundings to return back and count your fault moves
  if(C>THRESHOLD_W && abs(L-R) < ThresholdDiff_W) 
  {
    if(error == NUM_OF_ERROR)
      dir = !dir;
    if(!dir)
    {
      moveMotor(0, SPEEDFORWARD);
      error ++;
    }
    else
    {
      moveMotor(SPEEDFORWARD,0);
      error --;
    }
     
  }
  // decide the direction based on the sensors readings and the velocity based on the PID controller
  else{
        if(L-R>ThresholdDiff)  //Left more in white than R 
        {
          moveMotor(0, SPEEDFORWARD+abs(PIDError));
        }
        else if(ThresholdDiff<R-L)  //Right more in white than L 
        { 
          moveMotor(SPEEDFORWARD+abs(PIDError), 0);
        }
        else // forward speed is faster than the steering speed
        { 
          moveMotor(SPEEDFORWARD, SPEEDFORWARD);
         }
    
   }
  }

  void moveMotor(int speedR , int speedL)
{
  moveMotor2(speedR, speedL, 1, 1);
//  analogWrite(MOTOR_R_SPEED, speedR);
//  analogWrite(MOTOR_L_SPEED, speedL);
}
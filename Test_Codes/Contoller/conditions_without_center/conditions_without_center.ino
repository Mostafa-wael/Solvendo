#define LEFT 1
#define CENTER 2
#define RIGHT 3
#define IR_L A0
#define IR_C A1
#define IR_R A2

#define MOTOR_L_F 8
#define MOTOR_L_B 7
#define MOTOR_R_F 4
#define MOTOR_R_B 3
#define MOTOR_R_SPEED 5
#define MOTOR_L_SPEED 6

#define THRESHOLD 700 //assuming that this value above it is white and below it all black
#define ThresholdDiff 400
#define ThresholdDiff_W 150
#define SPEED 70
#define NUM_OF_ERROR 70
#define SPEEDFORWARD 110


void stopCar(){
  analogWrite(MOTOR_R_SPEED, LOW);
  analogWrite(MOTOR_L_SPEED, LOW);
//  Serial.println("Stop");
}

void moveCarLeft()
{
  analogWrite(MOTOR_R_SPEED, SPEED);
  analogWrite(MOTOR_L_SPEED, LOW);
//  Serial.println("Left");
}

void moveCarRight()
{
  analogWrite(MOTOR_R_SPEED,LOW);
  analogWrite(MOTOR_L_SPEED, SPEED);
//  Serial.println("Right");
}

void moveForward(){

  analogWrite(MOTOR_R_SPEED,SPEEDFORWARD);
  analogWrite(MOTOR_L_SPEED, SPEEDFORWARD);
//  Serial.println("Forward");
  }
void moveBackward(){

  analogWrite(MOTOR_R_SPEED,SPEED);
  analogWrite(MOTOR_L_SPEED, SPEED);
//  Serial.println("Backward");
 }

 void printRealValues()
{
  Serial.print("Abs Values ");
  Serial.print(analogRead(IR_L));
  Serial.print(" ");
  Serial.print(analogRead(IR_C));
  Serial.print(" ");
  Serial.print(analogRead(IR_R));
  Serial.print(" ");
  Serial.print((analogRead(IR_L) - analogRead(IR_R)));
  Serial.print(" ");
  Serial.println((analogRead(IR_R) - analogRead(IR_L)));
}
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IR_C, INPUT);
  pinMode(IR_R, INPUT);
  pinMode(IR_L, INPUT);
  
  pinMode(MOTOR_R_F, OUTPUT);
  pinMode(MOTOR_R_B, OUTPUT);
  pinMode(MOTOR_L_F, OUTPUT);
  pinMode(MOTOR_L_B, OUTPUT);
  digitalWrite(MOTOR_R_F, HIGH);
  digitalWrite(MOTOR_L_F, HIGH);
  digitalWrite(MOTOR_R_B, LOW);
  digitalWrite(MOTOR_L_B, LOW);
}

int lastDir = 0;
int error = 0;
boolean dir = false;
void loop() {
// printRealValues();
 moveForward();
}

void movecar(){
 float C = analogRead(IR_C);
 float L = analogRead(IR_L);
 float R = analogRead(IR_R);

  if(C>THRESHOLD && abs(L-R)<ThresholdDiff_W)
  {
    Serial.println("error");
    if(error == NUM_OF_ERROR)
      dir = !dir;
    if(!dir)
    {
      moveCarLeft();
      error ++;
    }
    else
    {
      moveCarRight();
      error --;
    }
     
  }
  else{
        if(L-R>ThresholdDiff)  //Left more in white than R 
        {
          moveCarRight();
          lastDir = RIGHT;
        }
        else if(ThresholdDiff<R-L)  //Right more in white than L 
        { 
          moveCarLeft();
          lastDir = LEFT;
        }
        else{ 
          moveForward();
          lastDir = CENTER;
          }
    
   }
  }
  

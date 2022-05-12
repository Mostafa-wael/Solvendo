#define IR_L A0
#define IR_C A1
#define IR_R A2

#define LEFT 0
#define CENTER 1
#define RIGHT 2
#define STOP 3


#define MOTOR_L_F 8
#define MOTOR_L_B 9
#define MOTOR_R_F 10
#define MOTOR_R_B 11
#define MOTOR_L_SPEED 5
#define MOTOR_R_SPEED 6


#define THRESHOLD3 300
#define THRESHOLD 500 // Salama changed this
#define THRESHOLD2 50
#define SPEED 70


byte directions [1000];



bool blackLine(int IR_num){
  
  return analogRead(IR_num) < THRESHOLD;
}

void moveCar(){
  Serial.print("S");
  analogWrite(MOTOR_R_SPEED, SPEED);
  analogWrite(MOTOR_L_SPEED, SPEED);
}
void stopCar(){
  Serial.print(" ");
  analogWrite(MOTOR_R_SPEED, LOW);
  analogWrite(MOTOR_L_SPEED, LOW);
}
void moveCarLeft()
{
  analogWrite(MOTOR_R_SPEED, SPEED);
  analogWrite(MOTOR_L_SPEED, LOW);
  Serial.print("L");
}
void moveCarRight()
{
  analogWrite(MOTOR_R_SPEED, LOW);
  analogWrite(MOTOR_L_SPEED, SPEED);
  Serial.print("R");
}
void printRealValues()
{
  Serial.print("Abs Values ");
  Serial.print(analogRead(IR_L));
  Serial.print(" ");
  Serial.print(analogRead(IR_C));
  Serial.print(" ");
  Serial.println(analogRead(IR_R));
}

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_R_F, OUTPUT);
  pinMode(MOTOR_R_B, OUTPUT);
  pinMode(MOTOR_L_F, OUTPUT);
  pinMode(MOTOR_L_B, OUTPUT);
  digitalWrite(MOTOR_R_F, HIGH);
  digitalWrite(MOTOR_L_F, HIGH);
  digitalWrite(MOTOR_R_B, LOW);
  digitalWrite(MOTOR_L_B, LOW);
}
  #define SMAPLE 100
void loop() {
//  printRealValues();
  int right = 0;  int center=0; int left =0 ;
  for(int i = 0; i< SMAPLE; i++){
       if(blackLine(IR_C))
           if (blackLine(IR_R))
               right++;
           else if (blackLine(IR_L))
              left++;
           else
               center+=5;
       else if (blackLine(IR_R) && center < THRESHOLD3 )
            right++;
       else if (blackLine(IR_L) && center < THRESHOLD3)
             left++;
  }
  Serial.print("Right: "); Serial.println(right);
  Serial.print("Left: "); Serial.println(left);
  Serial.print("Center: "); Serial.println(center);
   if (abs(left-right) < SMAPLE/2)
    center += (left + right); 
  if(center>right && center>left){
    moveCar();
    Serial.println(" Straight: ");
  }
  else if (left > right && left > center){
    moveCarLeft();
    Serial.println(" left: ");
  }
  else if (right > left && right > center){
    moveCarRight();
    Serial.println("Right: ");
  }

}

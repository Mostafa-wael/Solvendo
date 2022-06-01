#define IR_L A0
#define IR_C A1
#define IR_R A2

#define LEFT 0
#define CENTER 1
#define RIGHT 2
#define STOP 3
#define SPEED 70

#define MOTOR_L_F 2
#define MOTOR_L_B 3
#define MOTOR_R_F 4
#define MOTOR_R_B 5
#define MOTOR_R_SPEED 9
#define MOTOR_L_SPEED 10


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
#define thresh 10

void loop() {
  int R = analogRead(IR_R);
  int L = analogRead(IR_L);
  int C = analogRead(IR_C);
  int L_C = L-C; float errL = 1/L_C;
  int R_C = R-C; float errR = 1/R_C;
  Serial.println("errL: "); Serial.println(errL);
  Serial.println("errR: "); Serial.println(errR);
  err_diff = errL-errR;
  Serial.println("err_diff: "); Serial.println(err_diff);
  if(abs(err_diff) > thresh){
    moveCar();
  }
  else if(err_L > err_R){
      moveCarRight();   
  }
  else {
      moveCarLeft();   
  }
  delay(30);
}

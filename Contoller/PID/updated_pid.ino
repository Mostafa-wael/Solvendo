#define IR_L A2
#define IR_C A1
#define IR_R A0

#define LEFT 0
#define CENTER 1
#define RIGHT 2
#define STOP 3


#define MOTOR_L_F 8
#define MOTOR_L_B 7
#define MOTOR_R_F 4
#define MOTOR_R_B 3
#define MOTOR_R_SPEED 5
#define MOTOR_L_SPEED 6

#define THRESHOLD 890
#define THRESHOLD2 50
#define SPEED 90

//#define kp 0.08
#define kp 0.05
#define ki 0.0 // 0.000001
#define kd 1.5 //1.5
//#define kd 1e-3
#define SMAPLE 200
float eCurrent=0;
float ePrev=0;
long prevT = 0;
float eIntegral = 0;

int minLeft = 73;
int maxLeft = 970;
int minCenter = 91;
int maxCenter = 961;
int minRight = 88;
int maxRight = 982;

int readIR(int IR_num)
{
  float actualRead = analogRead(IR_num);
  if (IR_num == IR_L)
  {
    minLeft = actualRead < minLeft? actualRead : minLeft;
    maxLeft = actualRead < maxLeft? maxLeft : actualRead;
    return 1000.0 * actualRead / (maxLeft - minLeft); 
  }
   else if (IR_num == IR_C)
  {
    minCenter = actualRead < minCenter? actualRead : minCenter;
    maxCenter = actualRead < maxCenter? maxCenter : actualRead;
    return  1000.0 * actualRead / (maxCenter - minCenter); 
  }
    else if (IR_num == IR_R)
  {
    minRight = actualRead < minRight? actualRead : minRight;
    maxRight = actualRead < maxRight? maxRight : actualRead;
    return  1000.0 * actualRead / (maxRight - minRight); 
  }
}


bool blackLine(int IR_num)
{
  return analogRead(IR_num) > THRESHOLD;
}

void printRealValues()
{
  Serial.print("Abs Values ");
  Serial.print(analogRead(IR_L));
  Serial.print(" ");
  Serial.print(analogRead(IR_C));
  Serial.print(" ");
  Serial.println(analogRead(IR_R));

 Serial.print( minLeft); Serial.print(" ");
 Serial.print(maxLeft);  Serial.print(" ");
 Serial.print(minCenter);  Serial.print(" ");
 Serial.print(maxCenter);  Serial.print(" ");
 Serial.print( minRight);  Serial.print(" ");
 Serial.print(maxRight);  Serial.println(" ");

  Serial.print("Normalized Values ");
  Serial.print(readIR(IR_L));
  Serial.print(" ");
  Serial.print(readIR(IR_C));
  Serial.print(" ");
  Serial.println(readIR(IR_R));
}
void moveCar(float steering_angle , float velocity){
if(steering_angle<-50){
    // assuming that the steering angle less than 0 will  move the car right
  Serial.print("R: ");
  Serial.println(velocity-steering_angle);
  Serial.print("L: ");
  Serial.println(velocity);

  
  if(velocity-steering_angle > 255)
    analogWrite(MOTOR_R_SPEED, 255);
  else
    analogWrite(MOTOR_R_SPEED, velocity-steering_angle);
    analogWrite(MOTOR_L_SPEED, velocity);
}
else if(steering_angle>50){
  Serial.print("R: ");
  Serial.println(velocity);
  Serial.print("L: ");
  Serial.println(velocity+steering_angle);

  if(velocity+steering_angle > 255)
    analogWrite(MOTOR_L_SPEED, 255);
  else
    analogWrite(MOTOR_L_SPEED, velocity+steering_angle);
  analogWrite(MOTOR_R_SPEED, velocity);
  
}
else{

  Serial.print("R: ");
  Serial.println(velocity);
  Serial.print("L: ");
  Serial.println(velocity);
  
  analogWrite(MOTOR_R_SPEED, velocity);
  analogWrite(MOTOR_L_SPEED, velocity);
}
}


//void simple_line_follower(){
//  
//       if(blackLine(IR_C))
//           if (blackLine(IR_R))
//               moveCarRight();
//           else if (blackLine(IR_L))
//               moveCarLeft();
//           else
//               moveCar();
//       else if (blackLine(IR_R))
//            moveCarRight();
//       else if (blackLine(IR_L))
//            moveCarLeft();
//       else
//          stopCar();
//
//}

float differential_steering(float l,float c,float r) 
{
//  printRealValues();
//  float left_align= (c-l);    
//  float right_align= (c-r);
  float left_align= l;    
  float right_align= r;


  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;

  // calculating error values
//  printRealValues();
  eCurrent = 0 -(left_align-right_align); 
  ePrev = (eCurrent-ePrev)/deltaT;
  // float ePrev = (eCurrent-ePrev);
  eIntegral += eCurrent*deltaT;

  float delta_v = kp*eCurrent+ki*eIntegral ;
//  float delta_v = kp*eCurrent + ki*eIntegral + kd*ePrev;

    //  Serial.print("eCurrent");
    //  Serial.println(eCurrent);
    //  Serial.print("eDriv");
    //  Serial.println(eDriv);
    //  Serial.print("eIntegral");
    //  Serial.println(eIntegral);
  // upadating fro next round
  ePrev = eCurrent;
  prevT=currT;
  return delta_v;
}

void setup() {
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
#define SMAPLE 100
#define velocity 90.0
#define velocity_turn 70.0
void loop() {
  printRealValues();
  int left = 0;
  int right = 0;
  int center = 0;
  float c = 0;
  for (int i = 0; i < SMAPLE; i++)
  {
    // put your main code here, to run repeatedly:
    float l = readIR(IR_L);
    float R = readIR(IR_R);
    float C = readIR(IR_C);
    c += differential_steering(l,C,R);
  }

  c /=SMAPLE;
    Serial.print("c: ");
    Serial.println(c);
    //-28:28
   if (c<-50){
         analogWrite(MOTOR_L_SPEED, velocity_turn);
        analogWrite(MOTOR_R_SPEED, velocity_turn-c/2);

      }
      else if(c>50){
     analogWrite(MOTOR_R_SPEED, velocity_turn);
    analogWrite(MOTOR_L_SPEED, velocity_turn+c/2);
      }
      else{
    analogWrite(MOTOR_R_SPEED, velocity);
    analogWrite(MOTOR_L_SPEED, velocity);
    }
  ////////////
  delay(200);
  analogWrite(MOTOR_R_SPEED, 0);
  analogWrite(MOTOR_L_SPEED, 0);
  delay(50);
  
//  Serial.print("Right: "); Serial.println(right);
//  Serial.print("Left: "); Serial.println(left);
//  Serial.print("Center: "); Serial.println(center);
//   if (abs(left-right) < SMAPLE/2)
//    center += (left + right)/2+SMAPLE/10; 
//  if(center>right && center>left){
//    analogWrite(MOTOR_R_SPEED, velocity);
//    analogWrite(MOTOR_L_SPEED, velocity);
//    Serial.println(" Center: ");
//  }
//  else if (left > right && left > center){

//        analogWrite(MOTOR_R_SPEED, 0);
//    analogWrite(MOTOR_L_SPEED, velocity);
//    Serial.println(" left: ");
//  }
//  else if (right > left && right > center){
//    analogWrite(MOTOR_L_SPEED, 0);
//        analogWrite(MOTOR_R_SPEED, velocity);
//    Serial.println("Right: ");
//  }
//  moveCar(c,velocity);
 
//  analogWrite(MOTOR_R_SPEED, velocity);
//  analogWrite(MOTOR_L_SPEED, velocity);
}

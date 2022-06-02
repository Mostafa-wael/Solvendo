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
/*!
 * @brief SWITCH Pin
 */
#define SWITCH 11
#define MOTOR_L_B 7
#define MOTOR_L_F 8
#define MOTOR_R_F 4
#define MOTOR_R_B 3
#define MOTOR_R_SPEED 5
#define MOTOR_L_SPEED 6

/*!
 * @brief Thresholds
 */
#define THRESHOLD_W 700 // above it is white and below it all black
#define THRESHOLD_B 150 // above it is white and below it all black
#define ThresholdDiff 200
#define ThresholdDiff_W 150
#define SPEEDSTEERING 70 // steering speed
#define NUM_OF_ERROR 70  // thershold for accepted errors
#define SPEEDFORWARD 70 // forward speed

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

/*!
 * @brief Move the motor according to the passed speed
 * @param speedR the speed of the right motor
 * @param speedL the speed of the left motor
 * @return void
 */
void moveMotor(int speedR , int speedL)
{
  analogWrite(MOTOR_R_SPEED, speedR);
  analogWrite(MOTOR_L_SPEED, speedL);
}
/*!
 * @brief print the real analog values of the IR sensors and the difference between the left and the right sensors
 * @return void
 */
 void printValues()
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

/*!
 * @brief The core of the PID conrtoller, it calculates the required steering angle to minimize the error of the car (the center sensor is on the black line)
 * @param left_align the value of the left sensor
 * @param right_align the value of the right sensor
 * @return float the steering angle i.e. the difference of the speed between the two motors
 */
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


/*!
 * @brief the setup code
 * @return void
 */
void setup() {
  // set motor pins as output
//  Serial.begin(9600);
  pinMode(MOTOR_R_F, OUTPUT);
  pinMode(MOTOR_R_B, OUTPUT);
  pinMode(MOTOR_L_F, OUTPUT);
  pinMode(MOTOR_L_B, OUTPUT);

  // set switch pin as input
  pinMode(SWITCH, INPUT);
  // set motor direction
  digitalWrite(MOTOR_R_F, HIGH);
  digitalWrite(MOTOR_L_F, HIGH);
  digitalWrite(MOTOR_R_B, LOW);
  digitalWrite(MOTOR_L_B, LOW);
}

void loop() {
 //printValues();
 movecar();
}

/*!
 * @brief The core of the car movement, it decides the movement direction using some strict conditions to ensure that the car is always on the line
 * and the PID controller for the velocity profile
 * @return void
 */
void movecar(){
 // read the sensors
 float C = analogRead(IR_C);
 float L = analogRead(IR_L);
 float R = analogRead(IR_R);

 // calc the steering angle usign the PID controller
 float PIDError = differential_steering(L, C, R);

 // check the switch pin to switch between code : 1- if conditions  2- pid with if conditions 
 if(digitalRead(SWITCH) == HIGH)
 {
  PIDError = 0.0;
 }
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
      moveMotor(0, SPEEDSTEERING);
      error ++;
    }
    else
    {
      moveMotor(SPEEDSTEERING,0);
      error --;
    }
     
  }
  // decide the direction based on the sensors readings and the velocity based on the PID controller
  else{
        if(L-R>ThresholdDiff)  //Left more in white than R 
        {
          moveMotor(0, SPEEDSTEERING+abs(PIDError));
        }
        else if(ThresholdDiff<R-L)  //Right more in white than L 
        { 
          moveMotor(SPEEDSTEERING+abs(PIDError), 0);
        }
        else // forward speed is faster than the steering speed
        { 
          moveMotor(SPEEDFORWARD, SPEEDFORWARD);
         }
    
   }
  }
  

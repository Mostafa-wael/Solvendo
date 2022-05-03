// Constants
#define kp 15
#define ki 100
#define kd 1e-2
#define REDUCTION_FACTOR 10.0
#define COUNT_TO_RPM 60/9053.328
#define VERBOSE 1

// Pins
#define ENCA 2
#define ENCB 3
#define PWM_PIN 9
#define IN1 5
#define IN2 6

// globals
long prevT = 0;
int countPrev = 0;
float vFilt = 0;
float vPrev = 0;
float eIntegral = 0;
float ePrev = 0;
float vTarget = 0;
// Use the "volatile" directive for variables used in an interrupt
volatile int count_i = 0;
volatile long prevT_i = 0;


void setup() {
  vTarget = 10;
  Serial.begin(115200);

  pinMode(PWM_PIN,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  pinMode(ENCA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), doEncoderA, CHANGE);
  
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCB), doEncoderB, CHANGE);
}
void loop() {
  // Set a target velocity
  while(Serial.available()>0)
    vTarget =  Serial.parseInt();

  moveMotor(calcPID(vTarget),PWM_PIN,IN1,IN2);

}
int readCount(){
    int count = 0;
    noInterrupts(); // disable interrupts temporarily while reading
    count = count_i;
    interrupts(); // turn interrupts back on
    return count;
}
float calcPID(float vTarget)
{
  int count = readCount();
  // Compute velocity
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity = (count - countPrev)/deltaT;
  countPrev = count;
  prevT = currT;

  // Convert count/s to RPM
  float vTemp = velocity*COUNT_TO_RPM;
  // Low-pass filter (25 Hz cutoff)
  vFilt = 0.854*vFilt + 0.0728*vTemp + 0.0728*vPrev;
  float vCurrent = vFilt;
  vPrev = vTemp;
  
  // Compute the control signal c
  float eCuurent = vTarget-vCurrent;
  float eDriv = (eCuurent-ePrev)/deltaT;
  eIntegral += eCuurent*deltaT;
  ePrev = eCuurent;
  float c = kp*eCuurent + ki*eIntegral + kd*eDriv;
  c = fabs(c) < 255? c: c/REDUCTION_FACTOR ;

  // print values
  #ifdef VERBOSE
  printValues(vCurrent, vTarget, c, eCuurent, eIntegral, eDriv);
  #endif
  
  return c;
}
void moveMotor(float c, int pinPWM, int dirForward, int dirBackward)
{
  // Set the motor speed and direction
  int dir = c<0? -1: 1;
  int pwr = (int) fabs(c);
  pwr = pwr > 255? 255:pwr;
  setMotor(dir,pwr,pinPWM,dirForward,dirBackward);
}
void printValues(float vCurrent, float vTarget, float c, float eCuurent, float eIntegral, float eDriv){
  
  Serial.print(vCurrent);
  Serial.print(" ");
  Serial.print(vTarget);
  Serial.print(" ");
  Serial.print(c);
  Serial.print(" ");
  Serial.print(eCuurent);
  Serial.print(" ");
  Serial.print(eIntegral);
  Serial.print(" ");
  Serial.print(eDriv);
  Serial.println();
}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void doEncoderA()
{  
  count_i += (digitalRead(ENCA)==digitalRead(ENCB))?1:-1;
}
void doEncoderB()
{  
  count_i += (digitalRead(ENCA)==digitalRead(ENCB))?-1:1;
}


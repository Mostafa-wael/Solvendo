// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.

// Pins
#define ENCA 2
#define ENCB 3
#define PWM 9
#define IN1 5
#define IN2 6

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile long prevT_i = 0;

float vFilt = 0;
float v1Prev = 0;
float eIntegral = 0;
float ePrev = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

   pinMode(ENCA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), doEncoderA, CHANGE);
  
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCB), doEncoderB, CHANGE);
  
  
  // attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder,RISING);
}

void loop() {

  // read the position and velocity
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/9053.328*60.0;
  // Low-pass filter (25 Hz cutoff)
  vFilt = 0.854*vFilt + 0.0728*v1 + 0.0728*v1Prev;
  float vCurrent = vFilt;
  v1Prev = v1;


  // Set a target
  float vTarget = 20*(sin(currT/2e5)>0);

  // Compute the control signal u
  float kp = 25;
  float ki = 100;
  float kd = 1e-3;
  float eCuurent = vTarget-vCurrent;
  float eDriv = (eCuurent-ePrev)/deltaT;
  eIntegral += eCuurent*deltaT;
  eIntegral = (int) fabs(vCurrent) < 2? 0: eIntegral;
  float c = kp*eCuurent + ki*eIntegral + kd*eDriv;

  ePrev = eCuurent;

  // Set the motor speed and direction
  int dir = 1;
  if (c<0){
    dir = -1;
  }
  int pwr = (int) fabs(c);
  pwr = pwr > 255? 255:pwr;
 

  setMotor(dir,pwr,PWM,IN1,IN2);

  
  Serial.print(vCurrent);
  Serial.print(" ");
  Serial.print(vTarget);
  Serial.print(" ");
  Serial.print(c);
  Serial.print(" ");
  Serial.print(pwr);
  Serial.print(" ");
  Serial.print(eCuurent);
  Serial.print(" ");
  Serial.print(eIntegral);
  Serial.print(" ");
  Serial.print(eDriv);
  Serial.println();
  delay(1);
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
  pos_i += (digitalRead(ENCA)==digitalRead(ENCB))?1:-1;
}
void doEncoderB()
{  
  pos_i += (digitalRead(ENCA)==digitalRead(ENCB))?-1:1;
}


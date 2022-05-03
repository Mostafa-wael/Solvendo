// Constants
#define COUNT_TO_RPM 60/600.0
#define kp 5
#define ki 10
// Pins
#define ENCA 2 // Encoder A
#define ENCB 3 // Encoder B 
#define PWM 5 // PWM
#define IN1 6  // Motor Direction forward
#define IN2 7 // Motor Direction backward

// globals
long prevT = 0;
int countPrev = 0;
float vFilt = 0;
float vPrev = 0;
float eintegral = 0;

// Use the "volatile" directive for variables used in an interrupt
volatile int count_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;


void setup() {
  Serial.begin(115200); // Start serial communication

  // Set pins as input and output
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  // attach interrupt to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
}

void loop() {

  // read the count and velocity
  int count = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  count = count_i;
  interrupts(); // turn interrupts back on

  // Compute velocity 
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity = (count - countPrev)/deltaT;
  countPrev = count;
  prevT = currT;

  // Convert count/s to RPM
  float v = velocity * COUNT_TO_RPM;

  // Low-pass filter (25 Hz cutoff)
  vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;
  vPrev = v;

  // Set a target velocity
  float vt = 100*(sin(currT/1e6)>0); // step sine wave

  // Compute the control signal u
  float e = vt-vFilt;
  eintegral += e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = u<0 ?-1:1;
 
  int pwr = (int) fabs(u);
  pwr = pwr > 255 ? 255 : pwr;
  setMotor(dir,pwr,PWM,IN1,IN2);

  // Print the data
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v);
  Serial.println();
  delay(1); // sampling rate
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

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  // If B is high, increment forward
  // Otherwise, increment backward
  int increment = b>0? 1:-1;
  count_i = count_i + increment;

}

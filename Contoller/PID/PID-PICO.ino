/* PINs Giude : 
Pins GP0 through GP29 are mapped to Arduino "pins" 0 through 29.
Arduino Analog pins 0 through 3 are mapped to pins GP26 through GP29.
*/

// Constants
#define kp 15
#define ki 100
#define kd 1e-2
#define REDUCTION_FACTOR 10.0
#define COUNT_TO_RPM 60/9053.328
#define VERBOSE 1

// Pins for motor 1
#define ENCA_M1 2
#define ENCB_M1 3
#define PWM_PIN_M1 0
#define IN1_M1 2
#define IN2_M1 3

// Pins for motor 2
#define ENCA_M2 2
#define ENCB_M2 3
#define PWM_PIN_M2 1
#define IN1_M2 4
#define IN2_M2 5


class motorContoller
{
    public:
        int ENCA;
        int ENCB;
        int PWM_PIN;
        int IN1;
        int IN2;
        motorContoller(int _ENCA, int _ENCB ,int _PWM_PIN,int _IN1,int _IN2)
        {
            ENCA = _ENCA;
            ENCB = _ENCB;
            PWM_PIN = _PWM_PIN;
            IN1 = _IN1;
            IN2 = _IN2;
            setupMotor();
        }
        void move(float vTarget, int count_i)
        {
            moveMotor(calcPID(vTarget, count_i),PWM_PIN,IN1,IN2);
        }
    private:
        // globals
        long prevT = 0;
        int countPrev = 0;
        float vFilt = 0;
        float vPrev = 0;
        float eIntegral = 0;
        float ePrev = 0;
        //float vTarget_M1 = 10;
        // Use the "volatile" directive for variables used in an interrupt
        volatile long prevT_i = 0;
        void setupMotor()
        {
            pinMode(PWM_PIN,OUTPUT);
            pinMode(IN1,OUTPUT);
            pinMode(IN2,OUTPUT);
        }
        int readCount(int count_i)
        {
            int count = 0;
            noInterrupts(); // disable interrupts temporarily while reading
            count = count_i;
            interrupts(); // turn interrupts back on
            return count;
        }
        float calcPID(float vTarget_M1, int count_i)
        {
            int count = readCount(count_i);
            // Compute velocity
            long currT = micros();
            float deltaT = ((float) (currT-prevT))/1.0e6;
            float velocity = (count - countPrev)/deltaT;
            countPrev = count;
            prevT = currT;

            // Convert count/s to RPM
            float vCurrent = velocity*COUNT_TO_RPM;
            // Low-pass filter (25 Hz cutoff)
            vFilt = 0.854*vFilt + 0.0728*(vCurrent + vPrev);
            vCurrent = vFilt;
            vPrev = vCurrent;
            
            // Compute the control signal c
            float eCuurent = vTarget_M1-vCurrent;
            float eDriv = (eCuurent-ePrev)/deltaT;
            eIntegral += eCuurent*deltaT;
            ePrev = eCuurent;
            float c = kp*eCuurent + ki*eIntegral + kd*eDriv;
            c = fabs(c) < 255? c: c/REDUCTION_FACTOR ;

            // print values
            #ifdef VERBOSE
            printValues(vCurrent, vTarget_M1, c, eCuurent, eIntegral, eDriv);
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
        void printValues(float vCurrent, float vTarget_M1, float c, float eCuurent, float eIntegral, float eDriv)
        {
            Serial.print(vCurrent);
            Serial.print(" ");
            Serial.print(vTarget_M1);
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
        void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
        {
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
};



float  vTarget = 0;
volatile int count_i = 0;
motorContoller M1 (ENCA_M1,ENCB_M1 ,PWM_PIN_M1,IN1_M1,IN2_M1);
motorContoller M2 (ENCA_M2,ENCB_M2 ,PWM_PIN_M2,IN1_M2,IN2_M2);
void setup() {
    pinMode(ENCA_M1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCA_M1), doEncoderA, CHANGE);
    pinMode(ENCB_M1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCB_M1), doEncoderB, CHANGE);
    vTarget = 10.0;
    Serial.begin(115200);
  
}
void loop() {
  // Set a target velocity
  while(Serial.available()>0)
    vTarget =  Serial.parseInt();
    M1.move(vTarget,count_i);
  // moveMotor(calcPID(vTarget_M1),PWM_PIN_M1,IN1_M1,IN2_M1);

}
void doEncoderA()
{  
    count_i += (digitalRead(ENCA_M1)==digitalRead(ENCB_M1))?1:-1;
}
void doEncoderB()
{  
    count_i += (digitalRead(ENCA_M1)==digitalRead(ENCB_M1))?-1:1;
}

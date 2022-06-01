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
#define SPEEDFORWARD 90  // forward speed

/*!
 * @brief PID Constants
 */
#define kp 0.05
#define ki 0.0000004
#define kd .02 // 1.5

/*!
 * @brief Global variables for the PID
 */
float eCurrent = 0;
float ePrev = 0;
long prevT = 0;
float eIntegral = 0;

/*!
 * @brief Gllobal variables to handle the direction errors
 */
int lastDir = 0;
int error = 0;
boolean dir = false;

void moveMotor2(int speedR, int speedL, int rDir, int lDir)
{
    analogWrite(MOTOR_R_SPEED, speedR);
    analogWrite(MOTOR_L_SPEED, speedL);
    if (rDir == 1)
    {
        digitalWrite(MOTOR_R_F, HIGH);
        digitalWrite(MOTOR_R_B, LOW);
    }
    else
    {
        digitalWrite(MOTOR_R_F, LOW);
        digitalWrite(MOTOR_R_B, HIGH);
    }
    if (lDir == 1)
    {
        digitalWrite(MOTOR_L_F, HIGH);
        digitalWrite(MOTOR_L_B, LOW);
    }
    else
    {
        digitalWrite(MOTOR_L_F, LOW);
        digitalWrite(MOTOR_L_B, HIGH);
    }
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
 * @brief the setup code
 * @return void
 */
void setup()
{
    // set motor pins as output
    Serial.begin(9600);
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
    Serial.println("-------");
    Serial.println("-------");
}

void loop()
{
    // printValues();
    movecar2();
}
int lIsBlack = 0;
int rIsBlack = 0;

int lIsWhite = 0;
int rIsWhite = 0;
int cIsWhite = 0;
int abbas = 0;

unsigned long lastRight = 0;

int kam2 = 1000000;
void rotateRight()
{
    printValues();
    Serial.println("Rotate Right");
    moveMotor2(0, 0, 1, 1);
    delay(2000);
    int mincnt = kam2;
    while (mincnt > 0 || !(
                             analogRead(IR_R) > THRESHOLD_W && analogRead(IR_C) < THRESHOLD_W && analogRead(IR_L) > THRESHOLD_W))
    {
        moveMotor2(0, SPEEDFORWARD, 1, 1);
        mincnt--;
    }
    moveMotor2(0, 0, 1, 1);
    delay(2000);
}
void rotateLeft()
{
    printValues();
    Serial.println("Rotate Left");
    moveMotor2(0, 0, 1, 1);
    delay(2000);
    int mincnt = kam2;
    while (mincnt > 0 || !(
                             analogRead(IR_R) > THRESHOLD_W && analogRead(IR_C) < THRESHOLD_W && analogRead(IR_L) > THRESHOLD_W))
    {
        moveMotor2(SPEEDFORWARD, 0, 1, 1);
        mincnt--;
    }
    moveMotor2(0, 0, 1, 1);
    delay(2000);
}
void rotate180()
{
    printValues();
    Serial.println("Rotate 180");
    moveMotor2(0, 0, 1, 1);
    delay(2000);
    int mincnt = kam2;
    int periodToTakeBreak = 100;
    unsigned long lastBreak = millis();
    while (mincnt > 0 || !(
                             analogRead(IR_R) > THRESHOLD_W && analogRead(IR_C) < THRESHOLD_W && analogRead(IR_L) > THRESHOLD_W))
    {
        moveMotor2(SPEEDFORWARD, SPEEDFORWARD, 1, -1);
        if (millis() - lastBreak > periodToTakeBreak)
        {
            moveMotor2(0, 0, 1, -1);
            delay(100);
            moveMotor2(SPEEDFORWARD, SPEEDFORWARD, 1, -1);
        }
        mincnt--;
    }
    moveMotor2(0, 0, 1, 1);
    delay(2000);
}
void movecar2()
{
    float C_Reading = analogRead(IR_C);
    float L_Reading = analogRead(IR_L);
    float R_Reading = analogRead(IR_R);

    int C = C_Reading > THRESHOLD_W;
    int L = L_Reading > THRESHOLD_W;
    int R = R_Reading > THRESHOLD_W;

    if (!L)
        lIsBlack++, lIsWhite = 0;
    else
        lIsBlack = 0, lIsWhite++;

    if (!R)
        rIsBlack++, rIsWhite = 0;
    else
        rIsBlack = 0, rIsWhite++;

    if (!C)
        cIsWhite = 0;
    else
        cIsWhite++;

    //  if(rIsBlack >= 1 || lIsBlack >= 1 ){
    //    Serial.print(lIsBlack);
    //    Serial.print(" ");
    //    Serial.println(rIsBlack);
    //  }
    int kam = 130;
    if (lIsBlack >= kam)
    {
        rotateLeft();
    }
    else if (cIsWhite >= kam && rIsWhite >= kam && lIsWhite >= kam)
    {
        // TODO: check for last right
        //    Serial.print(lastRight);
        //    Serial.print(" ");
        //    Serial.println(millis()-lastRight);
        moveMotor(0, 0);
        if (millis() - lastRight < 2000)
            rotateRight();
        else
            rotate180();
    }
    if (rIsBlack >= kam)
    {
        lastRight = millis();
        // rotateRight();
    }

    // moveMotor(SPEEDFORWARD,SPEEDFORWARD);
    movecar();
    //  else if (rIsBlack >= 3){
    //    moveMotor(SPEEDFORWARD, 0);
    //  } else if (lIsBlack >=3){
    //    moveMotor(0, SPEEDFORWARD);
    //  }
    //  else
    // moveMotor(SPEEDFORWARD, SPEEDFORWARD);
}

float differential_steering(float left_align, float c, float right_align)
{
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;

    eCurrent = 0 - (left_align - right_align);
    ePrev = (eCurrent - ePrev) / deltaT;
    eIntegral += eCurrent * deltaT;

    float delta_v = kp * eCurrent + ki * eIntegral + kd * ePrev;

    ePrev = eCurrent;
    prevT = currT;
    return delta_v;
}

void movecar()
{
    // read the sensors
    float C = analogRead(IR_C);
    float L = analogRead(IR_L);
    float R = analogRead(IR_R);

    // calc the steering angle usign the PID controller
    float PIDError = differential_steering(L, C, R);

    // check the switch pin to switch between code : 1- if conditions  2- pid with if conditions
    if (digitalRead(SWITCH) == HIGH)
    {
        PIDError = 0.0;
    }
    // check if reached the end point
    //  if(C<THRESHOLD_B && L < THRESHOLD_B && R< THRESHOLD_B){
    //    moveMotor(0, 0);
    //    }

    // check if e have drifted out of the line
    // explore the surroundings to return back and count your fault moves
    if (C > THRESHOLD_W && abs(L - R) < ThresholdDiff_W)
    {
        if (error == NUM_OF_ERROR)
            dir = !dir;
        if (!dir)
        {
            moveMotor(0, SPEEDSTEERING);
            error++;
        }
        else
        {
            moveMotor(SPEEDSTEERING, 0);
            error--;
        }
    }
    // decide the direction based on the sensors readings and the velocity based on the PID controller
    else
    {
        if (L - R > ThresholdDiff) // Left more in white than R
        {
            moveMotor(0, SPEEDSTEERING + abs(PIDError));
        }
        else if (ThresholdDiff < R - L) // Right more in white than L
        {
            moveMotor(SPEEDSTEERING + abs(PIDError), 0);
        }
        else // forward speed is faster than the steering speed
        {
            moveMotor(SPEEDFORWARD, SPEEDFORWARD);
        }
    }
}

void moveMotor(int speedR, int speedL)
{
    analogWrite(MOTOR_R_SPEED, speedR);
    analogWrite(MOTOR_L_SPEED, speedL);
}
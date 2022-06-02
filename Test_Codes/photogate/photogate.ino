int Motor_bin = 5;
int VCC_bin = 6;
void setup()
{
   pinMode(Motor_bin, OUTPUT);
   pinMode(VCC_bin, OUTPUT);
   int wheel = 0;
   int pass = 0;
   Serial.begin(9600);
}

float c = 0;
int i =0;
void loop() {
//  if(i>255)
//    i =0;
// i+=5;
 analogWrite(Motor_bin, 100);
 digitalWrite(VCC_bin, HIGH);
 delayMicroseconds(250);
 int c1 = analogRead(A2);
 digitalWrite(VCC_bin, LOW);
 delayMicroseconds(250);
 c1 -= analogRead(A2);

// delay(5);

 digitalWrite(VCC_bin, HIGH);
 delayMicroseconds(250);
 int c2 = analogRead(A2);
 digitalWrite(VCC_bin, LOW);
 delayMicroseconds(250);
 c2 -= analogRead(A2);

c = 0.854*c + 0.0728*(c2 + c1); // low pass filter 25 Hz
//int c = ((c1 + c2) / 2) - 80;
//Serial.print(c1);
//Serial.print(" ");
//Serial.print(c2);
//Serial.print(" ");
Serial.println(c);

// if (c > 360)
//  Serial.println("pass");
// else
//   Serial.println("wheel");
//Serial.print(pass);
//Serial.print(" ");
//Serial.println(wheel);
}



// void setup() {
//   // put your setup code here, to run once:
//   pinMode(in1, OUTPUT);
// }
// int pwmValue = 0;
// void loop() {
//   // put your main code here, to run repeatedly:
  
// //  while(Serial.available()>0){
// //    pwmValue =  Serial.parseInt();
// //  }
//   analogWrite(in1, 255);
// }

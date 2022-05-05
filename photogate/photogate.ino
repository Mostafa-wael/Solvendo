void setup()
{
int wheel = 0;
int pass = 0;
  wheel = 0; pass = 0;
 Serial.begin(9600);
}
float c = 0;
void loop() {
 digitalWrite(6, HIGH);
 delayMicroseconds(250);
 int c1 = analogRead(A2);
 digitalWrite(6, LOW);
 delayMicroseconds(250);
 c1 -= analogRead(A2);

 delay(5);

 digitalWrite(6, HIGH);
 delayMicroseconds(250);
 int c2 = analogRead(A2);
 digitalWrite(6, LOW);
 delayMicroseconds(250);
 c2 -= analogRead(A2);

c = 0.854*c + 0.0728*(c2 + c1); // low pass filter 25 Hz
//int c = ((c1 + c2) / 2) - 80;
//Serial.print(c1);
//Serial.print(" ");
//Serial.print(c2);
//Serial.print(" ");
//Serial.println(c);

 if (c > 280)
   pass++;
 else
   wheel++;
Serial.print(pass);
Serial.print(" ");
Serial.println(wheel);
}

// int in1 = 16;

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
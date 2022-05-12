#define photoPin A5
#define vccPIN 6
float c = 0;

void setup()
{
  Serial.begin(9600);
}
void loop() {
  digitalWrite(vccPIN, HIGH);
  delayMicroseconds(250);
  int c1 = analogRead(photoPin);
  digitalWrite(vccPIN, LOW);
  delayMicroseconds(250);
  c1 -= analogRead(photoPin);

  delay(5);

  digitalWrite(vccPIN, HIGH);
  delayMicroseconds(250);
  int c2 = analogRead(photoPin);
  digitalWrite(vccPIN, LOW);
  delayMicroseconds(250);
  c2 -= analogRead(photoPin);

  c = 0.854 * c + 0.0728 * (c2 + c1);
  //int c = ((c1 + c2) / 2);
  //Serial.print(c1);
  //Serial.print(" ");
  //Serial.print(c2);
  //Serial.print(" ");
  Serial.println(c);

    if (c < 150)
      Serial.println("|||||||||||||||||||||||||||||\wheel ");
   
}

//int in1 = 16   //pico
//int in1 = 6;


//void setup() {
//  // put your setup code here, to run once:
//  pinMode(in1, OUTPUT);
//}
//int pwmValue = 0;
//void loop() {
//  // put your main code here, to run repeatedly:
//
////  while(Serial.available()>0){
////    pwmValue =  Serial.parseInt();
////  }
//  analogWrite(in1, 50);
//}

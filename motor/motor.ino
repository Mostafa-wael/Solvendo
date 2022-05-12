int in1 = 10;
int in2 = 11;

void setup() {
  // put your setup code here, to run once:
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}
int pwmValue = 0;
void loop() {
  // put your main code here, to run repeatedly:
  
  while(Serial.available()>0){
    pwmValue =  Serial.parseInt();
  }
  analogWrite(in1, pwmValue);
}

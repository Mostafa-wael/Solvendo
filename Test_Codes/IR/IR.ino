/*
Average two reading for the IR after removing the noise from each reading

Connections:
VCC             VCC
 |               |
 R              R -- Vin
 |              |
---------------------
| Anode   Collector |
| Blue   Black      |
| cathode  Emitter  |
---------------------
 |               |
Gnd             Gnd
*/
int a,b, c1, c2;
void setup() 
{
Serial.begin(9600);
pinMode(6,OUTPUT);
}

void loop() {
 digitalWrite(6,HIGH);    // Turning ON LED
 delayMicroseconds(500);  //wait
 c1=analogRead(A2);        //take reading from photodiode(pin A3) :noise+signal
 digitalWrite(6,LOW);     //turn Off LED
 delayMicroseconds(500);  //wait
 c1-=analogRead(A2);        // again take reading from photodiode :noise
 
delay(10);

 digitalWrite(6,HIGH);    // Turning ON LED
 delayMicroseconds(500);  //wait
 c2=analogRead(A2);        //take reading from photodiode(pin A3) :noise+signal
 digitalWrite(6,LOW);     //turn Off LED
 delayMicroseconds(500);  //wait
 c2-=analogRead(A2);        // again take reading from photodiode :noise

int c = (c1+c2)/2.0;



Serial.print(c1);        
Serial.print("\t");
Serial.print(c2);        
Serial.print("\t");
Serial.println(c);         
if(c > 500)
  Serial.println("Black");
else
  Serial.println("White");
}
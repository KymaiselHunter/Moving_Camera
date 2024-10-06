#include <Servo.h>
Servo Servo1;
//
int servoPin = 9;
//int potPin = A0;

int angle = 0;

void setup() {
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  Servo1.attach(servoPin);
  Servo1.write(angle);
  
}

void loop() {
  // put your main code here, to run repeatedly:
    if(Serial.available()>0)
    {
      String msg = Serial.readString();
      angle = msg.toInt();

      //Serial.println("100");
      Servo1.write(angle);    
    }
    
}

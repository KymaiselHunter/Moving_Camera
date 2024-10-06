#include <Servo.h>
Servo Servo1;
//
int servoPin = 3;
//int potPin = A0;

int angle = 90;

#include <Stepper.h>
Stepper stepper1(2048,8,10,9,11);

int stepper_speed = 15;


void setup() {
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  Servo1.attach(servoPin);
  Servo1.write(angle);

  stepper1.setSpeed(stepper_speed);
  stepper1.step(1024);
  
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
    stepper1.step(2);
}

#include <Servo.h>
Servo Servo1;
//
int servoPin = 3;
//int potPin = A0;

int angle = 90;

#include <Stepper.h>
Stepper stepper1(2048,8,10,9,11);

int stepper_speed = 10;
int stepper_direction = 0;

void setup() {
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  Servo1.attach(servoPin);
  Servo1.write(angle);

  stepper1.setSpeed(stepper_speed);
  //stepper1.step(1024);
  
}

void loop() {
  // put your main code here, to run repeatedly:
    if(Serial.available()>0)
    {
      // Read the incoming string from Serial
      String msg = Serial.readStringUntil('\n');  // Read until newline character
      String strAngle = msg.substring(0, msg.indexOf(' '));
      angle = strAngle.toInt();

      String dirStr = msg.substring(msg.indexOf(' ') + 1);
      stepper_direction = dirStr.toInt();

      //Serial.println("100");
      Servo1.write(angle);

      stepper1.step(stepper_direction);
    }
//    if(stepper_direction > 0)
//      stepper1.step(-2);
//    else if (stepper_direction < 0)
//      stepper1.step(2);
}

#include <Servo.h>
Servo Servo1;
Servo Servo2;
//
int servoPin1 = 3;
//int potPin = A0;
int servoPin2 = 5;


int angle1 = 90;
int angle2 = 90;

//#include <Stepper.h>
//Stepper stepper1(2048,8,10,9,11);
//
//int stepper_speed = 10;
//int stepper_direction = 0;

void setup() {
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  Servo1.attach(servoPin1);
  Servo1.write(angle1);

  Servo2.attach(servoPin2);
  Servo2.write(angle2);

//  stepper1.setSpeed(stepper_speed);
//  //stepper1.step(1024);
  
}

void loop() {
  // put your main code here, to run repeatedly:
    if(Serial.available()>0)
    {
      // Read the incoming string from Serial
      String msg = Serial.readStringUntil('\n');  // Read until newline character
      
      String strAngle = msg.substring(0, msg.indexOf(' '));
      angle1 += strAngle.toInt();

      strAngle = msg.substring(msg.indexOf(' ') + 1);
      angle2 += strAngle.toInt();

      //Serial.println("100");
      Servo1.write(angle1);
      Servo2.write(angle2);
  
//      stepper1.step(stepper_direction);
    }
//    if(stepper_direction > 0)
//      stepper1.step(-2);
//    else if (stepper_direction < 0)
//      stepper1.step(2);
}
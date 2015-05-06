#include <Servo.h>

Servo joint1Servo;
int joint1Position = 0;
Servo joint2Servo;
int joint2Position = 0;

void setup() {
  // put your setup code here, to run once:
  joint1Servo.attach(6);
  joint2Servo.attach(5);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Joint 1 motor test
//  analogWrite(5, 0);
//  analogWrite(6, 210);
  //  delay(3000);
  //  analogWrite(6, 0);
  //  analogWrite(5, 255);
  //  delay(3000);

  // Joint 2 motor test
  //  analogWrite(5, 0);
  //  analogWrite(6, 0);
  //  for (joint2Position = 0; joint2Position < 180 ; joint2Position++)
  //  {
  //    Serial.println(joint2Position);
  //    joint2Servo.write(joint2Position);
  //    delay(15);
  //  }
  //  for (joint2Position = 180; joint2Position > 0 ; joint2Position--)
  //  {
  //    Serial.println(joint2Position);
  //    joint2Servo.write(joint2Position);
  //    delay(15);
  //  }

  // Both joints test
  joint1Servo.write(180);
  for (joint2Position = 0; joint2Position < 180 ; joint2Position++)
  {
    Serial.println(joint2Position);
    joint2Servo.write(joint2Position);
    delay(15);
  }

  joint1Servo.write(-180);
  for (joint2Position = 180; joint2Position > 0 ; joint2Position--)
  {
    Serial.println(joint2Position);
    joint2Servo.write(joint2Position);
    delay(15);
  }
}

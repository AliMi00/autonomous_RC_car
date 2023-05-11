#include <Servo.h>


Servo myservo;  // create servo object to control a servo


void setup() {
  // put your setup code here, to run once:
    myservo.attach(6);
      // attaches the servo on pin 9 to the servo object


}

void loop() {
  // put your main code here, to run repeatedly:

    myservo.write(90);                 // sets the servo position according to the scaled value


}

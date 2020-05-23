/*
 * DC motor and servo motor code designed to work with a 
 * potentiometer for testing purposes. Later we will give
 * the digital angle to the motors with a gps algorithm
*/

#include <Servo.h>

Servo myServo;
Servo ESC;

int const potPin = A3;
int potVal;
int angle;

void setup() {
 myServo.attach(5);
 ESC.attach(9,1000,2000);
 Serial.begin(9600);
}
void loop() {
 potVal = analogRead(potPin);
 Serial.print("potVal: ");
 Serial.print(potVal);
 angle = map(potVal, 0, 1023, 0, 180);
 Serial.print(", angle: ");
 Serial.println(angle);
 myServo.write(angle);
 ESC.write(angle); 
 delay(15);
}

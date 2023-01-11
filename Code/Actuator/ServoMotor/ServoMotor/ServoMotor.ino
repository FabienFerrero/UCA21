/*
 Controlling a servo position 
Connect :
- brown cable to GND
- Red Cable to V_bus (5V from USB)
- Orange to A2
 
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

void setup() {
  myservo.attach(A2);  // attaches the servo on pin A2 to the servo object
}

void loop() {

  for (int i=0; i<=180; i=i+5){
  myservo.write(i);                  // sets the servo position according to the scaled value
  delay(100);                           // waits for the servo to get there
  }
}

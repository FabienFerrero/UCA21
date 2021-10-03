#include <Arduino.h>
#include "SHTC3.h"

SHTC3 s(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  s.begin(true);

}


void loop() {

    s.sample();
    Serial.print(F("[SHTC3] T:"));
    Serial.print(s.readTempC());
    Serial.print(F(" Cº  /   H: "));
    Serial.print(s.readHumidity());
    Serial.println(F(" %"));
    delay(2000);
}
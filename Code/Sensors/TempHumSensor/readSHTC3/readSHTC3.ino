#include <Wire.h>
#include "SHTSensor.h" // http://librarymanager/All#arduino_shtc3
#include <FastLED.h> // http://librarymanager/All#FASTLED

#define LED_PIN     4
#define NUM_LEDS    21
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
CRGBPalette16 currentPalette;
TBlendType    currentBlending;

#define UPDATES_PER_SECOND 100

SHTSensor sht;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sht.init();
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x

}


void loop() {

     sht.readSample();
    Serial.print(F("[SHTC3] T:"));
    Serial.print(sht.getTemperature());
    Serial.print(F(" Cº  /   H: "));
    Serial.print(sht.getHumidity());
    Serial.println(F(" %"));


uint8_t temp = map(sht.getTemperature(), 20,35,170,0); // Map value from luminosity sensor to LED

 // FastLED's built-in rainbow generator
  fill_solid( leds, NUM_LEDS, ColorFromPalette(RainbowColors_p,temp,BRIGHTNESS, LINEARBLEND));


  
  
// send the 'leds' array out to the actual LED strip
  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);
    
    delay(2000);
}

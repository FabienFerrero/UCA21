#include <Arduino.h>
#include "SHTC3.h"
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

SHTC3 s(Wire);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  s.begin(true);

  Serial.println("Temperature:,Humidity:");   // Plot labels

}


void loop() {

    s.sample();
   // Serial.print(F("[SHTC3] T:"));
    Serial.print(s.readTempC());
    Serial.print(F(" "));
    Serial.println(s.readHumidity());
    


uint8_t temp = map(s.readTempC(), 20,35,170,0); // Map value from luminosity sensor to LED

 // FastLED's built-in rainbow generator
  fill_solid( leds, NUM_LEDS, ColorFromPalette(RainbowColors_p,temp,BRIGHTNESS, LINEARBLEND));


  
  
// send the 'leds' array out to the actual LED strip
  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);
    
    delay(1000);
}

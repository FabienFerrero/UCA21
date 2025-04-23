/*
  Read PIR

  PIR is connected on A3 (OUT) and A2 (+)

  The LED will turn RED when OUT is triggered
  The LED are green when no presence is detected

 
*/


#include <Wire.h>

// LED control
#include <FastLED.h>
#define LED_PIN     4
#define NUM_LEDS    21
#define BRIGHTNESS  32
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100
CRGB leds[NUM_LEDS];
CRGBPalette16 currentPalette;
TBlendType    currentBlending;
extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;
 float saturation = 1; // Between 0 and 1 (0 = gray, 1 = full color)
 float brightness = .05; // Between 0 and 1 (0 = dark, 1 is full brightness)

  
 void setColor(int redValue,  int blueValue, int greenValue) {
fill_solid( leds, NUM_LEDS, CRGB(greenValue,redValue,blueValue));
FastLED.show();
}

void setup() {
    Serial.begin(115200);
    pinMode(A2, OUTPUT);
    pinMode(A3, INPUT);

    digitalWrite(A2, HIGH);  // turn the PIR (HIGH is the voltage level)  

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    
  
}

void loop() {

    if (digitalRead(A3)) {   // read the PIR value
        setColor(0, 0, 255);   // Turn the LED to RED
    }
    else {
        setColor(255, 0, 0);   // Turn the LED to Green
    }
    
    delay(100);
}



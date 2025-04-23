/******************************************************************************
Ultra-sound HC-SR04 + FastLED Arduino
For UCA21
Fabien Ferrero
Aug, 2021

Distributed as-is; no warranty is given.

Use HC-SR04
Trig is connected on A2
Echo is connected on A3

Objective : This code will :
    * Measure the distance with HC-SR04
    * use the LED to illustrate the distance (RED is close, blue is far)
******************************************************************************/

// Your sketch must #include this library, and the Wire library
// (Wire is a standard library included with Arduino):

#include <FastLED.h> // http://librarymanager/All#FASTLED

#define LED_PIN     4
#define NUM_LEDS    21
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
CRGBPalette16 currentPalette;
TBlendType    currentBlending;



const int trigPin = A3;  
const int echoPin = A2;
float duration, distance;   


void setup() {
  delay( 1000 ); // power-up safety delay
  // Initialize the Serial port:
  
  Serial.begin(115200);
  Serial.println("HC-SR04 example sketch");

  // Initialize the HC-SR04 pin
  
  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT); 


// Setup LED

 FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
 FastLED.setBrightness(  BRIGHTNESS );
    
 currentPalette = RainbowColors_p;
 currentBlending = LINEARBLEND;

}

void loop() {
 
  digitalWrite(trigPin, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trigPin, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trigPin, LOW); 

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;   // The speed of sound is approximately 340 meters per second, but since the pulseIn() function returns the time in microseconds
  Serial.println(distance);
  uint8_t dist_temp = map(distance,0,200,0,255); // Map value from distance sensor to LED

  // FastLED's built-in rainbow generator
  fill_solid( leds, NUM_LEDS, ColorFromPalette(RainbowColors_p,dist_temp,BRIGHTNESS, LINEARBLEND));

  
// send the 'leds' array out to the actual LED strip
  FastLED.show();
  delay( 100 ); // power-up safety delay

}



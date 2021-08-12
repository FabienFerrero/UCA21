/******************************************************************************
kXTJ3-1057 + FastLED Arduino
For UCA21
Fabien Ferrero
Aug, 2021

Resources:
Uses Wire.h for i2c operation

Distributed as-is; no warranty is given.
******************************************************************************/




#include <FastLED.h> // http://librarymanager/All#FASTLED
#include "kxtj3-1057.h" // http://librarymanager/All#kxtj3-1057

// Accelerometer provides different Power modes by changing output bit resolution
#define LOW_POWER
//#define HIGH_RESOLUTION

// Enable Serial debbug on Serial UART to see registers wrote
#define KXTJ3_DEBUG Serial

#include "kxtj3-1057.h"
#include "Wire.h"

float   sampleRate = 6.25;  // HZ - Samples per second - 0.781, 1.563, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
uint8_t accelRange = 2;     // Accelerometer range = 2, 4, 8, 16g

KXTJ3 myIMU(0x0E); // Address can be 0x0E or 0x0F


#define DATA_PIN    4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    21
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          16
#define FRAMES_PER_SECOND  120

//#define DEBUG          

uint8_t gHue = 0; // rotating "base color" used by many of the patterns



void setup()
{  
   
  Serial.begin(115200);
  delay(1000); //wait until serial is open...


  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);  
  
  if( myIMU.begin(sampleRate, accelRange) != 0 )
  {
    Serial.print("Failed to initialize IMU.\n");
  }
  else
  {
    Serial.print("IMU initialized.\n");
  }
  
  // Detection threshold, movement duration and polarity
  myIMU.intConf(123, 1, 10, HIGH);

  uint8_t readData = 0;

  // Get the ID:
  myIMU.readRegister(&readData, KXTJ3_WHO_AM_I);
  Serial.print("Who am I? 0x");
  Serial.println(readData, HEX);

}


void loop()
{
  myIMU.standby( false );
  int16_t dataHighres = 0;
  if( myIMU.readRegisterInt16( &dataHighres, KXTJ3_OUT_X_L ) == 0 ){}

  #ifdef DEBUG
  Serial.print(" Acceleration X RAW = ");
  Serial.println(dataHighres);
  #endif
  int offset = dataHighres/1550; 
if( myIMU.readRegisterInt16( &dataHighres, KXTJ3_OUT_Y_L ) == 0 ){}
  #ifdef DEBUG
  Serial.print(" Acceleration Y RAW = ");
  Serial.println(dataHighres);
  #endif
  int color = dataHighres/3000;
  
  myIMU.standby( true );

  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 10);
  //fade_video( leds, NUM_LEDS, 20);
  //int pos = beatsin16( 13, 0, NUM_LEDS-1 )+offset;
  int pos = 10-offset;
  if(pos==0){ pos = 1;}// To avoid first LED
  leds[pos] += CHSV( gHue, 255, 192);

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND); 
   // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue=gHue+color; } // slowly cycle the "base color" through the rainbow 

  delay(20);

}

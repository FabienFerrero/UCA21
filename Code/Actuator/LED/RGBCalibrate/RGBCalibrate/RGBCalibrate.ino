#include "FastLED.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RGB Calibration code
//
// Use this sketch to determine what the RGB ordering for your chipset should be.  Steps for setting up to use:

// * Uncomment the line in setup that corresponds to the LED chipset that you are using.  (Note that they
//   all explicitly specify the RGB order as RGB)
// * Define DATA_PIN to the pin that data is connected to.
// * (Optional) if using software SPI for chipsets that are SPI based, define CLOCK_PIN to the clock pin
// * Compile/upload/run the sketch

// You can then test this ordering by setting the RGB ordering in the addLeds line below to the new ordering
// and it should come out correctly, 1 red, 2 green, and 3 blue.
//
//////////////////////////////////////////////////

#include "LowPower.h"


#define NUM_LEDS 21

// Data pin that led data will be written out over
#define DATA_PIN 4
// Clock pin only needed for SPI based chipsets when not using hardware SPI
//#define CLOCK_PIN 8

CRGB leds[NUM_LEDS];
int lum = 20;

void setup() {
    // sanity check delay - allows reprogramming if accidently blowing power w/leds
    delay(2000);

    // Uncomment one of the following lines for your leds arrangement.
   
     FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
   
}

void loop() {
    leds[0] = CRGB(lum,lum,lum);
    leds[1] = CRGB(lum,lum,lum);
    leds[2] = CRGB(lum,lum,lum);
    leds[3] = CRGB(lum,lum,lum);
    leds[4] = CRGB(lum,lum,lum);
    leds[5] = CRGB(lum,lum,lum);
    leds[6] = CRGB(lum,lum,lum);
    leds[7] = CRGB(lum,lum,lum);
    leds[8] = CRGB(lum,lum,lum);
    leds[10] = CRGB(30,10,50);
    leds[12] = CRGB(20,40,20);
    leds[14] = CRGB(30,10,0);
    
    FastLED.show();
    delay(3000);
    leds[0] = CRGB(lum,0,0);
    leds[1] = CRGB(0,lum,0);
    leds[2] = CRGB(0,lum,lum);
    leds[3] = CRGB(lum,0,lum);
    leds[4] = CRGB(lum,lum,lum);
    leds[5] = CRGB(lum,lum,0);
    leds[6] = CRGB(0,lum,0);
    leds[7] = CRGB(0,0,lum);
    leds[8] = CRGB(0,lum,lum);
    FastLED.show();
    do_sleep(5);
        
}

void do_sleep(unsigned int sleepyTime) {
  unsigned int eights = sleepyTime / 8;
  unsigned int fours = (sleepyTime % 8) / 4;
  unsigned int twos = ((sleepyTime % 8) % 4) / 2;
  unsigned int ones = ((sleepyTime % 8) % 4) % 2;

#ifdef SHOW_DEBUGINFO
  Serial.print("Sleeping for ");
  Serial.print(sleepyTime);
  Serial.println(" seconds");  
  delay(100); //Wait for serial to complete
#endif


  for ( int x = 0; x < eights; x++) {
    // put the processor to sleep for 8 seconds
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
  for ( int x = 0; x < fours; x++) {
    // put the processor to sleep for 4 seconds
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
  }
  for ( int x = 0; x < twos; x++) {
    // put the processor to sleep for 2 seconds
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  }
  for ( int x = 0; x < ones; x++) {
    // put the processor to sleep for 1 seconds
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  }
  addMillis(sleepyTime * 1000);
}

extern volatile unsigned long timer0_millis;
void addMillis(unsigned long extra_millis) {
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis += extra_millis;
  SREG = oldSREG;
  sei();
}

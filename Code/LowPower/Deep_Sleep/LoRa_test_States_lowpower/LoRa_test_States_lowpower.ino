/*
  LoRa test States

  The objective of this sketch is to measure the power consumption in different states

  The code was modified from LoRa Duplex from  Tom Igoe
*/
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include "LowPower.h"

//#define debugSerial Serial
//#define SHOW_DEBUGINFO

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 8;       // LoRa radio reset
const int irqPin = 6;         // change for your board; must be a hardware interrupt pin


// Accelerometer provides different Power modes by changing output bit resolution
#define LOW_POWER
//#define HIGH_RESOLUTION

// Enable Serial debbug on Serial UART to see registers wrote
#define KXTJ3_DEBUG Serial

//#include "kxtj3-1057.h"
#include "Wire.h"

float   sampleRate = 6.25;  // HZ - Samples per second - 0.781, 1.563, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
uint8_t accelRange = 2;     // Accelerometer range = 2, 4, 8, 16g

//KXTJ3 myIMU(0x0E); // Address can be 0x0E or 0x0F


void setup() {
  
  delay(1000); // wait to be sure that measurement with a DC source will start
  Serial.begin(9600);                   // initialize serial
 // while (!Serial);
 // Serial.println("LoRa States power Consumption");
  pinMode(7, OUTPUT);

digitalWrite(7, HIGH); 


// override the default CS, reset, and IRQ pins (optional)
 LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(870E6)) {             // initialize ratio at 870 MHz to limit interferance with LoRaWan
   // Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

 //  myIMU.standby( true );

}

void loop() {
 // Serial.begin(9600);  
 digitalWrite(7, HIGH); 
 delay(100);
 LoRa.sleep(); 
delay(100);
digitalWrite(7, LOW);
  
    do_sleep(2);
 
Serial.end();
delay(100);
LoRa.sleep();

delay(100);
digitalWrite(7, HIGH);      
    do_sleep(2);    
}



extern volatile unsigned long timer0_millis;
void addMillis(unsigned long extra_millis) {
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis += extra_millis;
  SREG = oldSREG;
  sei();
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

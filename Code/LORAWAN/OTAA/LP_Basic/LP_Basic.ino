/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include "LowPower.h"

//Sensors librairies

#define SHOW_LED

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x97, 0x4C, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xFC, 0x3B, 0xF9, 0x11, 0xF1, 0x05, 0xB6, 0x10, 0xF3, 0x13, 0xDD, 0x12, 0x71, 0xB3, 0xAA, 0x95 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

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


static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 8,
    .dio = {6, 6, 6},
};

// ---------------------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------------------


void FillLEDsFromPaletteColors( uint8_t colorIndex,uint8_t brightness)
{
     
    for( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        colorIndex += 3;
    }
}


void StartLED( uint8_t palette, int LED_delay, uint8_t brightness )
{
    long j=(long)UPDATES_PER_SECOND*LED_delay/1000;
    static uint8_t startIndex = 0;

    if( palette ==  0)  { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
        if( palette == 1)  { currentPalette = RainbowColors_p;   currentBlending = NOBLEND;  }
        if( palette == 4)  { currentPalette = ForestColors_p;              currentBlending = LINEARBLEND; } //GREEN
        if( palette == 9)  { currentPalette = LavaColors_p; currentBlending = NOBLEND;  }
     
    for(int i =0;i< j;i++){
    FillLEDsFromPaletteColors( startIndex,brightness);    
    FastLED.show();
    FastLED.delay(LED_delay / UPDATES_PER_SECOND);
    startIndex++;
    }
    FastLED.clear ();
    FastLED.show();
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            #ifdef SHOW_LED
           StartLED(9,1000,100);
          #endif
          #ifdef SHOW_DEBUGINFO
          Serial.println(F("EV_JOINING"));
    #endif
       break;
        case EV_JOINED:
            #ifdef SHOW_LED
            StartLED(4,1000,100);
            #endif
            //#ifdef SHOW_DEBUGINFO
            Serial.println("EV_JOINED");
            //#endif
           // setDataRate();  // adapt SF
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              //#ifdef SHOW_DEBUGINFO
        Serial.print("Received ");
        Serial.println(LMIC.dataLen/4);
        Serial.println(F(" downlink(s)"));
        for (int i = 0; i < LMIC.dataLen; i++) {
        if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
            Serial.print(F("0"));
        }
        Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
    }
    Serial.println();
   // #endif 
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
       
    // Formatting for Cayenne LPP   
    
    int bat = (int)(readVcc()/10);  // readVCC returns in tens of mVolt ; // multifly by 10 for V in Cayenne

    unsigned char mydata[4];
    mydata[0] = 0x2;  // 2nd Channel
    mydata[1] = 0x2;  // Analog Value
    mydata[2] = bat >> 8;
    mydata[3] = bat & 0xFF;
        
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    Serial.println(F("PQ")); //Packet queued
  }
  // Next TX is scheduled after TX_COMPLETE event.
}


void setDataRate() {
  switch (LMIC.datarate) {
    case DR_SF12:
    #ifdef SHOW_DEBUGINFO
    Serial.println("Datarate: SF12");
    #endif      
      TX_INTERVAL = 4800;
      break;
    case DR_SF11: 
    #ifdef SHOW_DEBUGINFO
    Serial.println("Datarate: SF11");
    #endif
      TX_INTERVAL = 2400;
      break;
    case DR_SF10: 
    #ifdef SHOW_DEBUGINFO
    Serial.println("Datarate: SF10");
    #endif
      TX_INTERVAL = 1200;
      break;
    case DR_SF9: 
    #ifdef SHOW_DEBUGINFO
    Serial.println("Datarate: SF9");
    #endif
      TX_INTERVAL = 600;
      break;
    case DR_SF8: 
    #ifdef SHOW_DEBUGINFO
    Serial.println("Datarate: SF8");
    #endif
      TX_INTERVAL = 360;
      break;
    case DR_SF7: 
    #ifdef SHOW_DEBUGINFO
    Serial.println("Datarate: SF7");
    #endif
      TX_INTERVAL = 180;
      break;
    case DR_SF7B: 
    #ifdef SHOW_DEBUGINFO
    Serial.println("Datarate: SF7B");
    #endif
      TX_INTERVAL = 180;
      break;
    case DR_FSK: 
    #ifdef SHOW_DEBUGINFO
    Serial.println("Datarate: FSK");
    #endif
      TX_INTERVAL = 180;
      break;
    default: Serial.println("Datarate Unknown Value: ");
      Serial.println(LMIC.datarate); TX_INTERVAL = 600;
      break;
  }
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


long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}


void setup() {
    Serial.begin(115200);
    #ifdef SHOW_LED
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    StartLED(1,1400,100);
    delay(1000);
    #endif 

  
   
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

     /* This function is intended to compensate for clock inaccuracy (up to ±10% in this example), 
    but that also works to compensate for inaccuracies due to software delays. 
    The downside of this compensation is a longer receive window, which means a higher battery drain. 
    So if this helps, you might want to try to lower the percentage (i.e. lower the 10 in the above call), 
    often 1% works well already. */
    
    LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   
   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.
   
/*******************************************************************************
 This exemples has been modified by Fabien Ferrero to work on UCA Education board 
 and to remotly control the RGB LEDs
 ****************************************************************************************
 */

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include "LowPower.h"

//Sensors librairies

//Commented out keys have been zeroed for github

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

// LED control
#include <FastLED.h>
#define LED_PIN     4
#define NUM_LEDS    21
#define BRIGHTNESS  64
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100
CRGB leds[NUM_LEDS];
CRGBPalette16 currentPalette;
TBlendType    currentBlending;
extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;


static osjob_t sendjob;

// global enviromental parameters : Place here the environment data you want to measure


static float batvalue = 0.0;

static int LED_RED = 0;
static int LED_BLUE = 0;
static int LED_GREEN = 0;

// Pin mapping for RFM95
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 8,
  .dio = {6, 6, 6},
};

// ---------------------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------------------


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned int TX_INTERVAL = 5;

void setColor(int redValue,  int blueValue, int greenValue) {

fill_solid( leds, NUM_LEDS, CRGB(greenValue,redValue,blueValue));
FastLED.show();  
 
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

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
        Serial.print(v, HEX);
}



void updateEnvParameters() // place here your sensing
{  
  
  batvalue = (int)(readVcc()/10);  // readVCC returns in tens of mVolt 

  // print out the value you read:
  Serial.print("Vbatt : ");
  Serial.println(batvalue);

}


void onEvent (ev_t ev) {
  
  //Serial.print(os_getTime());
  //Serial.print(": ");
  
 
  switch (ev) {
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
    Serial.println(F("EV_JOINING"));
    setColor(0, 0, 32);   //RED   
      break;
    case EV_JOINED:
    Serial.println(F("EV_JOINED"));   
    setColor(32, 0, 0); //GREEN  
     
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
      // Ok send our first data in 10 ms
      //os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(10), do_send);
      break;
         
    
    case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));      
      lmicStartup(); //Reset LMIC and retry
      break;
    case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    
      lmicStartup(); //Reset LMIC and retry
      break;
    case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      
      if (LMIC.txrxFlags & TXRX_ACK) 
      Serial.print(F("Received ack"));              
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" byte(s) downlink(s)"));
        for (int i = 0; i < LMIC.dataLen; i++) {
        if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
            Serial.print(F("0"));
        }
        Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
    }
    Serial.println();
            
        switch (LMIC.frame[LMIC.dataBeg]){
        case 0x01 :
        LED_RED = (word(LMIC.frame[LMIC.dataBeg+1])); // Converter download payload in int
        Serial.print("Set RED COLOR to :");
        Serial.println(LED_RED);
        break;
       
        case 0x02 :
        LED_BLUE = (word(LMIC.frame[LMIC.dataBeg+1])); // Converter download payload in int
        Serial.print("Set BLUE COLOR to :");
        Serial.println(LED_BLUE);
        break;

        case 0x03 :
        LED_GREEN = (word(LMIC.frame[LMIC.dataBeg+1])); // Converter download payload in int
        Serial.print("Set GREEN COLOR to :");
        Serial.println(LED_GREEN);
        break;
        }
      
     }  
      delay(50);
      setColor(LED_GREEN*2.55, LED_BLUE*2.55, LED_RED*2.55);
           
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
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running

 
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    // Here the sensor information should be retrieved
    
    updateEnvParameters(); // Sensing parameters are updated
  


// Formatting for Cayenne LPP
    
    
    int bat = batvalue; // multifly by 10 for V in Cayenne

    unsigned char mydata[16];
    mydata[0] = 0x2;  // 2nd Channel
    mydata[1] = 0x2;  // Analog Value
    mydata[2] = bat >> 8;
    mydata[3] = bat & 0xFF;
    mydata[4] = 0x6;  // 6th Channel
    mydata[5] = 0x3;  // Analog Value PWM red
    mydata[6] = LED_RED >> 8;
    mydata[7] = LED_RED & 0xFF;
    mydata[8] = 0x7;  // 7th Channel
    mydata[9] = 0x3;  // Analog Value PWM red
    mydata[10] = LED_BLUE >> 8;
    mydata[11] = LED_BLUE & 0xFF;
    mydata[12] = 0x8;  // 8th Channel
    mydata[13] = 0x3;  // Analog Value PWM red
    mydata[14] = LED_GREEN >> 8;
    mydata[15] = LED_GREEN & 0xFF;
    
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued")); //Packet queued
  }
  // Next TX is scheduled after TX_COMPLETE event.
}


void lmicStartup() {
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

    LMIC_setLinkCheckMode(1);
    LMIC_setAdrMode(1);
    LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100); // Increase window time for clock accuracy problem

  // Start job (sending automatically starts OTAA too)
  // Join the network, sending will be
  // started after the event "Joined"
  LMIC_startJoining();
}


// ---------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000); //Wait 1s in order to avoid UART programmer issues when a battery is used
  
  Serial.begin(115200);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  
  
  Serial.println(F("Starting"));
  delay(100);
  
  
  Wire.begin();

  // LMIC init

  os_init();
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

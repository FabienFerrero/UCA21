
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

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

   Do not forget to define the radio type correctly in config.h.

/*******************************************************************************
 This exemples has been modified by Fabien Ferrero to work on UCA board 
 and to send various sensors payload
 ****************************************************************************************
 */

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include "LowPower.h"

//Sensors librairies
#include <Wire.h>
#include <LTR303.h>
#include "kxtj3-1057.h" // http://librarymanager/All#kxtj3-1057
#include "SHTC3.h"

#define SHOW_DEBUGINFO

// Create sensors:

LTR303 lightsensor;
KXTJ3 myIMU(0x0E); // Address can be 0x0E or 0x0F
SHTC3 s(Wire);

float   sampleRate = 6.25;  // HZ - Samples per second - 0.781, 1.563, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
uint8_t accelRange = 2;     // Accelerometer range = 2, 4, 8, 16g


///Commented out keys have been zeroed for github

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



static osjob_t sendjob;

// global enviromental parameters
static float temperature = 0.0;
//static float pressure = 0.0;
static float humidity = 0.0;
static float batvalue;
static double light;
static int16_t a_x = 0; // acclerometer
static int16_t a_y = 0;
static int16_t a_z = 0;




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


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned int TX_INTERVAL = 30;

void setDataRate() {
  switch (LMIC.datarate) {
    case DR_SF12:
    #ifdef SHOW_DEBUGINFO
    Serial.println(F("Datarate: SF12"));
    #endif      
      TX_INTERVAL = 1920;
      break;
    case DR_SF11: 
    #ifdef SHOW_DEBUGINFO
    Serial.println(F("Datarate: SF11"));
    #endif
      TX_INTERVAL = 960;
      break;
    case DR_SF10: 
    #ifdef SHOW_DEBUGINFO
    Serial.println(F("Datarate: SF10"));
    #endif
      TX_INTERVAL = 480;
      break;
    case DR_SF9: 
    #ifdef SHOW_DEBUGINFO
    Serial.println(F("Datarate: SF9"));
    #endif
      TX_INTERVAL = 240;
      break;
    case DR_SF8: 
    #ifdef SHOW_DEBUGINFO
    Serial.println(F("Datarate: SF8"));
    #endif
      TX_INTERVAL = 120;
      break;
    case DR_SF7: 
    #ifdef SHOW_DEBUGINFO
    Serial.println(F("Datarate: SF7"));
    #endif
      TX_INTERVAL = 60;
      break;
    case DR_SF7B: 
    #ifdef SHOW_DEBUGINFO
    Serial.println(F("Datarate: SF7B"));
    #endif
      TX_INTERVAL = 60;
      break;
    case DR_FSK: 
    #ifdef SHOW_DEBUGINFO
    Serial.println(F("Datarate: FSK"));
    #endif
      TX_INTERVAL = 60;
      break;
    default: Serial.print(F("Datarate Unknown Value: "));
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

  digitalWrite(7, LOW); // switch off Vcc_Periph LDO 

#ifdef SHOW_DEBUGINFO
  Serial.print(F("Sleeping for "));
  Serial.print(sleepyTime);
  Serial.print(F(" seconds = "));
  Serial.print(eights);
  Serial.print(F(" x 8 + "));
  Serial.print(fours);
  Serial.print(F(" x 4 + "));
  Serial.print(twos);
  Serial.print(F(" x 2 + "));
  Serial.println(ones);
  delay(500); //Wait for serial to complete
#endif

   Serial.end(); // switch off serial

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

  digitalWrite(7, HIGH); // switch ON Vcc_Periph LDO

  Serial.begin(115200); // restart Serial 

  delay(100);
  
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

void updateEnvParameters()
{
  unsigned int data0, data1;
  temperature = s.readTempC();
  delay(10); // add delay to finish I2C read
  humidity = s.readHumidity();
  delay(10); // add delay to finish I2C read
  
  batvalue = (int)(readVcc()/10);  // readVCC returns in tens of mVolt 
  int16_t dataHighres = 0;
  if( myIMU.readRegisterInt16( &dataHighres, KXTJ3_OUT_X_L ) == 0 ){}
  a_x = dataHighres/16.384;
  if( myIMU.readRegisterInt16( &dataHighres, KXTJ3_OUT_Y_L ) == 0 ){}
  a_y = dataHighres/16.384;
  if( myIMU.readRegisterInt16( &dataHighres, KXTJ3_OUT_Z_L ) == 0 ){}
  a_z = dataHighres/16.384;

  lightsensor.setControl(0, false, false);
  lightsensor.setMeasurementRate(1,3);
  lightsensor.setPowerUp(); // power-up light sensor
  
  delay(200);// delay to respect integration time
  lightsensor.getData(data0,data1);  
  // Perform lux calculation:
  lightsensor.getLux(0,1,data0,data1,light); 
  

  #ifdef SHOW_DEBUGINFO
  // print out the value you read:
            Serial.print("Sensors values : temp = ");
            Serial.print( temperature);
            Serial.print("deg, hum= ");
            Serial.print( humidity);
            Serial.print("%, lum = ");            
            Serial.print( light);            
            Serial.print(" lumen, Accel : X = ");
            Serial.print( a_x);
            Serial.print(" G, Y = ");
            Serial.print( a_y);
            Serial.print(" G, Z = ");
            Serial.print( a_z);
            Serial.println(" G");          
  #endif
  
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}


void onEvent (ev_t ev) {
  
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      #ifdef SHOW_DEBUGINFO
      Serial.println(F("EV_SCAN_TIMEOUT"));
      #endif     
      break;
    case EV_BEACON_FOUND:
      #ifdef SHOW_DEBUGINFO
      Serial.println(F("EV_BEACON_FOUND"));
      #endif      
      break;
    case EV_BEACON_MISSED:
      //Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      //Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      #ifdef SHOW_DEBUGINFO
      Serial.println(F("EV_JOINING"));
      #endif      
      break;
    case EV_JOINED:
      #ifdef SHOW_DEBUGINFO
      Serial.println(F("EV_JOINED"));
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
         #endif
         setDataRate();  // adapt SF    
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
            // size, we don't use it in this example.
           // LMIC_setLinkCheckMode(0);           
         
      
      // Ok send our first data in 10 ms
      //os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(10), do_send);
      break;
       
    case EV_JOIN_FAILED:
    #ifdef SHOW_DEBUGINFO
    Serial.println(F("EV_JOIN_FAILED"));
    #endif
      
      lmicStartup(); //Reset LMIC and retry
      break;
    case EV_REJOIN_FAILED:
    #ifdef SHOW_DEBUGINFO
    Serial.println(F("EV_REJOIN_FAILED"));
    #endif
      
      lmicStartup(); //Reset LMIC and retry
      break;
    
    case EV_TXCOMPLETE:

    #ifdef SHOW_DEBUGINFO
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    #endif
      
      if (LMIC.txrxFlags & TXRX_ACK)
      #ifdef SHOW_DEBUGINFO
      Serial.println(F("Received ack"));
      #endif
        
      if (LMIC.dataLen) {
        #ifdef SHOW_DEBUGINFO
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        for (int i = 0; i < LMIC.dataLen; i++) {
              if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
              Serial.print(F("0"));
              }
              Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
              }
              Serial.println("");
        #endif 
       
      }
            // Schedule next transmission
      setDataRate();
      do_sleep(TX_INTERVAL);
      os_setCallback(&sendjob, do_send);
      break;
    case EV_LOST_TSYNC:
      #ifdef SHOW_DEBUGINFO
      Serial.println(F("EV_LOST_TSYNC"));
      #endif      
      break;
    case EV_RESET:
      #ifdef SHOW_DEBUGINFO
      Serial.println(F("EV_RESET"));
      #endif        
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      #ifdef SHOW_DEBUGINFO
      Serial.println(F("EV_RXCOMPLETE"));
      #endif      
      break;
    case EV_LINK_DEAD:
      #ifdef SHOW_DEBUGINFO
      Serial.println(F("EV_LINK_DEAD"));
      #endif       
      break;
    case EV_LINK_ALIVE:
      #ifdef SHOW_DEBUGINFO
      Serial.println(F("EV_LINK_ALIVE"));
      #endif       
      break;
      case EV_TXSTART:
            #ifdef SHOW_DEBUGINFO
            Serial.println(F("EV_TXSTART"));
            #endif
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
      #ifdef SHOW_DEBUGINFO
      Serial.println(F("Unknown event"));
      #endif      
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
    
    updateEnvParameters();

    int t = (int)((temperature) * 10.0); // adapt to Cayenne format
    int h = (int)(humidity * 2.0); // adapt to Cayenne format
    int bat = batvalue; 
    int l = light; // light sensor in Lx

    unsigned char mydata[23];
            mydata[0] = 0x1; // CH1
            mydata[1] = 0x67; // Temp
            mydata[2] = t >> 8;
            mydata[3] = t & 0xFF;
            mydata[4] = 0x2; // CH2
            mydata[5] = 0x68; // Humidity
            mydata[6] = h & 0xFF;
            mydata[7] = 0x3; // CH3
            mydata[8] = 0x2; // Analog output
            mydata[9] = bat >> 8;
            mydata[10] = bat & 0xFF;
            mydata[11] = 0x4; // CH4
            mydata[12] = 0x65; // Luminosity
            mydata[13] = l >> 8;
            mydata[14] = l & 0xFF;
            mydata[15] = 0x4; // CH4
            mydata[16] = 0x71; // Accelerometer
            mydata[17] = a_x >> 8;
            mydata[18] = a_x & 0xFF;
            mydata[19] = a_y >> 8;
            mydata[20] = a_y & 0xFF;
            mydata[21] = a_z >> 8;
            mydata[22] = a_z & 0xFF;
    
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

void setup() {
  Serial.begin(115200);

  pinMode(7, OUTPUT); // LDO control for low power mode
  digitalWrite(7, HIGH);
  
  delay(1000); //Wait 1s in order to avoid UART programmer issues when a battery is used

  #ifdef SHOW_DEBUGINFO
  Serial.println(F("Starting"));
  delay(100);
  #endif
  
  Wire.begin();
  s.begin(true);
  // Set-up sensors

  lightsensor.begin(); 
    

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

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();    
    LMIC_setLinkCheckMode(1);
    LMIC_setAdrMode(1);

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

/* 
 *  
 *  Copyright (c) 2018 Fabien Ferrero
 *
 *  Permission is hereby granted, free of charge, to anyone
 *  obtaining a copy of this document and accompanying files,
 *  to do whatever they want with them without any restriction,
 *  including, but not limited to, copying, modification and redistribution.
 *  NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 *  This example receive LoRa packet with frequency hooping information in the
 *  payload and change the frequency for the next packet.
 *  This code made for Heltec LoRa board 860-915MHz version
 *  It can be use to measure LoRa device  radiation performance and to tune the 
 *  device antenna
 *  
 */

#define DEBUG   // If DEBUG, plot on the serial plotter the 3 accelerometer axis

#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include <SPI.h>
#include <LoRa.h>
#define SS 10
#define RST 8
#define DI0 6
#define BAND 868E6
#define spreadingFactor 7
#define SignalBandwidth 250E3
#define preambleLength 8
#define codingRateDenominator 8


long freq = 868e6; // Default frequency at initialisation
long freq_max=870e6;
long freq_min=865e6;

long max_freq=freq; // freq with max RSSI
long min_freq=freq; // freq with max RSSI

bool Pushdetected = false; // Change mode
bool Pushdetected2 = false; // change LED intensity

int counter = -1; // number of received packet since the last init
int index = -1;
int RSSI = -140;
int max_RSSI=-120;
int min_RSSI=-20;
int average_RSSI=0;
int RSSI_array[21];
int spectrum_array[21];
int offset = 120; // offset for RSSI
int norm = 5; // Normalization value
int buff = 21; // Buffer size for measurement

int mode = 0; //define the mode used :  0:(green)channel sounding with averaging 1:(yellow) channel sounding w/o averaging 3:(red)live antenna demo 4:(blue)

// LED control
#include <FastLED.h>
#define LED_PIN     4
#define NUM_LEDS    21
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100

int BRIGHTNESS = 16 ;
CRGB leds[NUM_LEDS];




// Function set the RGB color on all the LEDs
void setColor(int redValue,  int blueValue, int greenValue) {
fill_solid( leds, NUM_LEDS, CRGB(greenValue,redValue,blueValue));

}

// Function convert RSSI value into the LED display


void rssi2led(int RSSI,long freq){

 
  for( int i=0; i<=buff; i++ ) {
    leds[i] = ColorFromPalette(RainbowColors_p,spectrum_array[i],BRIGHTNESS, LINEARBLEND);    
      }
  uint8_t odd = freq % 2;
  
  if (odd==0) {
    fill_solid( leds, 1, CRGB(0,BRIGHTNESS,0));
    }
  else {
    fill_solid( leds, 1, CRGB(BRIGHTNESS,0,0));
    }
    
  FastLED.show(); 
  }


void setup() { 

  
  #ifdef DEBUG
  Serial.begin(115200);
  while (!Serial); //if just the the basic function, must connect to a computer
  delay(1000);
  #endif

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  //pinMode(7, OUTPUT);
  //digitalWrite(7, HIGH);
  
   
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    
  Serial.println("LoRa Receiver");
  LoRa.setPins(SS,RST,DI0);

  if (!LoRa.begin(BAND)) {
    setColor(0, 0, BRIGHTNESS); //YELLOW
    FastLED.show(); 
    while (1);
  }

  #ifdef DEBUG
  Serial.println("LoRa Init OK");  
  Serial.print("LoRa Frequency: ");
  Serial.println(BAND);  
  Serial.print("LoRa Spreading Factor: ");
  Serial.println(spreadingFactor);
  #endif
  LoRa.setSpreadingFactor(spreadingFactor);
  #ifdef DEBUG
  Serial.print("LoRa Signal Bandwidth: ");
  Serial.println(SignalBandwidth);
  #endif
  LoRa.setSignalBandwidth(SignalBandwidth);

  LoRa.setCodingRate4(codingRateDenominator);
   
}

void loop() {

  if (digitalRead(3)==LOW) { 
    Pushdetected = true;    
    }
    if (digitalRead(2)==LOW) { 
    Pushdetected2 = true;    
    }   
  
  if(Pushdetected){ // reset Peak freq and max RSSI value
    Pushdetected = false;
    counter = -1; // start with count < 0 to add a small delay before the start of the measurement
    index = 0;
    //mode = mode+1;
    if (mode==4) {mode=0;}

    
    if(mode ==0){
      // Serial.println("Mode 0");
      offset = 120;
      buff = 21;
      norm = 5; //5dB par LED
      setColor(BRIGHTNESS, 00, 0); //GREEN  
      FastLED.show(); 
      delay (1000);}
    else if(mode ==1){
      //Serial.println("Mode 1");
      offset = 120;
      buff = 1;
      norm = 5; //5dB par LED
      setColor(BRIGHTNESS, 00, BRIGHTNESS); //Yellow  
      FastLED.show(); 
      delay (1000);
      }
            
    else if (mode==2) {
    //Serial.println("Mode 2");  
      offset = 70;
      norm = 2; //2dB par LED
      buff = 2;
      setColor(00, 00, BRIGHTNESS); //RED  
      FastLED.show(); 
      delay (1000);
      }
    else if (mode==3){
      //Serial.println("Mode 3"); 
      offset = 90;
      norm = 2; //2dB par LED
      buff = 1;
      setColor(00, BRIGHTNESS, 00); //Blue  
      FastLED.show(); 
      delay (1000);
      }      
    }

if(Pushdetected2){ // Change LED intensity
    Pushdetected2 = false;
BRIGHTNESS = BRIGHTNESS + 32;
if (BRIGHTNESS > 150) { 
  BRIGHTNESS = 16;
    }
    rssi2led(average_RSSI,freq/ 1e6);
    delay(500);
}
    
  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packets      
    counter++;
    
    if (index<buff){
    index++;}

    if (counter >buff) { // reset array
      counter = 0;
      index = buff;
      }
    
    // read packet
    while (LoRa.available()) {
      String data = LoRa.readString();
      freq = data.toInt(); // data converted to int
     // If a packet is not received with a suitable frequency, set in the good freq range
     if(freq > freq_max){
     freq = freq_max;
     }
     if(freq < freq_min){
     freq = freq_min;
     }

     
     
    //RSSI calculation
      RSSI = LoRa.packetRssi()+157; // Extract raw value of packetrssi register
      RSSI = (16*RSSI)/15; //value correction from the datasheet
      RSSI = RSSI -157; // Re-Calculate value in dBm

      if(counter >= 0){
      RSSI_array[counter]=RSSI;      
      }
  
    long freq_MHz= freq / 1e6; // freq in MHz 
   
    if(index > (buff-1)){

    average_RSSI = 0;
    
    
    // Check for Min Max RSSI value
    max_RSSI=-120;
    min_RSSI=-20;

    for( int i=0; i<=index; i++ ) {
     
      if(RSSI_array[i] > max_RSSI){
      max_RSSI = RSSI_array[i];
      }
      if(RSSI_array[i] < min_RSSI){
      min_RSSI = RSSI_array[i];
      }   
    }

    // If RSSI difference is not at least 5dB, increase it to 5dB
    if (max_RSSI-min_RSSI <=10){
      min_RSSI = max_RSSI-10;
      }
    
    for( int i=0; i<=index; i++ ) {
    average_RSSI = average_RSSI + RSSI_array[i];
    spectrum_array[i] = map(RSSI_array[i], min_RSSI,max_RSSI,170,0); // Map value from luminosity sensor to LED
         }
    average_RSSI = average_RSSI / (index+1);
    

    #ifdef DEBUG     
    Serial.print(freq);
    long minfreq_MHz= min_freq / 1e6; // freq in MHz

    Serial.print("  ");
    Serial.print(RSSI);
    Serial.print("  ");
    Serial.print(min_RSSI);
    Serial.print("  ");
    Serial.println(max_RSSI);
        
    delay(10);
    #endif
    
    rssi2led(average_RSSI,freq/ 1e6);
      
      }
    }    
    long maxfreq_MHz= max_freq / 1e6; // freq in MHz
    
    LoRa.setFrequency(freq); // change frequency
  }
}

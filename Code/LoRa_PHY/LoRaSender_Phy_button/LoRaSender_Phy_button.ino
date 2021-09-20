#include <SPI.h>
#include <LoRa.h>

int counter = 0;

// Parameters you can play with :

int txPower = 14; // from 0 to 20, default is 14
int spreadingFactor = 12; // from 7 to 12, default is 12
long signalBandwidth = 125E3; // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,41.7E3,62.5E3,125E3,250E3,500e3, default is 125E3
int codingRateDenominator=5; // Numerator is 4, and denominator from 5 to 8, default is 5
int preambleLength=8; // from 2 to 20, default is 8
String payload = "hello"; // you can change the payload

int i =0;
int SF[6] = {7,8,9,10,11,12};
int j=0;
long BW[6] = {15.6E3,31.25E3,62.5E3,125E3,250E3,500e3};

// LED control
#include <FastLED.h>
#define LED_PIN     4
#define NUM_LEDS    21
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100

int BRIGHTNESS = 16 ;
CRGB leds[NUM_LEDS];



#define SS 10
#define RST 8
#define DI0 3
#define BAND 865E6  // Here you define the frequency carrier



// Function set the RGB color on all the LEDs
void setColor(int redValue,  int blueValue, int greenValue) {
fill_solid( leds, NUM_LEDS, CRGB(greenValue,redValue,blueValue));
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

   
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );

  Serial.println("LoRa Sender");
  Serial.print("SetFrequency : ");
  Serial.print(BAND);
  Serial.println("Hz");
  Serial.print("SetSpreadingFactor : SF");
  Serial.println(spreadingFactor);

  SPI.begin();
  LoRa.setPins(SS,RST,DI0);

  

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
 LoRa.setTxPower(txPower,1);
 LoRa.setSpreadingFactor(spreadingFactor);
 LoRa.setSignalBandwidth(signalBandwidth);
 LoRa.setCodingRate4(codingRateDenominator);
 LoRa.setPreambleLength(preambleLength);
// LoRa.setPolarity(1);
 //LoRa.setFSK(); 
 
}

void loop() {

   if (digitalRead(3)==LOW) { 
    // Fill the number of white LED depending on RSSI
  fill_solid( leds, NUM_LEDS, CRGB(BRIGHTNESS,0,BRIGHTNESS));
  FastLED.show(); 
  delay(1000);
    i++;
    if (i==6){i=0;}
    signalBandwidth = BW[i];
    LoRa.setSignalBandwidth(signalBandwidth);
    }

    if (digitalRead(2)==LOW) { 
    // Fill the number of white LED depending on RSSI
  fill_solid( leds, NUM_LEDS, CRGB(0,BRIGHTNESS,BRIGHTNESS));
  FastLED.show();
  delay(1000);
    j++;
    if (j==6){j=0;}  
    spreadingFactor = SF[j];
    LoRa.setSpreadingFactor(spreadingFactor);
    }

  
  // send packet
  
  LoRa.beginPacket();
  LoRa.print(payload);  
  
  LoRa.endPacket();
  counter++;

  Serial.print("Sending packet with payload {");
  Serial.print(payload);
  Serial.print("} N°");
  Serial.println(counter);
 

  delay(100);
}

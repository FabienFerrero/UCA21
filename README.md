# UCA_Education_Board version 2021
Board for Education

Version 1.0.1, September, 2021

Author: Fabien Ferrero and Trinh Le Huy

This board was developed for education to support courses on embedded software, digital and analog electronic, telecommunication, signal processing and IoT.

# What's new
- Lithium battery charger + JST-PH 2pin and 3pin connector : MCP73831
- Accelerometer sensor : KXTJ3-1057
- Temp/Humidity sensor : SHTC3
- Light sensor : TR-303A
- 21 RGB Leds : WS2812-2020
- Optimized sleep mode to 5uA
- VSWR meter

The board is fabricated by RFThings.

<img src="https://github.com/FabienFerrero/UCA21/blob/18ce02a330d579ff79cec318990150716ca1bb1d/Doc/Pictures/board.png">

# Wiring

```
 ATMega328pb       LoRa RFM95W 
                   Module
 D8          <----> RST
 MISO  (D12) <----> MISO
 MOSI  (D11) <----> MOSI
 SCK   (D13) <----> CLK
 SS    (D10) <----> SEL (Chip Select)
 D6          <----> DIO0
 D6          <----> DIO1
 D6          <----> DIO2
 3.3V        <----> Vcc
 D2          <----> BT0 (no pull-up)
 D3          <----> BT1 (no-pull-up)


 ```
 
 <img src="https://github.com/FabienFerrero/UCA21/blob/522868ad4d04c523dc64f552ade44d6143326396/Doc/Pictures/pinout_UCA.png">
 
 
# USB Driver
The board is using CH340C chip for USB. You may need to install the driver to use the board:
https://sparks.gogo.co.nz/ch340.html


<img src="https://github.com/FabienFerrero/UCA21/blob/69574b64fd2e3864930b67a356f340c04faa4418/Doc/Pictures/usb.png">



# Board Programming - Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/Main/Software) (at least version v1.6.8)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add ```https://rfthings.com.vn/wp-content/uploads/package_rfthings-avr_index.json``` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "RFTHings AVR Boards by RFThings Vietnam"
 6. Select your RFTHings UCA board from the Tools -> Board menu
 7. Select Board version "3.9 and newer : AT328PB" from the Tools -> Board menu
 8. Select the port

# Schematic

The schematic of the PCB is available the Schematic section.


# License


This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

<img src="https://github.com/FabienFerrero/UCA21/blob/main/Doc/Pictures/UCA_logo.png">

Maintained by Fabien Ferrero and Trinh Le Huy

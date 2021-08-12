# UCA_Education_Board version 2021
Board for Education

Version 1.0.0, September, 2021

Author: Fabien Ferrero and Trinh Le Huy

This board was developed for education to support courses on embedded software, digital and analog electronic, telecommunication, signal processing and IoT.

<img src="https://github.com/FabienFerrero/UCA_Education_Board/blob/master/Doc/Pictures/board.png">

The board is fabricated by RFThings.

# Wiring

```
 ATMega328p       LoRa RFM95W 
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
 
 <img src="https://github.com/FabienFerrero/UCA_Education_Board/blob/master/Doc/Pictures/pinout_UCA.png">
 
 
# USB Driver
The board is using CH340C chip for USB to 
You may need to install the driver to use the board:
https://sparks.gogo.co.nz/ch340.html

<img src="https://github.com/FabienFerrero/UCA_Education_Board/blob/master/Doc/Pictures/usb.png">

# Board Programming - Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/Main/Software) (at least version v1.6.8)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add ```https://rfthings.com.vn/wp-content/uploads/2021/07/package_rfthings-avr_index.json``` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "RFTHings AVR Boards by RFThings Vietnam"
 6. Select your RFTHings UCA board from the Tools -> Board menu
 7. Select Board version "3.9 and newer : AT328PB" from the Tools -> Board menu
 8. Select the port

# Schematic

The schematic of the PCB is available the Schematic section.


# License


This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

Maintained by Fabien Ferrero and Trinh Le Huy

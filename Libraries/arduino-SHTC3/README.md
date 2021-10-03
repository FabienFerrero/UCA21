# A minimial arduino library fro SHTC3 Temperature & Humidity sensor

A bare minimal arudino lib for this sensor for low power applications based on official [SHT libraries](https://github.com/Sensirion/arduino-sht).

If you find this library usefull and want to support open software consider to **donate**.
<p align="center">
  <a href="https://www.buymeacoffee.com/boros" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-white.png" alt="Buy Me A Coffee" style="height: 60px !important;width: 217px !important;" ></a>
</p>

## Usage
 The library is designed for low power applications that will poll the reading periodically. When the sensor is not reading
 it is keep asleep consuming less than 1 uA.

```c++

 
    SHTC3 sensor(Wire);

    void setup() {
            Serial.begin(9600)
            Wire.begin();
            sensor.begin();
    }
    void loop() {
            s.sample();
            Serial.print(F("[SHTC3] T:"));
            Serial.print(s.readTempC());
            Serial.print(F(" Cº  /   H: "));
            Serial.print(s.readHumidity());
            Serial.println(F(" %"));
            delay(2000);
    }


```

## Technical notes

- The sensor is ___NOT__ 5V tolerant: If using 5V boards a 3.3V power source or 3.3V regulator need to be used. Most arduino boards have 3.3V power line that can be used to power the device. SCL & SDA require also logic level shifters are required as this pins are nor 5V tolerant.
- Pull-up resistors are typically needed on SCL & SDA lines. Resistor values will depend on frequency and bus configuration.
- Sensor supports up to 400kHz (fast I2C).

## Compilation options:
 if __ARDUINO_SHTC3_NOFLOAT__ is defined no float computation is done. Temperature and Humidity will return a 16bit integer. This optimization will reduce the code size more at expense of precision in the reading.
 
## Notes:
Library has been tested on a arduino pro mini 3.3V.

## Change Log

- 0.0.1  First version
- 0.0.2  Bug correction
- 0.0.3  Fix wake & reset delay bug.

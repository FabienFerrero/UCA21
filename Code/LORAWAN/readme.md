* LoRaWAN MAC

In \libraries\arduino-lmic\project_config\lmic_project_config.h

Add  `#define LMIC_ENABLE_arbitrary_clock_error 1`

In your loraWan code, add after LMIC_reset in setup() :
    LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);'
    
    
    // Make LMIC start its RX windows a bit earlier, to compensate for an
// inaccurate clock.
//
// Beware that a specific value may work for a slow data rate, but not
// for faster ones, and remember that RX1 may use different data rates
// than RX2. Like for some tests, RX2 in EU868 (using SF9) worked with
// the standard settings, but RX1 for SF8 needed 2%, while RX1 for SF7
// even needed 5% of the maximum error. For corrections exceeding 0.4%
// (0.4 / 100) this also needs LMIC_ENABLE_arbitrary_clock_error; see
// https://github.com/mcci-catena/arduino-LMIC/blob/master/README.md

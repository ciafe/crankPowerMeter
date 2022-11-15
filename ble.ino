/**
 * This file keeps the BLE helpers, to send the data over bluetooth
 * to the bike computer. Or any other receiver, if dev/debug.
 *
 * For the Adafruit BLE lib, see:
 * https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/bd0747473242d5d7c58ebc67ab0aa5098db56547/libraries/Bluefruit52Lib
 */
 
#include "ble.h"

#include <stdarg.h>

// Service and character constants at:
// https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/bd0747473242d5d7c58ebc67ab0aa5098db56547/libraries/Bluefruit52Lib/src/BLEUuid.h
/* Pwr Service Definitions
 * Cycling Power Service:      0x1818
 * Power Measurement Char:     0x2A63
 * Cycling Power Feature Char: 0x2A65
 * Sensor Location Char:       0x2A5D
 */


void ble_Setup(void) {

}

void ble_startAdvertising(void) {

}

/*
 * Set up the power service
 */
void ble_setupPwr(void) {

}

/*
 * This service exists only to publish logs over BLE.
 */
void ble_setupLogger(void) {
#ifdef BLE_LOGGING

#endif
}


/*
 * Publish the instantaneous power measurement.
 */
void ble_PublishPower(int16_t instantPwr, uint16_t crankRevs, long millisLast) {

}
/*
 * Publish the battery status measurement.
 */
void blePublishBatt(uint8_t battPercent) {

}

/*
 * Publish a tiny little log message over BLE. Pass a null-terminated
 * char*, in 20 chars or less (counting the null).
 */
#ifdef BLE_LOGGING
void blePublishLog(const char* fmt, ...) {
 
}
#endif



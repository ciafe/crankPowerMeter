
 
#include "ble_ext.h"


void ble_Setup(void) {
  
    BLEDevice::init(BLE_DEV_NAME);
    
    InitBLEServer();

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

boolean ble_isConnected(void)
{
  return true;
}

/*
 * Publish the instantaneous power measurement.
 */
void ble_PublishPower(int16_t instantPwr, uint16_t crankRevs, long millisLast) {
  Serial.println("BLE DUMMY PUBLISH POWER");

  powerIn   = powerIn + 1u;
  if (powerIn > 1000)
  {
    powerIn = 0;
  }
  cadenceIn = 90;
  speedOut  = 25; 

  //speedOut = (cadenceIn * 2.75 * 2.08 * 60*gears[gearIndex]) / 10;            // calculated speed, required by the specification
  indoorBikeDataCharacteristicData[2] = (uint8_t)(speedOut & 0xff);
  indoorBikeDataCharacteristicData[3] = (uint8_t)(speedOut >> 8);             // speed value with little endian order
  indoorBikeDataCharacteristicData[4] = (uint8_t)((cadenceIn * 2) & 0xff);        
  indoorBikeDataCharacteristicData[5] = (uint8_t)((cadenceIn * 2) >> 8);          // cadence value
  indoorBikeDataCharacteristicData[6] = (uint8_t)(constrain(powerIn, 0, 4000) & 0xff);
  indoorBikeDataCharacteristicData[7] = (uint8_t)(constrain(powerIn, 0, 4000) >> 8);    // power value, constrained to avoid negative values, although the specification allows for a sint16
    
  indoorBikeDataCharacteristic.setValue(indoorBikeDataCharacteristicData, 8);       // values sent
  indoorBikeDataCharacteristic.notify();                          // device notified
  
}
/*
 * Publish the battery status measurement.
 */
void ble_PublishBatt(uint8_t battPercent) {
  //Serial.println("BLE DUMMY PUBLISH BATT");
}

/*
 * Publish a tiny little log message over BLE. Pass a null-terminated
 * char*, in 20 chars or less (counting the null).
 */
#ifdef BLE_LOGGING
void blePublishLog(const char* fmt, ...) {
 
}
#endif

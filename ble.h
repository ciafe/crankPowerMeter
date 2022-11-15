
#ifndef BLE_API
#define BLE_API

void ble_Setup(void);


void ble_startAdvertising(void);

/*
 * Set up the power service
 */
void ble_setupPwr(void);
/*
 * This service exists only to publish logs over BLE.
 */
void ble_setupLogger(void);
/*
 * Publish the instantaneous power measurement.
 */
void ble_PublishPower(int16_t instantPwr, uint16_t crankRevs, long millisLast);
/*
 * Publish the battery status measurement.
 */
void blePublishBatt(uint8_t battPercent);

/*
 * Publish a tiny little log message over BLE. Pass a null-terminated
 * char*, in 20 chars or less (counting the null).
 */
void blePublishLog(const char* fmt, ...);

#endif


#ifndef POWERMETER_CFG
#define POWERMETER_CFG


// Trek
#define DEBUG
//#define BLE_LOGGING
//#define CALIBRATE
#define DISABLE_LOGGING  // to the SD
// Crank length, in meters
#define CRANK_RADIUS 0.1725
#define LOAD_OFFSET 255904.f
#define HX711_MULT  -2466.8989547
#define GYRO_OFFSET -31
// Hooked up the wires backwards apparently, force is negated.
// If it isn't, just set to 1.
#define HOOKEDUPLOADBACKWARDS -1
#define DEV_NAME "CiafePwr"

/*
// Steve Merckx defines
//#define DEBUG
//#define BLE_LOGGING
//#define CALIBRATE
#define DISABLE_LOGGING  // to the SD
// Crank length, in meters
#define CRANK_RADIUS 0.1750
#define LOAD_OFFSET -32000.f
#define HX711_MULT  -2719.66716169
#define GYRO_OFFSET 3
// Hooked up the wires backwards apparently, force is negated.
// If it isn't, just set to 1.
#define HOOKEDUPLOADBACKWARDS -1
#define DEV_NAME "JrvsPwr"
*/

// Allie Orbea defines
/*
#define DEBUG
//#define BLE_LOGGING
//#define CALIBRATE
#define DISABLE_LOGGING  // to the SD
#define CRANK_RADIUS 0.1725
#define LOAD_OFFSET 3300.f;  // Allie Orbea
#define HX711_MULT  -2491.63452396
#define GYRO_OFFSET -50
#define HOOKEDUPLOADBACKWARDS -1
#define DEV_NAME "AlPwr"
*/

// Universal defines

#define VBATPIN A7

// The pause for the loop, and based on testing the actual
// calls overhead take about 20ms themselves E.g. at 50ms delay, 
// that means a 50ms delay, plus 20ms to poll. So 70ms per loop, 
// will get ~14 samples/second.
#define LOOP_DELAY 70

// Min pause How often to crunch numbers and publish an update (millis)
// NOTE If this value is less than the time it takes for one crank
// rotation, we will not report a crank revolution. In other words,
// if the value is 1000 (1 second), cadence under 60 RPM won't register.
#define MIN_UPDATE_FREQ 1500

// NOTE LED is automatically lit solid when connected,
// we don't currently change it, default Feather behavior
// is nice. 
// TODO Though not optimal for power, not sure how much it takes.
#define LED_PIN 33
#define SD_CS_PIN 5

#endif
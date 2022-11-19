
#ifndef POWERMETER_CFG
#define POWERMETER_CFG


/****** General configuration ***********************/

#define DEBUG
//#define BLE_LOGGING
//#define CALIBRATE
#define DISABLE_LOGGING

#define SENSOR_READ_RATE          (100u)
#define BLE_PUBLISH_POWER_RATE    (1000u * 2u)
#define BLE_PUBLISH_BATTERY_RATE  (1000u * 60u * 5u) /* 1000 ms / sec * 60 sec / min * 5 = 5 minutes */


/****** Bike/build configuration *******************/
// Crank length, in meters 
#define CRANK_RADIUS           0.1725    //? duatance from sensor position olr distance to pedal???
#define CRANK_SENSOR_RADIUS    0.10     /* */

#define LOAD_OFFSET     255904.f
#define HX711_MULT      -2466.8989547

#define IMU_GYRO_CALIBRATION_OFFSET     -31
#define IMU_MIN_DPS_MEASUREMENT          0.2 /* d */

#define HOOKEDUPLOADBACKWARDS -1


#define DEV_NAME         "CiafePwr"


/****** HW setup configuration *********************/

/* IMU motion detection Interrupt pin, undefine it to disable feature */
#define IMU_INT_PIN  0

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


#ifndef POWERMETER_CFG
#define POWERMETER_CFG


/****** General configuration ***********************/

#define DEBUG
//#define BLE_LOGGING
//#define CALIBRATE
#define DISABLE_LOGGING

#define SENSOR_READ_RATE          (100u)
#define BLE_PUBLISH_POWER_RATE    (1000u * 1u)
#define BLE_PUBLISH_BATTERY_RATE  (1000u * 60u * 5u) /* 1000 ms / sec * 60 sec / min * 5 = 5 minutes */

#define SPEED_MEASUREMENT_FILTERING  0.8
#define FORCE_MEASUREMENT_FILTERING  0.8

/****** Bike/build configuration *******************/
// Crank length, in meters 
#define CRANK_RADIUS                   0.1725    /* Distance from the cranck center to the pedal, in m */
#define CRANK_SENSOR_RADIUS            0.10      /* */

#define HX711_SENSOR_OFFSET            -1099400  /* Shall be measured with crank without load in vertical position */
#define HX711_MULT_SCALE                85770    /* Shall be measured with horitzontal with a known load in kg */
#define HX711_FORCE_LIMIT_RANGE         500      /* Limit measurement range of Force (avoid spikes detected) */

#define IMU_GYRO_CALIBRATION_OFFSET    -31
#define IMU_MIN_DPS_MEASUREMENT         0.2 /* d */

#define HOOKEDUPLOADBACKWARDS         -1   /* To allow to invert load cell sensor measurement, set 1 or -1 */

#define BLE_DEV_NAME                   "CiafePwr"


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

/* Load cell sensor HX711 sensor pins */
#define HX711_POS_DATA 9
#define HX711_NEG_CLK  8


#endif

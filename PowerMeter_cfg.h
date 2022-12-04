
#ifndef POWERMETER_CFG
#define POWERMETER_CFG


/****** General configuration ***********************/
//#define DEBUG_PRINT_SPEED        /* Enable/Disable debug print measured angular speed */
#define DEBUG_PRINT_FORCE        /* Enable/Disable debug print measured force */
//#define DEBUG_PRINT_REV          /* Enable/Disable debug print measured revolution data */

//#define SD_LOGGING               /* TODO: Enable/Disable logging of sensor data in SD file */

//#define USE_MANUAL_I2C_PINS     /* Allow the usage of GPIO pins for I2C comms */

#define BLE_DEV_NAME              "CiafePwr"
#define BLE_PUBLISH_POWER_RATE    (1000u * 1u)
#define BLE_PUBLISH_BATTERY_RATE  (1000u * 60u * 5u) /* 1000 ms / sec * 60 sec / min * 5 = 5 minutes */

#define SENSOR_READ_RATE          (100u)
#define SENSOR_SPEED_FILTERING    0.9
#define SENSOR_FORCE_FILTERING    1
#define POWER_FILTERING           0.9

#define CADENCE_MIN_VALUE           2  /**/
#define CADENCE_MIN_VALUE_REV      34  /* value in ms of a revolution in the slowest cadence, before to report 0 rpms */

/****** Bike/build configuration *******************/ 
#define CRANK_RADIUS                   0.1725    /* Distance from the crank center to the pedal, in m */
#define CRANK_SENSOR_RADIUS            0.10      /* Distance from the crank center to the IMU sensor in m */

#define HX711_SENSOR_OFFSET            0//-1099400  /* Shall be measured with crank without load in vertical position */
#define HX711_MULT_SCALE               1//85770    /* Shall be measured with horitzontal with a known load in kg */
#define HX711_FORCE_LIMIT_RANGE        500      /* Limit measurement range of Force (avoid spikes detected) */

#define IMU_GYRO_CALIBRATION_OFFSET     0
#define IMU_MIN_DPS_MEASUREMENT         0.2 /* rad/s, equivalent to 2 rpm */
#define IMU_CALIBRATION_SAMPLES         40

#define HOOKEDUPLOADBACKWARDS         -1   /* To allow to invert load cell sensor measurement, set 1 or -1 */


/****** HW setup configuration *********************/

/* IMU motion detection Interrupt pin, undefine it to disable feature */
//#define IMU_INT_PIN     0

/* Load cell sensor HX711 sensor pins */
#define HX711_POS_DATA 39
#define HX711_NEG_CLK  36

/* I2C pins for manual pin allocation */
#ifdef USE_MANUAL_I2C_PINS
#define IMU_I2C_SDA    15
#define IMU_I2C_SCL    14
#endif

#define LED_WHITE_PIN   4  /* configured for ESP32-CAM */
#define LED_BOARD_PIN  33  /* configured for ESP32-CAM */

#define VBATPIN         0 //Not supported yet

#endif

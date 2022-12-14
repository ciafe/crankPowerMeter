
#ifndef POWERMETER_CFG
#define POWERMETER_CFG


/****** General configuration ***********************/
//#define DEBUG_PRINT_SPEED        /* Enable/Disable debug print measured angular speed */
//#define DEBUG_PRINT_FORCE        /* Enable/Disable debug print measured force */
//#define DEBUG_PRINT_REV          /* Enable/Disable debug print measured revolution data */

#define SD_LOGGING                 /* Enable/Disable logging of sensor data in SD file */
#define SD_LOGGING_NUM_REVOLUTIONS_TO_STORE  15

//#define USE_MANUAL_I2C_PINS     /* Allow the usage of GPIO pins for I2C comms */

//#define DISABLE_BLE               /* Enable/Disable BLE compilation, to allow compilation in boards without BLE (Mega, ...) for testing */

#define BLE_DEV_NAME              "CiafePwr"
#define BLE_PUBLISH_POWER_RATE    (1000u * 1u)
#define BLE_PUBLISH_BATTERY_RATE  (1000u * 60u * 5u) /* 1000 ms / sec * 60 sec / min * 5 = 5 minutes */
#define BLE_POWER_FILTER_SAMPLES  5                  /* Avegaraging filter applied to BLE published power, number of published samples averaged */

#define SENSOR_READ_RATE          (20u)
#define SENSOR_SPEED_FILTERING    0.6
#define SENSOR_FORCE_FILTERING    0.6
#define POWER_FILTERING           0.8
#define POWER_CALIBRATION         0.9    /* Factor applied to the measured power to allow measurement calibration. Used to calibrate with Garmin Vector3/3S */

//#define POWER_DISCARD_NEG_FORCES       /* Enable/Disable considering negatives forces during power calculation, 
                                         /* They ara mainly generated by weight leg/foot and do not produce bike power */
#define CADENCE_MIN_VALUE_REV      50    /* value in ms of a revolution in the slowest cadence, before to report 0 rpms */

/****** Bike/build configuration *******************/ 
#define CRANK_RADIUS                   0.1725    /* Distance from the crank center to the pedal, in m */
#define CRANK_SENSOR_RADIUS            0.10      /* Distance from the crank center to the IMU sensor in m */

#define HX711_SENSOR_OFFSET            -1102300  /* Shall be measured with crank without load in vertical position */
#define HX711_MULT_SCALE               8440      /* Shall be measured with horitzontal with a known load in kg, and should report Newtons (Kg * 9,8) */
#define HX711_FORCE_LIMIT_RANGE        5000      /* Limit measurement range of Force (avoid spikes detected) */

#define IMU_GYRO_CALIBRATION_OFFSET     0
#define IMU_MIN_DPS_MEASUREMENT         0.3 /* rad/s, equivalent to x rpm */
#define IMU_CALIBRATION_SAMPLES         40

#define HOOKEDUPLOADBACKWARDS          -1   /* To allow to invert load cell sensor measurement, set 1 or -1 */


/****** HW setup configuration *********************/

/* IMU motion detection Interrupt pin, undefine it to disable feature */
//#define IMU_INT_PIN     0

/* Load cell sensor HX711 sensor pins */
#define HX711_POS_DATA 36 
#define HX711_NEG_CLK  23

/* I2C pins for manual pin allocation */
#ifdef USE_MANUAL_I2C_PINS
#define IMU_I2C_SDA    15
#define IMU_I2C_SCL    14
#endif

#define LED_WHITE_PIN   4  /* configured for ESP32-CAM */
#define LED_BOARD_PIN  33  /* configured for ESP32-CAM */

#define VBATPIN         0 //Not supported yet

#endif

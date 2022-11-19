
#ifndef GYRO_API
#define GYRO_API

#include <Wire.h>
#include <Adafruit_MPU6050.h>

bool imu_Setup(void);

void imu_readData(void);


/**
 * Gets a normalized averaged rotational velocity calculation. The MPU6050 library supports a
 * normalized gyroscope reading, which trims off outliers and scales the values to deg/s.
 *
 * An exponential average is applied to further smooth data, based on filter applied [0-1]. 
 *
 * Returns a value for foot speed, in degrees/second.
 */
float imu_getNormalAvgVelocity(float lastAvg, const double filter);


/**
 * Provide the linear speed of the pedal, end of the ckank shaft, in m/second.
 *
 * Returns the circular velocity of the rider's foot. Takes in the crank's averaged rotational
 * velocity, converts it to radians, multiplies by the crank radius, and returns the converted
 * value.
 *
 * Value returned is in meters/second
 */
float  imu_getCrankCircularVelocity(float dps);

/**
 *  Provide cadence of crank pedal, rotations/minute.
 *
 *  Note this isn't necessary for power measurement, but it's a gimme addon
 *  given what we already have and useful for the athlete.
 *
 *  Returns an int16 of cadence, rotations/minute.
 */
float  imu_getCrankCadence(float dps);


double imu_getCrankAngle(void);

void   imu_print_data(void);

#endif

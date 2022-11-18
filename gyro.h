
#ifndef GYRO_API
#define GYRO_API

#include <Wire.h>
#include <Adafruit_MPU6050.h>

extern bool imu_Setup(void);

extern void imu_readData(void);


/**
 * Gets a normalized averaged rotational velocity calculation. The MPU6050 library supports a
 * normalized gyroscope reading, which trims off outliers and scales the values to deg/s.
 *
 * An exponential average is applied to further smooth data, based on filter applied [0-1]. I don't
 * love this, becaues it means no value is every entirely discarded, but exponential decay
 * probably makes it effectively the same. Maybe something to revisit.
 *
 * Returns a value for foot speed, in degrees/second.
 */
extern float imu_getNormalAvgVelocity(float lastAvg, const double filter);


/**
 * Provide the average rate, in degrees/second.
 *
 * Returns the circular velocity of the rider's foot. Takes in the crank's averaged rotational
 * velocity, converts it to radians, multiplies by the crank radius, and returns the converted
 * value.
 *
 * Value returned is in meters/second
 */
float  imu_getCrankCircularVelocity(float dps);

float  imu_getCrankCadence(float dps);

double imu_getCrankAngle(void);

void   imu_print_data(void);

#endif

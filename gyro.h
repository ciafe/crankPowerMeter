
#ifndef GYRO_API
#define GYRO_API

#include <Wire.h>
#include <Adafruit_MPU6050.h>

extern void gyroSetup();

extern float getNormalAvgVelocity(const float & lastAvg);
extern float getCircularVelocity(float dps);

#endif

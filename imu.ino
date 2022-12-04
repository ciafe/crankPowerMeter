/**
 * MPU6050 specific code. Initialize, helpers to do angular math.
 *
 */
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu_sensor;

#ifdef USE_MANUAL_I2C_PINS
TwoWire I2Cmpu = TwoWire(0);
#endif

sensors_event_t last_acc, last_g, last_temp;

/**
 *  Initialize the gyroscope
 */
bool imu_Setup(void)
{
#ifdef ESP32_BOARD_I2C
  I2Cmpu.begin(_IMU_I2C_SDA, IMU_I2C_SCL, 100000);
#endif

  // Try to initialize!
#ifdef ESP32_BOARD_I2C
  if (!mpu_sensor.begin(MPU6050_I2CADDR_DEFAULT, &I2Cmpu, 0))
#else
  if (!mpu_sensor.begin()) 
#endif
  {
    return(false); //TODO: if init fails, block next API and report always 0
  }
  else
  {
    /* Configure sensor */
    mpu_sensor.setGyroRange(MPU6050_RANGE_1000_DEG); //check improvement to change MPU6050_RANGE_500_DEG
    mpu_sensor.setFilterBandwidth(MPU6050_BAND_44_HZ); // test even with lowe filters, low speed Gyro application
    mpu_sensor.setAccelerometerRange(MPU6050_RANGE_2_G);
	
    // Set that calibration -- NOT supported by Adafruit
    //gyro.setZGyroOffset(IMU_GYRO_CALIBRATION_OFFSET);

    //Todo, does it make sense to use cycle mode?
    
    /* configure motion detection Interrupt - If configured */
	#if defined(IMU_INT_PIN)
    mpu_sensor.setMotionInterrupt(true);
  	mpu_sensor.setInterruptPinLatch(true);
	  mpu_sensor.setInterruptPinPolarity(false); /* Active HIGH, Rising INT */
    mpu_sensor.setMotionDetectionThreshold(4);
    mpu_sensor.setMotionDetectionDuration(80);  /* 1 LSB = 64ms. So 30s =  */
    
    pinMode(IMU_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imu_motionDetectInterrupt, RISING);
	#else
	  mpu_sensor.setMotionInterrupt(false);
  #endif
	
	return(true);
  }

}

void imu_motionDetectInterrupt(void)
{
  uint8_t motion = mpu_sensor.getMotionInterruptStatus();
  // TODO note really clear what our power options actually are here..
  if (motion) {
    Serial.println("Go to sleep..");
  } else {
    Serial.println("Wakey wakey let's get crankey.");
  }
}

void imu_readData(void)
{
  /* Get IMU sensor data */
  mpu_sensor.getEvent(&last_acc, &last_g, &last_temp);
}

double imu_getNormalAvgVelocity(double lastAvg, const double filter)
{
  /* Take Z giro rad/s data: due to sensor mount in the bike, Z is the measure of the rotation of the shaft */
  /* Use the absolute value, just to discard the mounting side of the sensor (left/rigth shaft arm). */
  double rotz = abs(last_g.gyro.z);
  if (rotz < IMU_MIN_DPS_MEASUREMENT) {
    /* Magic number here, but less than 90 dps is less than 1 crank rotation 
       in 4 seconds (15 RPM), just assume it's noise from the road bumps. */
    rotz = 0.f;
  }
  /* Library reports rad/s, Convert to deg/s */
  rotz = (180 * rotz) / PI;
  
  /* Return a rolling average filtered value */
  double newavg = (rotz * filter) + (lastAvg * (1.0 - filter));

  return newavg;
}


double imu_getCrankCircularVelocity(double dps) {
  /* Get angular velocity based on sensor velocity and distance from the center of the cranck shaft to the sensor:
  
     2 * PI * radians = 360 degrees
     dps degrees/second * (PI / 180) rad/degree = rad/second
     (rad/sec) / (rad/circumference) = (rad/sec) / 2 * PI = ratio of a circumference traveled, still per second
     2 * PI * CRANK_RADIUS = circumference  -- by definition, that's a circle
     ratio of circumference traveled * circumference = meters/second
    
     It all comes out to:
     m/s = ((dps * (PI / 180)) / (2 * PI)) * (2 * PI * CRANK_RADIUS);
	   m/s = (dps * PI * CRANK_RADIUS) / 180;
  
  */
  return (dps * PI * CRANK_RADIUS) / 180;
}


double imu_getCrankCadence(double dps)
{
  /* Cadence is the normalized angular velocity, times 60/360, which
     converts from deg/s to rotations/min. x * (60/360) = x / 6. */
  return dps / 6;
}

/**
 *  Determine current angle of the crank arm. Based on the acceleration
 *  for gravity.
 */
double imu_getCrankAngle(void)
{
  // angle measurement from gravitaional acceleration
  // centripetal acceleration acceleration is accaunted for
  // the angle is adjusted to match the gatt specifications with fmod
  double angle = fmod(((atan2(last_acc.acceleration.y + (last_g.gyro.z * last_g.gyro.z * CRANK_SENSOR_RADIUS), last_acc.acceleration.x)) * 360 / 6.28) + 180, 360.0);  

  return angle;
}


void imu_calibrate_reading(void)
{
#if 0 //TODO
  /* Calibrate the gyro */
  gyro.setZGyroOffset(0);
  float sumZ = 0;
  int16_t maxSample = -32768;
  int16_t minSample = 32767;
  // Read n-samples
  for (uint8_t i = 0; i < IMU_CALIBRATION_SAMPLES; ++i) {
    delay(5);
    int16_t reading = gyro.getRotationZ();
    if (reading > maxSample) maxSample = reading;
    if (reading < minSample) minSample = reading;
    sumZ += reading;
  }

  // Throw out the two outliers
  sumZ -= minSample;
  sumZ -= maxSample;

  // Two fewer than the calibration samples because we took out two outliers.
  float deltaZ = sumZ / (CALIBRATION_SAMPLES - 2);
  deltaZ = -1 * deltaZ;
#ifdef DEBUG
  Serial.printf("Discounting max (%d) and min (%d) samples.\n", maxSample, minSample);
  Serial.printf("Gyro calculated offset: %f\n", deltaZ); 
#endif // DEBUG

#endif

}

void imu_print_data(void)
{
  /* Print out sensor values */
  Serial.print("Acceleration X: ");
  Serial.print(last_acc.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(last_acc.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(last_acc.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(last_g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(last_g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(last_g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(last_temp.temperature);
  Serial.println(" degC");

  Serial.println("");
}

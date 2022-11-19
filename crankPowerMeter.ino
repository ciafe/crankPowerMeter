/**
 * Main
 */

#include "PowerMeter_cfg.h"
#include "imu.h"
#include "loadCell.h"
#include "ble.h"

/******* Local function ******************************/
uint8_t checkBatt();
int16_t calcPower(double footSpeed, double force);

/******* Global variable for the project ************/
/* Speeds/revolutions data variable */
static double dps = 0.f;
static double avgDps = 0.f;
static double angSpeed = 0.f;
static float cadence = 0.f;
static float crankAngle = 0.f;
static float revElapsedDegrees = 0.f;
static uint32_t totalCrankRevs = 0;

/* orce data variables */
static double force = 0.f;
static double avgForce = 0.f;
static const float MIN_DOUBLE = -100000.f;
static const float MAX_DOUBLE = 100000.f;
static double maxForce = MIN_DOUBLE;
static double minForce = MAX_DOUBLE;
static int16_t numPolls = 0;

/* Time data variables */
static long lastUpdateSensor = 0u;
static long lastUpdatePowerCalc = 0u;
static long lastUpdateBlePower = 0u;
static long lastUpdateBleBatt = 0u;

/******* Setup *************************************/
void setup() 
{
  Serial.begin(115200);

  /* Setup sensors */
  if (!imu_Setup()){
    Serial.print(F("Imu device not found... check wiring."));
  }
  load_Setup();
  ble_Setup();

#ifdef DEBUG
  Serial.println(F("All setup complete."));
#endif
}

void loop()
{
  get_sensor_data();
  
  calculate_power();

  publish_ble_data();

}

void get_sensor_data(void)
{
  long timeNowSensor, timeSinceLastUpdate;
  timeNowSensor = millis();
  timeSinceLastUpdate = timeNowSensor - lastUpdateSensor;
  
    /* Wait for sensor reading Rate */
  if (timeSinceLastUpdate > SENSOR_READ_RATE)
  {
    /* Get updated data from sensor */
    imu_readData();
    /* Convert sensor data to degree/sec */
    dps        = imu_getNormalAvgVelocity(dps, SPEED_MEASUREMENT_FILTERING);
    /* Convert sensor data to degree/sec */
    angSpeed   = imu_getCrankCircularVelocity(dps);
    /* Convert sensor data to rpm/sec */
    cadence    = imu_getCrankCadence(dps);
    /* Convert sensor data to crank position angle */
    crankAngle = imu_getCrankAngle();

    lastUpdateSensor = timeNowSensor;
  }
}

void calculate_power(void)
{
  long timeNowPowerCalc, timeSinceLastUpdate;
  float speedMps;
  int16_t revPower;
  timeNowPowerCalc = millis();
  timeSinceLastUpdate = timeNowPowerCalc - lastUpdatePowerCalc;
  
  /* Wait for sensor reading Rate */
  if (timeSinceLastUpdate > SENSOR_READ_RATE)
  {
    /* Get count of number of samples used for averaging */
    numPolls += 1;
    /* Average speed */
    avgDps += dps;
    /* Now get force from the load cell */
    force = load_getAvgForce(force, FORCE_MEASUREMENT_FILTERING);
    /* Calculate and store the max and min. */
    if (force > maxForce) {
      maxForce = force;
    }
    if (force < minForce) {
      minForce = force;
    }
    /* Average force */
    avgForce += force;
    /* Get elapsed degrees with current speed and elapsed time */
    revElapsedDegrees += getDegreesFromSpeed(dps, timeSinceLastUpdate);

    /* Is a new revolution completed? let's calculate the average Power */
    if (revElapsedDegrees >= 360.0)
    {
      /* If some degrees ha been moved, just store them for the next round */
      revElapsedDegrees = 360.0 - revElapsedDegrees;
      /* Keep count of all revolutions done */
      totalCrankRevs += 1;
      /* Calculate the actual averages over the polling period. */
      avgDps = avgDps / numPolls;
      /* Subtract 2 from the numPolls for force because we're removing the high and low here.*/
      avgForce = avgForce - minForce - maxForce;
      avgForce = avgForce / (numPolls - 2);
      /* Convert dedrees/s to m/s */
      speedMps = imu_getCrankCircularVelocity(avgDps);
      /* Let's calculate the power used for the completed revolution */
      revPower = calcPower(speedMps, avgForce);
      /* Reset averages from this polling period just carry over. */
      numPolls = 0;
      maxForce = MIN_DOUBLE;
      minForce = MAX_DOUBLE;
    }
    lastUpdatePowerCalc = timeNowPowerCalc;
  }
}

void publish_ble_data(void)
{
  long timeNowBle, timeSinceLastUpdate;
  
  /* Any user connected? */
  if (ble_isConnected())
  {
    timeNowBle = millis();
    timeSinceLastUpdate = timeNowBle - lastUpdateBlePower;
    /* Wait for publish Power Rate */
    if (timeSinceLastUpdate > BLE_PUBLISH_POWER_RATE)
    {
      /* TODO: get power and cadence */
      ble_PublishPower(0,0, timeNowBle);
      lastUpdateBlePower = timeNowBle;

      //just for debug, to delete
      Serial.print("Deg: ");Serial.print(revElapsedDegrees);
      Serial.print(" Speed: ");Serial.print(avgDps);
      Serial.print(" REV: ");Serial.println(totalCrankRevs);
    }

    timeSinceLastUpdate = timeNowBle - lastUpdateBleBatt;
    /* Wait for publish Battery Rate */
    if (timeSinceLastUpdate > BLE_PUBLISH_BATTERY_RATE)
    {
      float batPercent = checkBatt();
      ble_PublishBatt(batPercent);
      lastUpdateBleBatt = timeNowBle;
    }
  }
}

/***** Local functions ***************************************************************************/

/**
 * Calculate the number of degrees moved dureing the elapsedTime at the speed provided by 
 * speedDegps. (elapsedTime units are ms, and speedDegps units deg/s
 */
float getDegreesFromSpeed(float speedDegps, long elapsedTime)
{
  return (float)((speedDegps * elapsedTime)/1000);
}

/**
 * Rolling average for power reported to head unit.
 * Pass the current reading and the float to weight it.
 * Weight is the weight given to the current, previous will
 * have 1.0-weight weight.
 */
int16_t rollAvgPower(int16_t current, float weight) {
  static int16_t prevAvg = 0;  // Initially none
  int16_t rollingAvg = (weight * current) + ((1.0 - weight) * prevAvg);
  prevAvg = rollingAvg;
  return rollingAvg;
}

/**
 * Figure out how long to sleep in the loops. We'd like to tie the update interval
 * to the cadence, but if they stop pedaling need some minimum.
 *
 * Return update interval, in milliseconds.
 */
float updateTime(float dps, bool *pedaling) {
  // So knowing the dps, how long for 360 degrees?
  float del = min((float)MIN_UPDATE_FREQ, (float)(1000.0 * (360.0 / dps)));
  if (del < MIN_UPDATE_FREQ) {
      // Let the caller know we didn't just hit the max pause,
      // the cranks are spinning.
      *pedaling = true;
  }
  // Because need to account for delay, on average get 1 rotation. Empirically the overhead
  // for calls in the loop is about 20 ms, plus the coded loop delay. If we account for half of that,
  // we should be on average right on 1 rotation. We want to be within half of the overhead time
  // for a perfect 360 degrees.
  return (del - (0.5 * (LOOP_DELAY + 30)));
}

/**
 * Given the footspeed (angular velocity) and force, power falls out.
 *
 * Returns the power, in watts. Force and distance over time.
 */
int16_t calcPower(double footSpeed, double force) {
  // Multiply it all by 2, because we only have the sensor on 1/2 the cranks.
  return (2 * force * footSpeed);
}

/**
 * 
 */
uint8_t checkBatt() {     
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // Board divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    // TODO would be cool to convert to an accurate percentage, but takes 
    // some science because I read it's not a linear discharge. For now
    // make it super simple, based off this info from Adafruit:
    // 
    // Lipoly batteries are 'maxed out' at 4.2V and stick around 3.7V 
    // for much of the battery life, then slowly sink down to 3.2V or 
    // so before the protection circuitry cuts it off. By measuring the 
    // voltage you can quickly tell when you're heading below 3.7V
    if (measuredvbat > 4.1) {
      return 100;
    } else if (measuredvbat > 3.9) {
      return 90;
    } else if (measuredvbat > 3.7) {
      return 70;
    } else if (measuredvbat > 3.5) {
      return 40;
    } else if (measuredvbat > 3.3) {
      return 20;
    } else {
      return 5;
    }
}

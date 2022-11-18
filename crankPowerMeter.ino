/**
 * Main
 */
 #include <SD.h>
#include <SPI.h>

#include "PowerMeter_cfg.h"
#include "gyro.h"
#include "loadCell.h"
#include "ble.h"

uint8_t checkBatt();
int16_t calcPower(double footSpeed, double force);

static float cadence = 0;
static float dps = 0.f;
static float angSpeed = 0.f;
static float circular = 0.f;
static double angle = 0;

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
  imu_readData();
  //imu_print_data();
  
  dps      = imu_getNormalAvgVelocity(dps, 0.7);
  angSpeed = imu_getCrankCircularVelocity(dps);
  cadence  = imu_getCrankCadence(dps);
  angle    = imu_getCrankAngle();

  //Serial.print(F("Speed: ")); 
  //Serial.print(dps);Serial.print(",");
  Serial.println(100 * angSpeed);
  //Serial.print(cadence);Serial.println(";");
  //Serial.print(F("Angle: ")); Serial.println(angle);
  delay(50);

  
#if 0
  // These aren't actually the range of a double, but
  // they should easily bookend force readings.
  static const float MIN_DOUBLE = -100000.f;
  static const float MAX_DOUBLE = 100000.f;

  // Vars for polling footspeed
  static float dps = 0.f;
  static float avgDps = 0.f;
  // Cadence is calculated by increasing total revolutions.
  // TODO it's possible this rolls over, about 12 hours at 90RPM for 16 bit unsigned.
  static uint16_t totalCrankRevs = 0;
  // Vars for force
  static double force = 0.f;
  static double avgForce = 0.f;
  // Track the max and min force per update, and exclude them.
  static double maxForce = MIN_DOUBLE;
  static double minForce = MAX_DOUBLE;
  // We only publish every once-in-a-while.
  static long lastUpdate = millis();
  // Other things (like battery) might be on a longer update schedule for power.
  static long lastInfrequentUpdate = millis();
  // To find the average values to use, count the num of samples
  // between updates.
  static int16_t numPolls = 0;

  // During every loop, we just want to get samples to calculate
  // one power/cadence update every interval we update the central.

  // Degrees per second
  dps = load_getNormalAvgVelocity(dps, 0.9);
  avgDps += dps;

  // Now get force from the load cell.
  force = load_getAvgForce(force, 0.8);
  // We wanna throw out the max and min.
  if (force > maxForce) {
    maxForce = force;
  }
  if (force < minForce) {
    minForce = force;
  }
  avgForce += force;

  numPolls += 1;


#ifdef DEBUG
  // Just print these values to the serial, something easy to read.
  Serial.print(F("Force: ")); Serial.println(force);
  Serial.print(F("DPS:   ")); Serial.println(dps);
#endif  // DEBUG

  if (ble_isConnected())
  {
    // We have a central connected
    long timeNow = millis();
    long timeSinceLastUpdate = timeNow - lastUpdate;
    // Must ensure there are more than 2 polls, because we're tossing the high and low.
    // Check to see if the updateTime fun determines the cranks are cranking (in which)
    // case it'll aim to update once per revolution. If that's the case,
    // increment crank revs.
    bool pedaling = false;
    if (timeSinceLastUpdate > updateTime(dps, &pedaling) && numPolls > 2) {
      // Find the actual averages over the polling period.
      avgDps = avgDps / numPolls;
      // Subtract 2 from the numPolls for force because we're removing the high and
      // low here.
      avgForce = avgForce - minForce - maxForce;
      avgForce = avgForce / (numPolls - 2);

      // Convert dps to mps
      float mps = getCircularVelocity(avgDps);

      // That's all the ingredients, now we can find the power.
      int16_t power = calcPower(mps, avgForce);

      // Also bake in a rolling average for all records reported to
      // the head unit. Will hopefully smooth out the power meter
      // spiking about.
      //power = rollAvgPower(power, 0.7f);

#ifdef DEBUG
  // Just print these values to the serial, something easy to read.
  Serial.print(F("Pwr: ")); Serial.println(power);
#endif  // DEBUG

      // The time since last update, as published, is actually at
      // a resolution of 1/1024 seconds, per the spec. BLE will convert, just send
      // the time, in millis.
      if (pedaling) {
        totalCrankRevs += 1;
      }


      // Reset the latest update to now.
      lastUpdate = timeNow;
      // Let the averages from this polling period just carry over.
      numPolls = 1;
      maxForce = MIN_DOUBLE;
      minForce = MAX_DOUBLE;

      // And check the battery, don't need to do it nearly this often though.
      // 1000 ms / sec * 60 sec / min * 5 = 5 minutes
      if ((timeNow - lastInfrequentUpdate) > (1000 * 60 * 5)) {
        float batPercent = checkBatt();
       //TODO blePublishBatt(batPercent);
        lastInfrequentUpdate = timeNow;
      }
    }
  }
  delay(LOOP_DELAY);
#endif 

}

/***** Local functions ***************************************************************************/

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

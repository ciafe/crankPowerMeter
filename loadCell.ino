/**
 * Force and load cell-specific code and helpers. HX711 chip.
 */
#include "HX711.h"

HX711 loadsensor;

void load_Setup() {
  loadsensor.begin(HX711_POS_DATA, HX711_NEG_CLK);
  /* Set the scale for the multiplier to get grams */
  loadsensor.set_scale(HX711_MULT_SCALE);
  /* Set the offset of the sensor when unloaded */
  loadsensor.set_offset(HX711_SENSOR_OFFSET);
  /* Data filtering in handeled in this file, no need tu use library filtering modes */
  loadsensor.set_raw_mode();
  /* Enable sensor */
  loadsensor.power_up();

}


double load_getAvgForce(const double & lastAvg, const double filter) {
  static double currentData = 0;

  currentData = loadsensor.get_units() * HOOKEDUPLOADBACKWARDS;

  /* If measured data is out of range just discard and return previous value (sporadically, raw value 0 is read) */
  /*if ((currentData > HX711_FORCE_LIMIT_RANGE) || (currentData < (-1 * HX711_FORCE_LIMIT_RANGE)))
  {
    currentData = lastAvg;
  }*/

  // Return a rolling average, including the last avg readings.
  // e.g. if weight is 0.90, it's 10% what it used to be, 90% this new reading.
  return (currentData * filter) + (lastAvg * (1 - filter));
}


double load_getRawData(void) {
  static double currentData = 0;

  currentData = loadsensor.get_value() * HOOKEDUPLOADBACKWARDS;

  return (currentData);
}

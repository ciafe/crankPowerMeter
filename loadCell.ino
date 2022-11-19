/**
 * Force and load cell-specific code and helpers. HX711 chip.
 */
#include "HX711.h"

// This offset value is obtained by calibrating the scale with known
// weights, currently manually with a separate sketch.

// It's 'm' as in:
// 'y = m*x + b'
// where 'y' is Newtons (our desired answer, kilograms at
// acceleration of gravity), 'x' is the raw reading of the load
// cell, and 'b' is the tare offset. So this multiplier is the
// scale needed to translate raw readings to units of Newtons.
// (defined in main file)

// Call tare to average this many readings to get going.
// NOTE: 30 takes kind of long, like > 1 second, but it definitely
// dials in better >20 in testing.
#define NUM_TARE_CALLS 50
// How many raw readings to take each sample.
#define NUM_RAW_SAMPLES 1

// Pins we're using.
#define EXCIT_POS_DATA 8
#define EXCIT_NEG_CLK  10


HX711 loadsensor;

void load_Setup() {
  // 'load' is declared in power.ini
  loadsensor.begin(EXCIT_POS_DATA, EXCIT_NEG_CLK);
  // Set the scale for the multiplier to get grams.
  loadsensor.set_scale(HX711_MULT);
  // Lots of calls to get load on startup, this is the offset
  // that will be used throughout. Make sure no weight on the
  // pedal at startup, obviously.

#ifdef CALIBRATE
  loadsensor.tare(NUM_TARE_CALLS); 
#else
  float offset = LOAD_OFFSET;
  loadsensor.set_offset(offset);
#endif
  
  loadsensor.power_up();

#ifdef DEBUG
  load_showConfigs();
#endif
}


void load_showConfigs(void) {
//  Serial.println();
//  Serial.printf(" * Load offset:       %d\n", loadsensor.get_offset());
//  Serial.printf(" * Load multiplier:   %.3f\n", loadsensor.get_scale());
//  Serial.println("Power meter calibrated.");
}


double load_getAvgForce(const double & lastAvg, const double filter) {
  static double currentData = 0;

  currentData = loadsensor.get_units(NUM_RAW_SAMPLES) * HOOKEDUPLOADBACKWARDS;

  // Return a rolling average, including the last avg readings.
  // e.g. if weight is 0.90, it's 10% what it used to be, 90% this new reading.
  return (currentData * filter) + (lastAvg * (1 - filter));
}

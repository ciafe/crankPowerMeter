
#ifndef LOADCELL_API
#define LOADCELL_API


#include "HX711.h"

extern void load_Setup();

/**
 * Get the current force from the load cell. Returns an exponentially
 * rolling average (based on filter applied [0-1]), in Newtons.
 */
extern double load_getAvgForce(const double & lastAvg, const double filter);



#endif

#ifndef LOADCELL_API
#define LOADCELL_API


#include "HX711.h"

extern void loadSetup();

extern double getAvgForce(const double & lastAvg);



#endif
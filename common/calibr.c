#include "calibr.h"
#include <avr/pgmspace.h>
#include <avr/io.h>



//const uint8_t cor[] PROGMEM= {0, 3, 6, 10, 13, 17, 20, 24, 27, 30, 34, 37, 41, 44, 47, 51, 54, 58, 61, 64, 68, 71, 75, 78, 81, 85, 88, 91, 95, 98, 101, 105, 108, 111, 115, 118, 121, 125, 128, 131, 135, 138, 141, 144, 148, 151, 154, 158, 161, 164, 167, 171, 174, 177, 180, 184, 187, 190, 193, 197, 200, 203, 206, 209, 213, 216, 219, 222, 225, 229, 232, 235, 238, 241, 245, 248, 251, 254, 255, 255};

#define a 7.5
#define b 237.3

inline double SDDD(double t,double x) {
	 //volatile double p=((a*t/(b+t)-(a*t+a*x)/(b+t+x)))*2000.0;
	//return ((p*3.3)/3000.0)+1;
	//return ((3.5*((a*t/(b+t)-(a*t+a*x)/(b+t+x)))*2000.0)/3000.0+1);
	return 1 - (2.33333 *a* b* x)/((b + t)* (b + t + x));
}

double calibr_hum(double temp,double tempdiv,double hum) {
	 double r=hum*(SDDD(temp,tempdiv)*(tempdiv/(temp+273.15)+1));
	 if (r>100) return 100.0;
	return r;
}

inline double calibr_hum05(double t,double hum) {
	double y=-0.0006*t*t-0.2455*t-28.5902;
	return -(hum/y)+hum;
}
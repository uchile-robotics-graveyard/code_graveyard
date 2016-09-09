#include "tracking/laser_tracker/utilities.h"

double Deg2Rad(double DEG){
	return DEG*Math_PI/180.0;
}

double Rad2Deg(double RAD){
	return RAD*180.0/Math_PI;
}

double PolarDistance(double r1, double a1, double r2, double a2) {
	return sqrt(r1*r1+r2*r2-2*r1*r2*cos(Deg2Rad(a2-a1)));
}

#pragma once
#include <stdio.h>
#include <math.h>
#define Math_PI 3.1415926535897932384626433832795

#define N135  (120)

double Deg2Rad(double DEG);
double Rad2Deg(double RAD);
double PolarDistance(double r1, double a1, double r2, double a2);

//enum TrackingStatus {
//	TRACK_OK,
//	TRACK_LOST,
//	TRACK_OCCLUSION_RISK,
//	TRACK_OCCLUSION,
//	TRACK_RECOVER,
//	TRACK_NOT_AVAILABLE
//};

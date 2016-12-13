#include "tracking/laser_tracker/SegmentFeatures.h"
SegmentFeatures::SegmentFeatures(){

}

SegmentFeatures::~SegmentFeatures(){

}

float SegmentFeatures::getMeanAngle() {
	return meanAngle;
}

float SegmentFeatures::getMeanRadius() {
	return meanRadius;
}

float SegmentFeatures::getDeltaAngle() {
	return deltaAngle;
}


float SegmentFeatures::distanceTo(float ma, float mr, float da){
	float distMa = (ma-meanAngle);
	if(meanAngle != 0)
		distMa /= meanAngle;
	else
		distMa /= 0.3f;  // No hay mucha diferencia entre 0 y 0.3 dada la resolucion del laser

	float distMr = (mr-meanRadius);
	if(meanRadius != 0)
		distMr /= meanRadius;
	else
		distMr /= 1;  // No hay mucha diferencia entre 0 y 1 dada la precision del laser


	float distDa = (da-deltaAngle);
	if(deltaAngle != 0)
		distDa /= deltaAngle;
	else
		distDa /= 0.3f;  // No hay mucha diferencia entre 0 y 0.3 dada la resolucion del laser

	return distMa*distMa + distMr*distMr + distDa*distDa;
}
void SegmentFeatures::updateTo(float ma, float mr, float da) {
	meanAngle = ma;
	meanRadius = mr;
	deltaAngle = da;
}
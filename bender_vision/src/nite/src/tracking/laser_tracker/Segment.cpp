#include "tracking/laser_tracker/Segment.h"
#include "tracking/laser_tracker/utilities.h"
#include "math.h"

Segment::Segment(void)
{
}

Segment::~Segment(void)
{
}

void Segment::addPoint(double angle, long rad) {
		distances[angle] = rad;
}

bool Segment::hasAngle(double angle) const {
	return getMinAngle() <= angle && angle <= getMaxAngle();
}

bool Segment::hasData() const {
	return distances.size() > 0;
}

double Segment::getMinAngle() const {
	return distances.begin()->first;
}

double Segment::getMaxAngle() const {
	return distances.rbegin()->first;
}

long Segment::getMinRadius() const {
	return distances.begin()->second;
}

long Segment::getMaxRadius() const {
	return distances.rbegin()->second;
}

double Segment::getExtent() const {
	if(!hasData()) return 0;

    double minAngle = getMinAngle();
    double maxAngle = getMaxAngle();
	double minRad = getMinRadius();
	double maxRad = getMaxRadius();

	return sqrt(minRad*minRad+maxRad*maxRad-2*minRad*maxRad*cos(Deg2Rad(maxAngle-minAngle)));
}

long Segment::getMaxDistance() const {
	std::map<double,long>::const_iterator it;
	long foundMax = -1;

	for ( it=distances.begin() ; it != distances.end(); it++ ) {
		if(it->second > foundMax) foundMax = it->second;
    }

	return foundMax;
}

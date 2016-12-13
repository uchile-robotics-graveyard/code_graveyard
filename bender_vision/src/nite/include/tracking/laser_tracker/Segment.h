#pragma once

#include <map>

class Segment
{
public:
	Segment(void);
	~Segment(void);
	std::map<double,long> distances;
	void   addPoint(double angle, long rad);
	bool   hasAngle(double angle) const;
	double getMinAngle() const;
	double getMaxAngle() const;
	long   getMinRadius() const;
	long   getMaxRadius() const;
	bool   hasData() const;
	double getExtent() const;
	long   getMaxDistance() const;
};

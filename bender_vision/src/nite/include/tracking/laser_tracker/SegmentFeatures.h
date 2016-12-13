#pragma once

class SegmentFeatures{
public:
	SegmentFeatures();
	~SegmentFeatures();
	float distanceTo(float ma, float mr, float da);
	void updateTo(float ma, float mr, float da);

	float getMeanAngle();
	float getMeanRadius();
	float getDeltaAngle();
private:
	float meanAngle, meanRadius, deltaAngle;
};
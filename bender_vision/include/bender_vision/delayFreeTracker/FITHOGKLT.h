#include <vector>
#include <opencv2/core/core.hpp>
#include "KLT/klt.h"
#include "CornerTrack.h"
#include "FITHOG.h"

#ifndef FITHOGKLT_H_
#define FILHOGKLT_H_

class FITHOGKLT : public FITHOG
{
public:
	FITHOGKLT();
	void clear();
	void processFrame(cv::Mat &img);
	void doAllTests();

	~FITHOGKLT();

private:
	cv::Mat lastImg;
	KLT_TrackingContext tc;
	KLT_FeatureList fl;
};

#endif // FITHOGKLT_H_

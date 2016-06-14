#ifndef FEATURE_GENERATOR_H_
#define FEATURE_GENERATOR_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>
#include "opencv2/opencv.hpp"
#include <vector>
#include "funciones_comp.h"
#include "RectFrame.h"

using namespace cv;
using namespace std;

class FeatureGenerator
{
public:
	vector<float> histLBP;
	vector<float> histRGB;
	vector<float> histHS;
	Rect bb;

	bool computeFrom(Mat &im, Rect bbx,const char *nomImg);

	vector<float> getFeatures(const FeatureGenerator &other);
	static int getSize() {return 7;} // 8+1

	vector<float> getFeaturesRed(const FeatureGenerator &other);
	static int getSizeRed() {return 2;} //6// 8+1
};

bool is_valid(Rect bbx);
Rect repairBB(Rect bbx);
Point center_of_mass(Rect bbx);

#endif // FEATURE_GENERATOR_H_

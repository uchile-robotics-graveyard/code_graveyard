#ifndef DETECTION_MANAGER_H_
#define DETECTION_MANAGER_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>
#include "opencv2/opencv.hpp"
#include "FeatureGenerator.h"
#include "CornerTrack.h"

class DetectionManager
{
public:
	DetectionManager();
	//void updateUsingNewDetection(cv::Rect HoG1k, cv::Rect HoGnk, cv::Mat &im1k, cv::Mat &imnk, int frame1k, float score1k);
	void updateUsingNewDetection(cv::Rect HoG1k, cv::Rect HoGnk, bool validFeat);
	void clear();

	cv::Rect BestHogEstim() {if (HoGnA.x > 0) return HoGnA; return HoG1A;}

	bool getOcclusionScore_mbr(bool gtCorrect, std::vector<CornerTrack> &tracks, double &m, double &b, double &r, double &p);

	cv::Rect HoG1A;
	cv::Rect HoGnA;
};

#endif // DETECTION_MANAGER_H_

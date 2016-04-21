#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <cstdio>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>
#include "opencv2/opencv.hpp"
#include "KLT/klt.h"
#include "CornerTrack.h"
#include <opencv2/ml/ml.hpp>
#include "DetectionManager.h"
#include "occlStateMach.h"

#include "funciones_comp.h"

#ifndef FITHOG_H_
#define FITHOG_H_

typedef struct Rect_Hog{
    double x;
    double y;
    double width;
    double height;
    double w;
}Rect_Hog;


class FITHOG
{
public:

	
	virtual void clear() = 0;
	virtual void processFrame(cv::Mat &img) = 0;
	virtual void doAllTests() = 0;
	FITHOG(){
				//this->clear();
				manager.clear();
				tracks.clear();
				selected.clear();
				initialized = false;
				frame = 0;
				HOG_Ant.x = -1;

				std::string pathbender = ros::package::getPath("bender_vision");
				std::string db = "/config/hog/Model/";

				std::string svm = pathbender+db+"learned_svm.svm";
				std::string svm_ocl = pathbender+db+"learned_svm_ocl.svm";

				SVM.load(svm.c_str()); // Errores aislados
				SVMOcl.load(svm_ocl.c_str()); // Oclusiones largas
			}

	enum UseScale {transl, transl_scale, scale, no_transf};
	enum ApplyCorrection{always,when_changed,no_corr};
	enum OcclusionHandling {no_ocl, long_ocl, short_ocl, short_long_ocl, longvit_ocl, short_longvit_ocl};
	enum ErrorHandling {no_errh, use_errh};

	int forwardInTimeRANSAC_T(cv::Rect oldHOG, cv::Rect &newHOG);
	int forwardInTimeRANSAC_TE(cv::Rect oldHOG, cv::Rect &newHOG);
	int forwardInTimeRANSAC_TTE(cv::Rect oldHOG, cv::Rect &newHOG);
	int do_none(cv::Rect oldHOG, cv::Rect &newHOG);

	void drawTracks(cv::Mat &img);
	static double score(cv::Rect rect1, cv::Rect rect2);

	int numIn(cv::Rect rect);
	int numOut(cv::Rect rect);

	// Requiere HoG + GT + SVM
	Rect benchmarkFITSVM(Mat &image,Rect HOG, UseScale adaptScale, ApplyCorrection when, OcclusionHandling occlh, ErrorHandling errh, bool hogDetect);

	
	virtual ~FITHOG() {}

protected:
	DetectionManager manager;
	std::vector<CornerTrack> tracks;
	std::vector<CornerTrack> selected;
	bool initialized;
	int frame;
	//Para mplementarlo en linea
	Mat imageHoGAnt;
	Rect HOG_Ant;

	Mat imageHoG, imageDraw, imageGray;
	OcclStateMach sm;
	//Abriendo el SVM
	CvSVM SVM, SVMOcl;
};

#endif // FITHOG_H_

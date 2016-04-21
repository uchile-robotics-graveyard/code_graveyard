#include <fstream>
#include <stdio.h>
#include <iostream>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/ml/ml.hpp> // machine learning module

// unix
#include <sys/time.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>

#ifndef FUNCIONES_COMP_H_
#define FUNCIONES_COMP_H_

typedef std::vector<float> Histogram;

// parameters
const float crop_x_factor    = 4.0;
const float crop_up_factor   = 2.5;
const float crop_down_factor = 3.5;
const uchar NON_UNIFORM_LBP = 0b01010101;
const uint HIST_SIZE = 59;
const uint HIST_REGIONS = 4;



/**
 * reads an image as grayscale
 */
void readImage(std::string img_name, cv::Mat& image);

/**
 * returns true if <x> is lbp-uniform
 *
 */
bool isLBPUniform(uchar x);

/**
 * generates a new image where every pixel contains a LBP-u2-8,1 code
 */
void computeLBPuniform(cv::Mat &input, cv::Mat &im_lbp);

void computeLBPregionHist(cv::Mat &im_lbp, Histogram &hist);


/**
 * generates a normalized intensity histogram for image <im_lbp>
 */
void computeLBPhist(cv::Mat &im_lbp, Histogram &hist);

/**
 * reduces a naive 256 length histogram to one with size 59
 */
void reduceHist(Histogram &input, Histogram &output);

//para comparar histogramas
double compHistBhattacharyya(Histogram& h1, Histogram& h2);

double d(const Histogram& h1, const Histogram& h2);

Histogram hist(cv::Mat src, int hist_size);

Histogram histHSV(cv::Mat src);

#endif //FUNCIONES_COMP_H_

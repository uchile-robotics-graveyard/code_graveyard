#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <fstream>

typedef struct features
{
	float clase;
	float distLBP;
	float distColor;
}features;

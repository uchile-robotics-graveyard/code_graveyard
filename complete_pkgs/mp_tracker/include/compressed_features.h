#ifndef _COMPRESS_FEATURES_H
#define _COMPRESS_FEATURES_H
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/uniform_int.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <time.h>

using namespace std;
using namespace cv;
using namespace Eigen;

struct CompressedFeatures {
  bool initialised;
  size_t num_feat;
  size_t min_num_rect;
  size_t max_num_rect;

  struct feature {
    vector<Rect> rects;
    vector<double> weights;
    int channel;
  };

  vector<feature> features;

  CompressedFeatures(size_t num_feat_):
    initialised(false),
    num_feat(num_feat_),
    min_num_rect(2),
    max_num_rect(3) {}

  void generate_features(size_t width, size_t height) {
    boost::mt19937 rng;
    rng.seed(time(NULL));
    boost::uniform_01<> unf;
    boost::uniform_int<> unfc(0,2);
    boost::uniform_int<> unfe(0,1);
    boost::uniform_int<> unfr(min_num_rect, max_num_rect);
    boost::variate_generator<boost::mt19937, boost::uniform_01<> > var_unf(rng, unf);
    boost::variate_generator<boost::mt19937, boost::uniform_int<> > var_unfc(rng, unfc);
    boost::variate_generator<boost::mt19937, boost::uniform_int<> > var_unfe(rng, unfe);
    boost::variate_generator<boost::mt19937, boost::uniform_int<> > var_unfr(rng, unfr);

    features.resize(num_feat);
    for(size_t i = 0; i < num_feat; i++) {
      int num_rects = var_unfr();
      features[i].channel = var_unfc();
      features[i].rects.resize(num_rects);
      for(int j = 0; j < num_rects; j++) {
        features[i].rects[j].x = std::floor(var_unf() * (width - 3));
        features[i].rects[j].y = std::floor(var_unf() * (height - 3));
        features[i].rects[j].width = std::ceil(var_unf() * (width - features[i].rects[j].x - 2));
        features[i].rects[j].height = std::ceil(var_unf() * (height - features[i].rects[j].y - 2));
        features[i].weights.push_back(std::pow(-1.0, var_unfe()) / sqrt((double)num_rects));
      }
    }
    initialised = true;
  }


  VectorXd evaluate(const vector<Mat> &imgs) {
    assert(initialised);
    vector<Mat> intimgs(3);
    integral(imgs[0], intimgs[0], CV_64F);
    integral(imgs[1], intimgs[1], CV_64F);
    integral(imgs[2], intimgs[2], CV_64F);

    const double c = 1.0 / 255.0;
    VectorXd result = VectorXd::Zero(num_feat);
    for(size_t i = 0; i < num_feat; ++i) {
      const int channel = features[i].channel;
      for(size_t j = 0; j < features[i].rects.size(); ++j) {
        const double x0 = features[i].rects[j].x;
        const double y0 = features[i].rects[j].y;
        const double x1 = x0 + features[i].rects[j].width;
        const double y1 = y0 + features[i].rects[j].height;
        result(i) = c * features[i].weights[j] * (intimgs[channel].at<double>(y0, x0)
                                         +intimgs[channel].at<double>(y1, x1)
                                         -intimgs[channel].at<double>(y0,x1)
                                         -intimgs[channel].at<double>(y1,x0));
      }
    }
    return result;
  }

};

#endif

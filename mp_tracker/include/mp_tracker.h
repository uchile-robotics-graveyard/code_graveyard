#ifndef _MP_TRACKER
#define _MP_TRACKER
#include <map>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

/*
#include <unsupported/Eigen/FFT>
#define EIGEN_FFTW_DEFAULT
*/
#include "util.h"
#include "person.h"
#include "map.h"
#include "unscented_sampler.hpp"

#include "munkres.h"

#define DEBUG
#define SAMPLE_FROM_MIXTURE
#define USE_HOG
//#define USE_LATENT
#define IMAGE_TRACKING
#define UT_BIRTH_MODEL

namespace icl {

struct measurement_2d_fullbody {
  double u, v, h;
  double weight;
/*
  cv::Mat histogram_head;
  cv::Mat histogram_body;
  cv::Mat histogram_legs;
*/
};

// Birth process based on "PHD Filter with Diffuse Spatial prior on the Birth Process with applications to GMPHD filter"
// But the actual mean and cov used for each birth gaussianc omponent is different
class MPTracker {
public:
  typedef boost::shared_ptr<MPTracker> ptr;
  // State in the current state
  std::vector<person_state::ptr> states_t;
  // Variable for the next states calculation
  std::vector<person_state::ptr> states_tp1;
  Eigen::Matrix<double,5,5> F;
  Eigen::Matrix<double,5,5> Q;
  map static_map;
  boost::shared_ptr<cv::HOGDescriptor> people_detector;

  double cluster_proximity;
  double birth_prior_weight;

  // Laser field likelihood parameters (not in use)
  double r_max, p_const, sigma, sigma_2, p_r;
  double angle_epsilon;

  unsigned int num_targets;
  double truncate_threshold;
  double merge_threshold;
  double max_components;

  double simple_laser_likelihood_gamma;

  double visual_max_dist_detect, visual_min_dist_detect;

  double p_survival;
  double p_fullbody_detect;
  double clutter_fullbody_rfs;

  double detection_pixel_variance, detection_height_variance;
  bool use_map_likelihood;
  icl::unscented_sampler<> unsctd_sampler;
  double pixel_error;
  double height_error;
  double detection_border;
  std::map<size_t, std::map<double, Eigen::VectorXd> > paths;
  std::vector<size_t> active_paths;
  double max_path_distance;
  double heading_path_cost;
  double start_running;
  double last_update_time;
private:
  size_t last_path;
  // Predict an observation of a 2d fullbody feature. signature includes person state, the error, and the output measurement
  void observation_2d_fullbody(const person_state::ptr &, const Eigen::Vector3d &, measurement_2d_fullbody &, const Eigen::Matrix<double,3,4> &);
  // Constant needed for the likelihood model
  double laser_reading_likelihood(const Eigen::VectorXd &, const Eigen::Matrix4d &, const std::vector<float> &, const double &, const double &);
  double laser_reading_likelihood_simple(const Eigen::VectorXd &, const pcl::KdTree<pcl::PointXYZ>::Ptr &);

public:
  MPTracker();
  // Predict the state of the world given dt
  void predict(const double &, const Eigen::Vector3d &, const double &);
  // Correct the state given a laser reading. T is the transformation matrix
  void correct_laser_mc(const std::vector<float> &, double , double , const Eigen::Matrix4d &);
  // Correct the state given features from a 2d fullbody detector. K matrix is P*T where P is the projection matrix and T the transformation matrix
  void correct_feature_2d_fullbody_ut(const std::vector<measurement_2d_fullbody> &, const Eigen::Matrix<double,3,4> &, const Eigen::Matrix<double,4,4> &, const std::vector<person_state::ptr> &);
  // Prune the gaussian mixture
  void prune();
  // Get the state
  template<typename Container>
  void get_state(Container &output, bool manage_tracks = false);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#include "mp_tracker.hpp"
}
#endif

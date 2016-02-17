#ifndef _D_TRACKER
#define _D_TRACKER
#include <cmath>
#include <algorithm>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/function.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_01.hpp>
#include "unscented_sampler.hpp"
// Neede for PointWithVelocity
#include "mp_tracker.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

// GNU Sientific Library
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multiroots.h>
#include <gsl/gsl_sf.h>

// Munkres algorithm for id estimation
#include "munkres.h"

// For debugging
#include <boost/python/import.hpp>
#include <boost/python/exec.hpp>
#include <boost/python/list.hpp>
#include <time.h> 

#include <ros/console.h>
#include <bender_msgs/StateInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <bender_srvs/Transformer.h>
#include <bender_srvs/NavGoal.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#define LOG_SQRT_2PI_INV -0.39908993417

namespace python = boost::python;

namespace icl {

template<int D>
struct _particle {
  double weight;
  int indx;
  Eigen::Matrix<double,D,1> state;

  bool operator< (const _particle &p2) const {
    return (this->weight < p2.weight);
  }

  typedef typename std::vector<_particle<D>, Eigen::aligned_allocator<_particle<D> > > particles;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Function used to optimise beta parameters
int optimise_beta_params(const gsl_vector *x, void *params, gsl_vector *f);
int optimise_beta_params_df(const gsl_vector *x, void *params_, gsl_matrix *J);
int optimise_beta_params_fdf(const gsl_vector *x, void *params_, gsl_vector *f, gsl_matrix *J);

double kl_diff_beta(const double &a_1, const double &b_1, const double &a_2, const double &b_2);
double bhatta_dist_beta(const double &a_1, const double &b_1, const double &a_2, const double &b_2);
void clear_bhatta_cache();
double bhatta_dist_cat(const double &a_1, const double &b_1, const double &a_2, const double &b_2);

/*
template<int D>
struct constant_probability {
  double p;
  constant_probability(): p(1.0) {}
  constant_probability(double p_): p(p_) {}
  double operator()(const Eigen::Matrix<double,D,1> &x) const {
    return p;
  }
};
*/


// Birth process based on "PHD Filter with Diffuse Spatial prior on the Birth Process with applications to GMPHD filter"
// But the actual mean and cov used for each birth gaussian component is different
template<int dim_state=4, int dim_measurement=2>
struct DTracker {
  typedef boost::shared_ptr<DTracker<dim_state,dim_measurement> > ptr;
  typedef typename Eigen::Matrix<double,dim_measurement,1> measurement;
  typedef typename Eigen::Matrix<double,dim_state,1> state;
  typedef typename Eigen::Matrix<double,dim_measurement,dim_measurement> measurement_mat;
  typedef typename Eigen::Matrix<double,dim_state,dim_state> state_mat;
  typedef typename Eigen::Matrix<double,dim_state,dim_measurement> state_measurement_mat;
  typedef typename Eigen::Matrix<double,dim_measurement,dim_state> measurement_state_mat;
  typedef typename std::vector<double> weights;
  typedef typename std::vector<state, Eigen::aligned_allocator<state> > states;
  typedef typename std::vector<measurement, Eigen::aligned_allocator<measurement> > measurements;
  typedef typename std::vector<measurement_mat, Eigen::aligned_allocator<measurement_mat> > measurement_mats;
  typedef typename std::vector<state_mat, Eigen::aligned_allocator<state_mat> > state_mats;
  typedef typename std::vector<state_measurement_mat, Eigen::aligned_allocator<state_measurement_mat> > state_measurement_mats;
  typedef typename boost::function<state(const state &, const Eigen::VectorXd &)> update_function;
  typedef typename boost::function<measurement(const state &, const Eigen::VectorXd &)> measurement_function;
  typedef typename boost::function<double(const state &)> state_probability_function;
  typedef typename std::map<size_t, state, std::less<size_t>, Eigen::aligned_allocator<std::pair<const size_t, state> > > ids_storage;
//  typedef typename std::map<size_t, size_t > ids_undetect_count;

  const bool use_beta_mixture;
  const bool use_optimal_beta_merge;
  const bool cap_max_weights;

  //ROS
  ros::NodeHandle priv;
  ros::Publisher _mov_pub;
  ros::Publisher obj_pub;
  ros::ServiceClient _transf_serv;
  ros::ServiceClient _move_approach_serv;
  ros::Subscriber pose_bender;

  bool publish_state;
  bool last_seen;
  time_t last_time;

  geometry_msgs::PoseStamped PoseBender;

  // This structure hold the data of each component
  struct _internal_component {
    // Internal variables
    double weight;
    state mean;
    state_mat cov;
    // Only used when dealing with mixtures ob BetaGaussian
    double a, b;

    bool operator<(const _internal_component &c) {
      return this->weight < c.weight;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  typedef typename std::vector<_internal_component, Eigen::aligned_allocator<_internal_component> > components_type;

  struct _internal_state {
    // Internal variables
    int id;
    double x;
    double y;
    double vx;
    double vy;
  };

  // Vector with the components for time t and t+1
  components_type components_t, components_tp1;

  // Motion model
  state_mat F;
  state_mat Q;
  state_mat T;

  //Training person
  bool findperson;
  _internal_state trPerson;
  // Observation model
  measurement_state_mat H;
  measurement_mat R;
  // Sigma for the unscented sampling
  double sigma_f, sigma_h;

  // Likelihood of the observations
  double observation_likelihood;
  double log_observation_likelihood;

  // Parameters of the correction step
  double p_survival;
  double p_detect;
  double clutter_rfs;

  // Parameters of the pruning step
  unsigned int num_targets;
  double truncate_threshold;
  double merge_gauss_threshold;
  double merge_beta_threshold;
  double max_components;

  double certainty_person_threshold;
  double certainty_person_extraction;
  double is_person_probability;
  
  // Parameters of the birth prior
  weights birth_prior_weights;
  states birth_prior_means;
  state_mats birth_prior_covs;

  icl::unscented_sampler<> unsctd_sampler;
  ids_storage targets_id;

//  ids_undetect_count id_undetection;
  size_t last_id;

  DTracker(const DTracker &dt);
  DTracker(bool use_beta, bool use_optimal_merging, bool cap_max_weights_);
  ~DTracker();
  // Predict the state of the world given dt
  void predict();
  void predict_ut(size_t dim_noise, const update_function &);
  // Correct the state given features from a 2d fullbody detector. K matrix is P*T where P is the projection matrix and T the transformation matrix
  void set_birth_prior(const weights &weights, const states &means, const state_mats &covs) {
    assert(means.size() == covs.size());
    assert(means.size() == weights.size());
    const size_t new_size = means.size();
    birth_prior_means.resize(new_size);
    birth_prior_covs.resize(new_size);
    birth_prior_weights.resize(new_size);
    std::copy(means.begin(), means.end(), birth_prior_means.begin());
    std::copy(covs.begin(), covs.end(), birth_prior_covs.begin());
    std::copy(weights.begin(), weights.end(), birth_prior_weights.begin());
  }

  inline void clear_birth_prior() {
    birth_prior_means.resize(0);
    birth_prior_covs.resize(0);
  }

  // Different correction methods
  void correct(const measurements &, const std::vector<int> &categories=std::vector<int>());
  void correct_ut(const measurements &, size_t dim_noise, const measurement_function &, const std::vector<int> &categories=std::vector<int>());
  void correct_mc(const state_probability_function &state_likelihood);

  bool train_person();
  bool StartFollow();
  bool PauseFollow();
  bool StopFollow();
  geometry_msgs::PoseStamped  publish_marker();

  void PoseCallback(geometry_msgs::PoseStamped in);

  // Prune the gaussian mixture
  void prune();
  // Get the state
  void get_state(pcl::PointCloud<icl::PointWithVelocity>::Ptr &output, bool one_per_gaussian, bool get_only_moving);

  // For debugging
  PyThreadState *gil_save;
  python::object python_module;
  python::object python_namespace;
  python::object python_plotter;
  bool python_loaded;
  void load_debug_plugin(const std::string &python_file);
  void debug_plot(bool t, double time, const Eigen::Matrix4d &T,  const pcl::PointCloud<pcl::PointXYZ>::Ptr &laser_pc, const measurements &meas);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Sample from a mixture of gaussians
template<int D, int M>
void _sample_from_mixture(const typename DTracker<D,M>::components_type &mixture,
                         size_t num_samples, typename _particle<D>::particles &out);
template<int D>
void _sample_from_samples(const typename _particle<D>::particles &in,
                         size_t num_samples, typename _particle<D>::particles &out);

// The actual implementation
#include "d_tracker.hpp"
// Usefull typedef
typedef DTracker<4,2> DTracker42;
typedef DTracker<7,4> DTracker74;

}

#endif

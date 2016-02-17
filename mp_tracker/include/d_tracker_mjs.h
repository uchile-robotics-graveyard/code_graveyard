#ifndef _D_TRACKER_MJS
#define _D_TRACKER_MJS
#include "util.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>
#include <boost/function.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_01.hpp>
#include "unscented_sampler.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

// GNU Sientific Library
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multiroots.h>
#include <gsl/gsl_sf.h>

// For debugging
#include <boost/python/import.hpp>
#include <boost/python/exec.hpp>
#include <boost/python/list.hpp>

namespace python = boost::python;

namespace icl {
template<int dim_state=4, int dim_measurement=2, int num_states=2>
struct DTrackerMJS {
  typedef boost::shared_ptr<DTrackerMJS<dim_state,dim_measurement,num_states> > ptr;
  typedef typename Eigen::Matrix<double,num_states,num_states> transition_matrix;;
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
  typedef typename boost::function<state(state, state)> update_function;
  typedef typename boost::function<measurement(state, measurement)> measurement_function;

  const bool cap_max_weights;

  // Transition matrix of the markov process
  transition_matrix transition;

  // Internal variables
  weights weights_t[num_states];
  states means_t[num_states];
  state_mats covs_t[num_states];
  // Variable for the next states calculation
  weights weights_tp1[num_states];
  states means_tp1[num_states];
  state_mats covs_tp1[num_states];
  // Motion model
  state_mat F[num_states];
  state_mat Q[num_states];
  // Observation model
  measurement_state_mat H[num_states];
  measurement_mat R[num_states];
  double sigma_f[num_states], sigma_h[num_states];

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
  
  // Parameters of the birth prior
  double birth_prior_weight;
  states birth_prior_means;
  state_mats birth_prior_covs;

  icl::unscented_sampler<> unsctd_sampler;

  DTrackerMJS(bool use_beta, bool cap_max_weights_);
//  ~DTrackerMJS();
  // Predict the state of the world given dt
  void predict();
  void predict_ut(const update_function &);
  // Correct the state given features from a 2d fullbody detector. K matrix is P*T where P is the projection matrix and T the transformation matrix
  void set_birth_prior(const states &means, const state_mats &covs) {
    assert(means.size() == covs.size());
    birth_prior_means.resize(means.size());
    birth_prior_covs.resize(covs.size());
    std::copy(means.begin(), means.end(), birth_prior_means.begin());
    std::copy(covs.begin(), covs.end(), birth_prior_covs.begin());
  }

  inline void clear_birth_prior() {
    birth_prior_means.resize(0);
    birth_prior_covs.resize(0);
  }

  void correct(const measurements &);
  void correct_ut(const measurements &, const measurement_function &);
  // Prune the gaussian mixture
  void prune();
  // Get the state
  template<typename Container>
  void get_state(Container &output, int mode, bool one_per_gaussian);

/*
  // For debugging
  PyThreadState *gil_save;
  python::object python_module;
  python::object python_namespace;
  python::object python_plotter;
  bool python_loaded;
  void load_debug_plugin(const std::string &python_file);
  void debug_plot(bool t, const pcl::PointCloud<pcl::PointXYZ>::Ptr &laser_pc, const measurements &meas);
*/
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#include "d_tracker_mjs.hpp"


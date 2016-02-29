#ifndef _KB_TRACKER
#define _KB_TRACKER
#include <cmath>
#include <algorithm>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <boost/function.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_01.hpp>
#include "unscented_sampler.hpp"
// Needed for PointWithVelocity
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

#include <ros/console.h>

#define LOG_SQRT_2PI_INV -0.9189385332046727

namespace icl {

// Gaussian-Beta tracker for estimation of class and spatial position
template<int dim_state=4, int dim_measurement=2>
struct GBTracker {
  typedef boost::shared_ptr<GBTracker<dim_state,dim_measurement> > ptr;
  typedef typename Eigen::Matrix<double,dim_measurement,1> measurement;
  typedef typename Eigen::Matrix<double,dim_state,1> state;
  typedef typename Eigen::Matrix<double,dim_measurement,dim_measurement> measurement_mat;
  typedef typename Eigen::Matrix<double,dim_state,dim_state> state_mat;
  typedef typename Eigen::Matrix<double,dim_state,dim_measurement> state_measurement_mat;
  typedef typename Eigen::Matrix<double,dim_measurement,dim_state> measurement_state_mat;
  typedef typename std::vector<state, Eigen::aligned_allocator<state> > states;
  typedef typename std::vector<measurement, Eigen::aligned_allocator<measurement> > measurements;
  typedef typename std::vector<state_mat, Eigen::aligned_allocator<state_mat> > state_mats;
  typedef typename std::vector<measurement_mat, Eigen::aligned_allocator<measurement_mat> > measurement_mats;
  typedef typename std::vector<state_measurement_mat, Eigen::aligned_allocator<state_measurement_mat> > state_measurement_mats;
  typedef typename boost::function<state(const state &, const Eigen::VectorXd &)> update_function;
  typedef typename boost::function<measurement(const state &, const Eigen::VectorXd &)> measurement_function;
  typedef typename boost::function<double(const state &)> state_probability_function;
  typedef typename std::map<size_t, state, std::less<size_t>, Eigen::aligned_allocator<std::pair<const size_t, state> > > ids_storage;
  typedef typename Eigen::ArrayXd colour_projection_type;
  typedef typename std::vector<colour_projection_type> colour_projections_type;

  // This structure hold the data of each component
  struct tracker_component {
    // Internal variables
    size_t id;
    size_t obs_count;
    size_t miss_detections_count;
    state mean;
    state_mat cov;
    colour_projection_type colour_projection_mean;
    colour_projection_type colour_projection_std;
    // Only used when dealing with mixtures ob BetaGaussian
    double a, b;
    bool operator<(const tracker_component &c) {
      return this->weight < c.weight;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  typedef typename std::vector<tracker_component, Eigen::aligned_allocator<tracker_component> > components_type;
  typedef typename boost::function<tracker_component(const measurement &, int, colour_projection_type)> new_state_function;

  new_state_function state_creator;

  // Vector with the components for time t and t+1
  components_type components_t, components_tp1;
  size_t num_colour_components;

  // Motion model
  state_mat F;
  state_mat Q;
  double colour_std_change;
  double probability_var_increase;
  // Observation model
  measurement_state_mat H;
  measurement_mat R;
  double colour_observation_std;

  // Sigma for the unscented sampling
  double sigma_f, sigma_h;

  // Confident levels for the beta observation
  double is_confident_value;
  double is_not_confident_value;

  // Likelihood of the observations
  double observation_likelihood;
  double log_observation_likelihood;

  double gating_threshold;

  // Parameters of the correction step
  size_t num_frames_missdetected;
  size_t min_frames_creation;

  // Parameteres to extract the final state
  double is_person_probability;
#if 0
  double certainty_person_extraction;
#endif

  double probability_var_extraction;
  icl::unscented_sampler<> unsctd_sampler;
  size_t last_id;

  // Use beta mixture or not
  bool use_beta;
  // Use colour compoennts
  bool use_colour;

  // Constructor
  GBTracker();

  // Predict the state of the world given dt
  void update_beta(size_t i);
  void update_colour(size_t i);
  void predict();
  void predict_ut(size_t dim_noise, const update_function &);

  void correct(const measurements &, const std::vector<double> &, const colour_projections_type &);
//  void correct_ut(const measurements &, const std::vector<int> &, const new_state_function &, size_t dim_noise, const measurement_function &);
//  void correct_mc(const state_probability_function &state_likelihood);

  // Get the state
  void get_state(pcl::PointCloud<icl::PointWithVelocity>::Ptr &output);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// The actual implementation
#include "gb_tracker.hpp"

}

#endif


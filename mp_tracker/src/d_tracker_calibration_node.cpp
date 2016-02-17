#include "d_tracker.h"
#include "util.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <fstream>
#include <cstdlib>
#include <stdexcept>
#include <functional>
#include <util.h>
#include "pointcloud_process.h"

// To avoid too many namespcae with the sensors
using namespace sensor_msgs;

// Helpfull to avoid chainging all the apearances all the time
typedef icl::DTracker<4,2> DaTracker;

struct DTrackerAndCalibrationNode {
  ros::NodeHandle main_nh;
  ros::NodeHandle priv_nh;
  ros::Publisher track_pub;
  boost::shared_ptr<tf::TransformListener> tf_listener;
  ros::Subscriber laser_sub;
  std::string tracking_frame;

  // Structure holding the estimation
  struct particle {
    std::vector<DaTracker::ptr> trackers;
    double weights;
    Eigen::VectorXd state;

    bool operator<(const particle &p1, const particle &p2) const {
      return p1.weights < p2.weights;
    }
  };
  std::vector<particle> particles_t, particles_tp1;
  // Number of particles to use
  size_t M;
  double last_time;
  double comfort_distance_sq;

  void sensors_cb(const LaserScanConstPtr &laser) {
    double time = ros::Time::now().toSec();
    // Convert the laser to pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pc(new pcl::PointCloud<pcl::PointXYZ>);
    icl::pointcloud_from_laser(laser->ranges, laser->angle_min, laser->angle_increment, 30.0, Eigen::Matrix4d::Identity(), laser_pc);

    // Extract the clusters from the laser
    DaTracker::measurements meas;
    DaTracker::measurement_mats meas_covs;
    icl::cluster_laser_2(laser_pc, 0.12, 3, 100, comfort_distance_sq, meas, meas_covs);

    // Create the base birth process mixture
    DaTracker::states birth_means(meas.size());
    DaTracker::state_mats birth_covs(meas.size());
    for(size_t i = 0; i < meas.size(); i++) {
      birth_means[i] = DaTracker::state::Zero();
      birth_means[i].block(0,0,2,1) = meas[i];
      birth_covs[i] = 1.0 * DaTracker::state_mat::Identity();
      birth_covs[i].block(0,0,2,2) = meas_covs[i];
    }

    // Do the filtering
    double dt = time - last_time;
    // Predict evolution of the system
    Eigen::Matrix<double,4,2> w;
    w << dt*dt / 2.0, 0.0,
       0.0, dt*dt / 2.0,
       dt, 0.0,
       0.0, dt;
    // 1. Prediction for static/dynamic objects
    for(size_t i = 0; i < M; i++) {
      particles_t[i].tracker->Q = w * w.transpose();
      particles_t[i].tracker->F(0, 2) = dt;
      particles_t[i].tracker->F(1, 3) = dt;
      particles_t[i].tracker->predict();
    }

    // 2. Prediction for robot's parameters
    std::vector<size_t> index;
    sample_from_weights(particle_t, M, index);
    for(size_t i = 0; i < M; i++) {
      particles_tp1[i].tracker.reset(DaTracker(*particles_t[index[i]].tracker));
      particles_tp1[i].weight = 1.0 / M;
      particles_tp1[i].state = sample_update(particles_states_t[index[i]]);
    }

    double total_obs_like = 0.0;
    // 3. Correction for static/dynamic objects
    for(size_t i = 0; i < M; i++) {
      // CREATE BIRTH PROCESS FOR THIS TRACKER
      // Set the birth process prior
      particles_tp1[i].tracker->set_birth_prior(birth_means, birth_covs);
      // Correct with the measurements
      particles_tp1[i].tracker->correct(meas);
      // Prune only if the sampling is not from the GMM directly
      particles_tp1[i].tracker->prune();
      // Accumulate the log likelihood
      total_obs_like += particles_tp1[i].tracker->observation_likelihood;
    }

    // 4. Correction for robot's parameters
    for(size_t i = 0; i < M; i++) {
      particles_tp1[i].weight *= particles_tp1[i].tracker->observation_likelihood / total_obs_like;
    }

    // 5. Copy to the _t arrays
    for(size_t i = 0; i < M; i++) {
      particles_t[i].weight = particles_tp1[i].weight;
      particles_t[i].state = particles_tp1[i].state;
      particles_t[i].tracker = particles_tp1[i].tracker;
    }
    last_time = time;
    pcl::PointCloud<icl::PointWithVelocity>::Ptr pc(new pcl::PointCloud<icl::PointWithVelocity>);
    // Publish
    d_tracker->get_state(pc, true, true);
    pc->width = 1;
    pc->height = pc->size();
    pc->is_dense = true;
    pc->header.frame_id = tracking_frame;
    pc->header.stamp.fromSec(last_time);
    track_pub.publish(pc);
  }

  DTrackerNode():
    main_nh(),
    priv_nh("~") {

    // Basic frames
    priv_nh.param("tracking_frame", tracking_frame, std::string("odom"));
    priv_nh.param("number_of_particles", M, 100);
    particles_t.resize(M);
    particles_tp1.resize(M);

    // Use beta-gaussian or gaussian only
    bool use_beta_mixture, cap_max_weights;
    priv_nh.param("use_beta_mixture", use_beta_mixture, false);
    priv_nh.param("cap_max_weights", cap_max_weights, false);
    d_tracker.reset(new DaTracker(use_beta_mixture, cap_max_weights));

    // Basic process model
    d_tracker->F = DaTracker::state_mat::Identity();
    // Basic process observation
    d_tracker->H = (DaTracker::measurement_state_mat() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished();
    d_tracker->R = 0.01*DaTracker::measurement_mat::Identity();

    // Parameters of the PHD filter
    priv_nh.param("birth_prior_weight", d_tracker->birth_prior_weight, 1E-5);
    priv_nh.param("probability_survival", d_tracker->p_survival, 0.95);
    priv_nh.param("probability_detect", d_tracker->p_detect, 0.95);
    priv_nh.param("clutter", d_tracker->clutter_rfs, 1E-5);

    // Set the parameters
    priv_nh.param("truncate_threshold", d_tracker->truncate_threshold, 1E-6);
    priv_nh.param("merge_gauss_threshold", d_tracker->merge_gauss_threshold, 5E-1);
    priv_nh.param("merge_beta_threshold", d_tracker->merge_beta_threshold, 0.1);
    priv_nh.param("max_components", d_tracker->max_components, 100.0);

    // Comfort distance
    double comfort_distance;
    priv_nh.param("comfort_distance", comfort_distance, 0.6);
    comfort_distance_sq = comfort_distance * comfort_distance;

    tf_listener.reset(new tf::TransformListener());

    // Initialise the last_time variable with the current time
    last_time = ros::Time::now().toSec();

    std::string laser_topic;
    priv_nh.param("laser_topic", laser_topic, std::string("scan"));

    laser_sub = main_nh.subscribe<LaserScan>(laser_topic, 1, &DTrackerNode::sensors_cb, this);

    // Create the publisher
    track_pub = main_nh.advertise<pcl::PointCloud<icl::PointWithVelocity> >("tracked", 10);

    // See if we load the plotting plugin
    std::string debug_script;
    priv_nh.param("debug_script", debug_script, std::string(""));
    if(!debug_script.empty()) {
      std::cout << "Loading file: " << debug_script << std::endl;
      d_tracker->load_debug_plugin(debug_script);
      if(d_tracker->python_loaded)
        ROS_INFO("Debuging python module loaded");
    }
  }

private:
  void sample_from_weights(const std::vector<particle> &particles, size_t num_samples, std::vector<size_t> &sample_index) {
    boost::mt19937 rng;
    boost::uniform_01<> unf;
    boost::variate_generator<boost::mt19937, boost::uniform_01<> > var_unf(rng, unf);

    std::sort(particles.begin(), particles.end());
    std::revert(particles.begin(), particles.end());
    double total_weight = 0.0;
    for(size_t i = 0; i < particles.size(); i++) {
      total_weight += particles[i].weight;
    }

    for(size_t i = 0; i < num_samples; i++) {
      double sample_weight = var_unf() * total_weight;
      size_t j = 0;
      for(; j < in.size(); j++) {
        sample_weight -= weights(j);
        if(sample_weight <= 0)
          break;
      }
      if(j == in.size()) j--;
      sample_index.push_back(j);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "density_tracker");
  // The main node
  DTrackerNode node;
  // Spin
  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;
}

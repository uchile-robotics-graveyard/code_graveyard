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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "legsdetector.h"
#include "pointcloud_process.h"

// To avoid too many namespcae with the sensors
using namespace sensor_msgs;

// Helpfull to avoid chainging all the apearances all the time
typedef icl::DTracker<4,2> DaTracker;

DaTracker::measurement observe_laser(const DaTracker::state &x, const Eigen::VectorXd &error) {
  double px = x(0);
  double py = x(1);
  double angle = std::atan2(py, px);
  double range = std::sqrt(px * px + py * py + error(1) * error(1));
  DaTracker::measurement ret;
  ret(0) = angle + error(0) / 10.0;
  ret(1) = range;
  return ret;
}

struct DTrackerNode {
  ros::NodeHandle main_nh;
  ros::NodeHandle priv_nh;
  DaTracker::ptr d_tracker;
  ros::Publisher track_pub, vis_pub;
  boost::shared_ptr<tf::TransformListener> tf_listener;
  ros::Subscriber laser_sub;
  boost::mutex mtx;
  ros::Timer spin_timer;
  double last_time;
  std::string tracking_frame;
  double sigma_a_dynamic, sigma_v_static, sigma_obs;
  double birth_prior_weight;
  double comfort_distance_sq;
  // Leg detector object
  bool use_laser_leg_detector;
  boost::shared_ptr<LegsDetector> leg_detector;
  size_t previous_published;

  void sensors_cb(const LaserScanConstPtr &laser) {
    double time = ros::Time::now().toSec();
    // Get needed trasforms
    Eigen::Matrix4d T_lt;
    try {
      icl::lookupTransform(*tf_listener, tracking_frame, laser->header.frame_id, T_lt);
    } catch(tf::TransformException e) {
      ROS_WARN_STREAM("Miss a transformation. " << e.what());
      return;
    }

    // This variables are needed regardless of the detection method use
    vector<laser_t> laser_buf;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pc;

    // Extract the clusters from the laser
    DaTracker::measurements meas;
    DaTracker::measurement_mats meas_covs;
    if(use_laser_leg_detector) {
      if(!leg_detector) {
        // Initialise the leg detector 
        leg_detector.reset(new LegsDetector(laser->ranges.size(), 0));
      }
      laser_buf.resize(laser->ranges.size());
      for(size_t i = 0; i < laser->ranges.size(); i++) {
        laser_buf[i].range = laser->ranges[i];
        laser_buf[i].angle = laser->angle_min + i * laser->angle_increment;
        laser_buf[i].x = laser_buf[i].range * cos(laser_buf[i].angle);
        laser_buf[i].y = laser_buf[i].range * sin(laser_buf[i].angle);
        if(laser->intensities.size() == laser->ranges.size())
          laser_buf[i].intensity = laser->intensities[i];
        else
          laser_buf[i].intensity = 1.0;
      }
      leg_detector->update(laser_buf);
      // Create the array of observations
      for(int i = 0; i < leg_detector->getHowMany(); i++) {
        double range = leg_detector->getDistance(i);
        double angle = leg_detector->getBearing(i);
        Eigen::Vector4d x = T_lt * (Eigen::Vector4d() <<
            range * cos(angle),
            range * sin(angle),
            0.0,
            1.0
        ).finished();
        meas.push_back(x.block(0,0,2,1));
        meas_covs.push_back(sigma_obs * Eigen::Matrix2d::Identity());
      }
    } else {
      // Convert the laser to pointcloud
      laser_pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
      icl::pointcloud_from_laser(laser->ranges, laser->angle_min, laser->angle_increment, 30.0, T_lt, laser_pc);
      // Get the clusters as measurements
      icl::cluster_laser_2(laser_pc, 0.15, 5, 100, comfort_distance_sq, meas, meas_covs);
    }


    // Create the birth process mixture
    DaTracker::weights birth_weights(meas.size());
    DaTracker::states birth_means(meas.size());
    DaTracker::state_mats birth_covs(meas.size());
    for(size_t i = 0; i < meas.size(); i++) {
      birth_weights[i] = birth_prior_weight;
      birth_means[i] = DaTracker::state::Zero();
      birth_means[i].block(0,0,2,1) = meas[i];
      birth_covs[i] = 1.0 * DaTracker::state_mat::Identity();
      birth_covs[i].block(0,0,2,2) = meas_covs[i];
    }

    // Do the filtering
    double dt = time - last_time;
    last_time = time;
    // Predict evolution of the system
    Eigen::Matrix<double,4,2> w;
    w << dt*dt / 2.0, 0.0,
       0.0, dt*dt / 2.0,
       dt, 0.0,
       0.0, dt;
    d_tracker->Q = sigma_a_dynamic * (w * w.transpose());
    d_tracker->T(0,0) = sigma_v_static * dt;
    d_tracker->T(1,1) = sigma_v_static * dt;
    d_tracker->F(0, 2) = dt;
    d_tracker->F(1, 3) = dt;
    d_tracker->predict();
    // For debuging purposes
    d_tracker->debug_plot(true, time, T_lt, laser_pc, meas);
    // Set the birth process prior
    d_tracker->set_birth_prior(birth_weights, birth_means, birth_covs);
    // Correct with the measurements
    d_tracker->correct(meas);
    // Prune only if the sampling is not from the GMM directly
    d_tracker->prune();
    // Get the estimated state
    pcl::PointCloud<icl::PointWithVelocity>::Ptr pc(new pcl::PointCloud<icl::PointWithVelocity>);
    d_tracker->get_state(pc, true, !d_tracker->use_beta_mixture);
    // Finish the pointcloud definitions
    pc->width = 1;
    pc->height = pc->size();
    pc->is_dense = true;
    pc->header.frame_id = tracking_frame;
    pc->header.stamp = laser->header.stamp.toSec();
    // Publish the results
    track_pub.publish(pc);
    // Create the visualisation array
		visualization_msgs::MarkerArray mrks;
    fill_visualisation_msg(pc, previous_published, mrks);
    previous_published = pc->size();
    vis_pub.publish(mrks);
  }

  DTrackerNode():
    main_nh(),
    priv_nh("~") {

    // Basic frames
    priv_nh.param("tracking_frame", tracking_frame, std::string("odom"));

    // Use beta-gaussian or gaussian only
    bool use_beta_mixture, optimal_merge, cap_max_weights;
    priv_nh.param("use_beta_mixture", use_beta_mixture, false);
    priv_nh.param("use_optimal_beta_merge", optimal_merge, false);
    priv_nh.param("cap_max_weights", cap_max_weights, false);
    d_tracker.reset(new DaTracker(use_beta_mixture, optimal_merge, cap_max_weights));

    // Use range-bearing or cartesian observations
    d_tracker->sigma_h = 0.01;
    priv_nh.param("use_laser_leg_detector", use_laser_leg_detector, false);

    // Basic process model
    priv_nh.param("sigma_v_static", sigma_v_static, 0.0025);
    priv_nh.param("sigma_a_dynamic", sigma_a_dynamic, 4.0);
    d_tracker->F = DaTracker::state_mat::Identity();
    d_tracker->T = sigma_v_static * DaTracker::state_mat::Identity();
    // Basic process observation
    d_tracker->H = (DaTracker::measurement_state_mat() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished();
    priv_nh.param("sigma_obs", sigma_obs, 0.01);
    d_tracker->R = sigma_obs * DaTracker::measurement_mat::Identity();

    // Parameters of the PHD filter
    priv_nh.param("birth_prior_weight", birth_prior_weight, 1E-5);
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
    vis_pub = main_nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",10);
    previous_published = 0;

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

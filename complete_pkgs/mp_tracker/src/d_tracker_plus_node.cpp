#include "d_tracker.h"
#include "util.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <util.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "legsdetector.h"
#include "pointcloud_process.h"
#include <pcl_conversions/pcl_conversions.h>
#include <bender_srvs/IsOn.h>
#include <bender_srvs/Dummy.h>



// To avoid too many namespcae with the sensors
using namespace sensor_msgs;

// Helpfull to avoid chainging all the apearances all the time
typedef icl::DTracker<4,2> DaTracker;

struct DTrackerNode {
  ros::NodeHandle main_nh;
  ros::NodeHandle priv_nh;
  DaTracker::ptr d_tracker;

  ros::Publisher track_pub, vis_pub;
  ros::ServiceServer _train_server, _start_follow, _pause_follow, _stop_follow;
  boost::shared_ptr<tf::TransformListener> tf_listener;

  ros::Subscriber laser_sub;
  ros::Timer spin_timer;
  double last_time;
  std::string tracking_frame;
  double sigma_a_dynamic, sigma_v_static, sigma_obs;
  double birth_prior_weight;
  double comfort_distance_sq;
  boost::shared_ptr<LegsDetector> leg_detector;
  size_t previous_published;

  void laser_cb(const LaserScanConstPtr &laser) {
    // See if the filter is available to be updated
    double time = ros::Time::now().toSec();
    // Get needed trasforms
    Eigen::Matrix4d T_lt;
    try {
      icl::lookupTransform(*tf_listener, tracking_frame, laser->header.frame_id, T_lt);
    } catch(tf::TransformException e) {
      std::cout << "Miss a transformation. " << e.what() << std::endl;
      return;
    }

    // This variables are needed regardless of the detection method use
    vector<laser_t> laser_buf;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pc;

    // Extract the clusters from the laser
    DaTracker::measurements meas;
    DaTracker::measurement_mats meas_covs;
    DaTracker::weights birth_weights;
    DaTracker::states birth_means;
    DaTracker::state_mats birth_covs;
    #pragma omp parallel sections num_threads(2)
    {
      {
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
        const size_t new_size = leg_detector->getHowMany();
        birth_weights.resize(new_size);
        birth_means.resize(new_size);
        birth_covs.resize(new_size);
        for(int i = 0; i < new_size; i++) {
          double range = leg_detector->getDistance(i);
          double angle = leg_detector->getBearing(i);
          Eigen::Vector4d x = T_lt * (Eigen::Vector4d() <<
              range * cos(angle),
              range * sin(angle),
              0.0,
              1.0
          ).finished();
          // Create the birth process mixture
          birth_weights[i] = birth_prior_weight;
          birth_means[i] = DaTracker::state::Zero();
          birth_means[i].block(0,0,2,1) = x.block(0,0,2,1);
          birth_covs[i] = 0.09 * DaTracker::state_mat::Identity();
        }
      }
      #pragma omp section
      if(0){
        // Convert the laser to pointcloud
        laser_pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
        icl::pointcloud_from_laser(laser->ranges, laser->angle_min, laser->angle_increment, 30.0, T_lt, laser_pc);
        // Get the clusters as measurements
        icl::cluster_laser_2(laser_pc, 0.15, 5, 100, comfort_distance_sq, meas, meas_covs);
      }
    }
    meas.resize(birth_means.size());
    for(size_t i = 0; i < birth_means.size(); i++)
      meas[i] = birth_means[i].block(0,0,2,1);

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
    std::vector<int> cat;
    d_tracker->correct(meas, cat);
    // Prune
    d_tracker->prune();
    // Get the estimated state
    pcl::PointCloud<icl::PointWithVelocity>::Ptr pc(new pcl::PointCloud<icl::PointWithVelocity>);
    // d_tracker->get_state(pc, true, !d_tracker->use_beta_mixture);
    d_tracker->get_state(pc, true, false);
    // Finish the pointcloud definitions
    pc->width = 1;
    pc->height = pc->size();
    pc->is_dense = true;
    pc->header.frame_id = tracking_frame;
    pc->header.stamp = ros::Time::now().toNSec() / 1e3;
    // Publish the results
    track_pub.publish(pc);
    // Create the visualisation array
    visualization_msgs::MarkerArray mrks;
    // Create the visualiaiton msg
    fill_visualisation_msg(pc, previous_published, mrks);

    // Add birth
    static size_t previous_births_count = 0;
    for(size_t i = 0; i < std::max(birth_means.size(), previous_births_count); i++) {
	visualization_msgs::Marker marker;
	marker.header = pcl_conversions::fromPCL(pc->header);
	marker.ns = "d_tracker_birth";
	marker.id = i;
	marker.type = visualization_msgs::Marker::CYLINDER;
	if(i < birth_means.size()) {
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.pose.position.x = birth_means[i](0);
	  marker.pose.position.y = birth_means[i](1);
	  marker.pose.position.z = 0.0;
	  marker.scale.x = 0.6;
	  marker.scale.y = 0.6;
	  marker.scale.z = 0.1;
	  marker.color.r = 1.0;
	  marker.color.g = 1.0;
	  marker.color.b = 0.0;
	  marker.color.a = 0.8;
	  marker.lifetime = ros::Duration(1./15);
	} else {
	  marker.action = visualization_msgs::Marker::DELETE;
	}
	mrks.markers.push_back(marker);
    }

    // Add the measurements
    static size_t previous_measurements_count = 0;
    for(size_t i = 0; i < std::max(meas.size(), previous_measurements_count); i++) {
	visualization_msgs::Marker marker;
	marker.header = pcl_conversions::fromPCL(pc->header);
	marker.ns = "d_tracker_measurements";
	marker.id = i;
	marker.type = visualization_msgs::Marker::CUBE;
	if(i < meas.size()) {
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.pose.position.x = meas[i](0);
	  marker.pose.position.y = meas[i](1);
	  marker.pose.position.z = 0.0;
	  marker.scale.x = 0.3;
	  marker.scale.y = 0.3;
	  marker.scale.z = 0.3;
	  marker.color.r = 0.0;
	  marker.color.g = 1.0;
	  marker.color.b = 1.0;
	  marker.color.a = 0.8;
	  marker.lifetime = ros::Duration(1./15);
	} else {
	  marker.action = visualization_msgs::Marker::DELETE;
	}
	mrks.markers.push_back(marker);
    }
    previous_published = pc->size();
    previous_births_count = birth_means.size();
    previous_measurements_count = meas.size();
    // Publish the msg
    vis_pub.publish(mrks);
  }

  bool trainperson(bender_srvs::IsOn::Request &req, bender_srvs::IsOn::Response &res){
    res.is_on = d_tracker->train_person();
    return  true;
  }


  bool startfollowperson(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res){
    d_tracker->StartFollow();
    return  true;
  }

  bool pausefollowperson(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res){
    d_tracker->PauseFollow();
    return  true;
  }

  bool stopfollowperson(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res){
    d_tracker->StopFollow();
    return  true;
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
    d_tracker->clear_birth_prior();

    // Use range-bearing or cartesian observations
    d_tracker->sigma_h = 0.01;

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
    double p_detect;
    priv_nh.param("probability_detect", p_detect, 0.95);
    d_tracker->p_detect = p_detect;
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
    laser_sub = main_nh.subscribe<LaserScan>(laser_topic, 1, &DTrackerNode::laser_cb, this);

    // Create the service
    _train_server = main_nh.advertiseService("train_person", &DTrackerNode::trainperson, this);
    _start_follow = main_nh.advertiseService("Start_follow", &DTrackerNode::startfollowperson, this);
    _pause_follow = main_nh.advertiseService("Pause_follow", &DTrackerNode::pausefollowperson, this);
    _stop_follow = main_nh.advertiseService("Stop_follow", &DTrackerNode::stopfollowperson, this);



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
  //ros::MultiThreadedSpinner spinner;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::spin();
  return 0;
}


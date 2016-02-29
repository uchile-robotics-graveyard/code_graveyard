#include "d_tracker.h"
#include "util.h"
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/transforms.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include<Eigen/StdVector>
#include <util.h>
#include "pointcloud_process.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <unscented_sampler.hpp>
#include <icl_robot/plane_filter.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// To avoid too many namespcae with the sensors
using namespace sensor_msgs;
namespace enc = sensor_msgs::image_encodings;

// Helpfull to avoid chainging all the apearances all the time
typedef icl::DTracker<4,2> DaTracker;

struct DTrackerNode {
  // Usefull typedefes
  typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, pcl::PointCloud<pcl::PointXYZ> > Policy;
  typedef message_filters::Synchronizer<Policy> Synchronizer;
  typedef icl::unscented_sampler<11> unscented_sampler;

  ros::NodeHandle main_nh, priv_nh;
  DaTracker::ptr d_tracker;
  ros::Publisher track_pub, vis_pub;
  boost::shared_ptr<tf::TransformListener> tf_listener;
  boost::shared_ptr<message_filters::Subscriber<Image> > img_sub;
  boost::shared_ptr<message_filters::Subscriber<CameraInfo> > cam_sub;
  boost::shared_ptr<message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > > pointcloud_sub;
  boost::shared_ptr<Synchronizer> synch;
	boost::shared_ptr<cv::HOGDescriptor> people_detector;
  boost::shared_ptr<icl::PlaneFilter<PointXYZ> > floor_filter;
  boost::shared_ptr<icl::PlaneFilter<PointXYZ> > ceiling_filter;
  std::vector<size_t> downsampler;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered;
  ros::Timer spin_timer;
  double last_time;
  std::string tracking_frame;
  double sigma_a_dynamic, sigma_v_static, sigma_obs;
  double comfort_distance_sq;
  double birth_prior_weight;
  double prob_a, prob_b, weight_XX_percent;
  unscented_sampler unsctd_sampler;
  size_t previous_publish_estimations, previous_published_meas;

  void sensors_cb(const ImageConstPtr &img, const CameraInfoConstPtr &cam, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc) {
    // See if the filter is available to be updated
    const double time = pc->header.stamp.toSec();
    // Get needed trasforms
    Eigen::Matrix4d T_pt, T_tc, T_ct, T_cp;
    try {
      icl::lookupTransform(*tf_listener, tracking_frame, pc->header.frame_id, T_pt);
      icl::lookupTransform(*tf_listener, img->header.frame_id, tracking_frame, T_tc);
      icl::lookupTransform(*tf_listener, tracking_frame, img->header.frame_id, T_ct);
      icl::lookupTransform(*tf_listener, pc->header.frame_id, img->header.frame_id, T_cp);
    } catch(tf::TransformException e) {
      ROS_WARN_STREAM("Miss a transformation. " << e.what());
      return;
    }
    // The camera position and a point in front of the camera in tracking coordinates
    Eigen::Vector4d camera_pos = T_ct * (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished();
    Eigen::Vector4d camera_front = T_ct * (Eigen::Vector4d() << 0.0, 0.0, 1.0, 1.0).finished();
    Eigen::Matrix<double,3,4> P = (Eigen::Matrix<double,3,4>() << cam->P[0], cam->P[1], cam->P[2], cam->P[3],
         cam->P[4], cam->P[5], cam->P[6], cam->P[7],
         cam->P[8], cam->P[9], cam->P[10], cam->P[11]).finished();
    // Get the opencv image from the ROS message
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, enc::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image;
    cv::cvtColor(cv_ptr->image, image, CV_RGB2GRAY);

    // Extract the clusters from the laser
    DaTracker::weights birth_weights;
    DaTracker::states birth_means;
    DaTracker::state_mats birth_covs;

    // Preprocess the pointcloud: Downsample
    pcl::PointXYZ origin;
    origin.x = origin.y = origin.z = 0.0;
    for(size_t i = 0; i < downsampler.size(); i++) {
      filtered->points[i] = pc->points[downsampler[i]];
    }
    filtered->header = pc->header;
    filtered->width = 320;
    filtered->height = 240;
    // Preprocess the pointcloud: Remove the floor
    floor_filter->setInputCloud(filtered);
    floor_filter->filter(*filtered);
    // Preprocess the pointcloud: Remove the ceiling
    ceiling_filter->setInputCloud(filtered);
    ceiling_filter->filter(*filtered);
    filtered->is_dense = false;

    // Get the clusters as measurements
    std::vector<pcl::PointIndices> clusters;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > mean_centroids;
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > cov_centroids;
    icl::cluster_pointcloud(filtered, 0.45, 100, 1000000, comfort_distance_sq, clusters, mean_centroids, cov_centroids);

    // Extract the ROIs in the image and generate measurements
    DaTracker::measurements meas;
    for(size_t i = 0; i < clusters.size(); i++) {
      // A person shouldn't have the y components outside (0.1, 0.5)
      if(mean_centroids[i](1) <= 0.1 || mean_centroids[i](1) >= 0.5) {
        continue;
      }
      // If the mean position is too up (kinect y points down), discard this measurement
      // At this point this is condition enough to be a measurement
      Eigen::Vector4d point = T_pt * (Eigen::Vector4d() << mean_centroids[i], 1.0).finished();
      meas.push_back(point.block<2,1>(0,0));

      // Start creating the birth process
      std::vector<cv::Point> points;
      // Project all the points into the image space
      for(size_t k = 0; k < clusters[i].indices.size(); k++) {
        // Backproject some points
        const pcl::PointXYZ &p = filtered->points[clusters[i].indices[k]];
        // Assumes the depth sensors is on the same frame of reference as the colour camera
        Eigen::Vector3d pixel = P * (Eigen::Vector4d() << p.getVector3fMap().cast<double>(), 1.0).finished();
        pixel /= pixel[2];
        points.push_back(Point(pixel[0], pixel[1]));
      }
      // Get the bouding box of the points
      cv::Rect roi = boundingRect(points);
      roi &= cv::Rect(0, 0, cam->width, cam->height);
      // Check for wrong intersections
      if(roi.width== 0 || roi.height ==0) {
        ROS_ERROR("Problem with ROI");
        continue;
      }
      try {
        cv::Mat window;
        cv::resize(image(roi), window, people_detector->winSize);
        std::vector<cv::Point> hits;
        std::vector<double> w;
        people_detector->detect(window, hits, w, weight_XX_percent);
        // It should be 1 or 0 hits
        if(hits.size() > 0) {
          // Variables to hold the result
          DaTracker::state mean_birth = DaTracker::state::Zero();
          mean_birth.block<2,1>(0,0) = point.block<2,1>(0,0);
          DaTracker::state_mat cov_birth = sigma_v_static * DaTracker::state_mat::Identity();
          // Add the birth posibility
          birth_weights.push_back(birth_prior_weight / (1.0 + std::exp(prob_a * (*std::max_element(w.begin(), w.end())) + prob_b)));
          birth_means.push_back(mean_birth);
          birth_covs.push_back(cov_birth);
        }
      }
      catch(...) {
      }
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
    // Set the birth process prior
    d_tracker->set_birth_prior(birth_weights, birth_means, birth_covs);
    // Correct with the measurements
    d_tracker->correct(meas);
    // For debuging purposes
//    Eigen::Matrix4f trans = T_ct.cast<float>();
//    pcl::transformPointCloud(*filtered, *filtered, trans);
//    d_tracker->debug_plot(false, time, T_ct, filtered, meas);
    // Prune
    d_tracker->prune();

    // Get the estimated state
    pcl::PointCloud<icl::PointWithVelocity>::Ptr publish_pc(new pcl::PointCloud<icl::PointWithVelocity>);
    d_tracker->get_state(publish_pc, true, false);
    // Finish the pointcloud definitions
    publish_pc->width = 1;
    publish_pc->height = publish_pc->size();
    publish_pc->is_dense = true;
    publish_pc->header.frame_id = tracking_frame;
    publish_pc->header.stamp = pc->header.stamp;
    // Publish the results
    track_pub.publish(publish_pc);
    // Create the visualisation array
		visualization_msgs::MarkerArray mrks;
    // Fill with measurements
    for(size_t i = 0; i < std::max(meas.size(), previous_published_meas); i++) {
      // A sphere in the position
			visualization_msgs::Marker marker;
			marker.header = publish_pc->header;
			marker.ns = "d_tracker_measurement";
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
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        marker.lifetime = ros::Duration();
      } else {
				marker.action = visualization_msgs::Marker::DELETE;
      }
			mrks.markers.push_back(marker);
    }
    previous_published_meas = meas.size();
    // Fill with the esitmation
    fill_visualisation_msg(publish_pc, previous_publish_estimations, mrks);
    previous_publish_estimations = publish_pc->size();
    // Publish visualisation mesages
    vis_pub.publish(mrks);
  }

  DTrackerNode():
    main_nh(),
    priv_nh("~"),
    unsctd_sampler() {

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
    priv_nh.param("sigma_v_static", sigma_v_static, 0.25);
    priv_nh.param("sigma_a_dynamic", sigma_a_dynamic, 4.0);
    d_tracker->F = DaTracker::state_mat::Identity();
    d_tracker->T = sigma_v_static * DaTracker::state_mat::Identity();
    // Basic process observation
    d_tracker->H = (DaTracker::measurement_state_mat() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished();
    priv_nh.param("sigma_obs", sigma_obs, 0.05);
    d_tracker->R = sigma_obs * DaTracker::measurement_mat::Identity();

    // Parameters of the PHD filter
    priv_nh.param("birth_prior_weight", birth_prior_weight, 1E-5);
    priv_nh.param("probability_survival", d_tracker->p_survival, 0.99);
    priv_nh.param("probability_detect", d_tracker->p_detect, 0.95);
    priv_nh.param("clutter", d_tracker->clutter_rfs, 1.0);

    // Set the parameters
    priv_nh.param("truncate_threshold", d_tracker->truncate_threshold, 1E-6);
    priv_nh.param("merge_gauss_threshold", d_tracker->merge_gauss_threshold, 5E-1);
    priv_nh.param("merge_beta_threshold", d_tracker->merge_beta_threshold, 0.1);
    priv_nh.param("max_components", d_tracker->max_components, 100.0);

    // Comfort distance
    double comfort_distance;
    priv_nh.param("comfort_distance", comfort_distance, 0.6);
    comfort_distance_sq = comfort_distance * comfort_distance;

    // Parameters to convert SVM output to probability measurement
    priv_nh.param("svm_probability_a", prob_a, -0.95394079);
    priv_nh.param("svm_probability_b", prob_b, -1.18738746);
    // Calculate the weight threshold to get 50% certainty of a person
    double p_star;
    priv_nh.param("detection_certainty", p_star, 0.6);
    weight_XX_percent = (std::log(1.0 / p_star - 1.0) - prob_b) / prob_a;
    // Should never be positive, but just in case
    if(prob_a > 0)
      weight_XX_percent *= -1;

    // The transformation listener
    tf_listener.reset(new tf::TransformListener());

    // Initialise the last_time variable with the current time
    last_time = ros::Time::now().toSec();

    // Initialise the pointcloud filter
    floor_filter.reset(new icl::PlaneFilter<PointXYZ>(-0.00214303, -0.999838, -0.01785, 1.2, 0.0, true));
    ceiling_filter.reset(new icl::PlaneFilter<PointXYZ>(-0.00214303, -0.999838, -0.01785, -1.0, 0.0, false));

    downsampler.resize(320*240);
    for(size_t j = 0; j < 240; j++) {
      for(size_t i = 0; i < 320; i++) {
        downsampler[j*320 + i] = j * 2 * 640 + (i % 2) * (640 + 2) + (i / 2) * 4;
      }
    }
    filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    filtered->points.resize(downsampler.size());

    // Set up the people detector
		people_detector.reset(new cv::HOGDescriptor());
/*
		if(!people_detector->load("/data/ros/people_detect_and_tracking/mp_tracker/data/hogdetector.yaml")) {
			throw std::runtime_error("Could not load /data/ros/people_detect_and_tracking/mp_tracker/data/hogdetector.yaml");
		}
*/
    // Daimler people detector works the best
    people_detector->winSize = cv::Size(48, 96);
		people_detector->setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());

    std::string pointcloud_topic;
    priv_nh.param("pointcloud_topic", pointcloud_topic, std::string("pointcloud"));
    // Subscribe to the image and laser
    std::string camera_topic, camera_info_topic;
    priv_nh.param("camera_topic", camera_topic, std::string("camera"));
    priv_nh.param("camera_info_topic", camera_info_topic, std::string("camera_info"));
    img_sub.reset(new message_filters::Subscriber<Image>(main_nh, camera_topic, 4));
    cam_sub.reset(new message_filters::Subscriber<CameraInfo>(main_nh, camera_info_topic, 4));
    pointcloud_sub.reset(new message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> >(main_nh, pointcloud_topic, 4));
    synch.reset(new Synchronizer(Policy(10), *img_sub, *cam_sub, *pointcloud_sub));
    synch->registerCallback(boost::bind(&DTrackerNode::sensors_cb, this, _1, _2, _3));

    // Create the publisher
    track_pub = main_nh.advertise<pcl::PointCloud<icl::PointWithVelocity> >("tracked", 10);
    vis_pub = main_nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",10);
    previous_publish_estimations = 0;
    previous_published_meas = 0;

    // See if we load the plotting plugin
    std::string debug_script;
    priv_nh.param("debug_script", debug_script, std::string(""));
    if(!debug_script.empty()) {
      ROS_INFO_STREAM("Loading file: " << debug_script);
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


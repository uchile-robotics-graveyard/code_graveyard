#include "d_tracker.h"
#include "util.h"
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <util.h>
#include "pointcloud_process.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <unscented_sampler.hpp>
//#include "legsdetector.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// To avoid too many namespcae with the sensors
using namespace sensor_msgs;
namespace enc = sensor_msgs::image_encodings;

// Helpfull to avoid chainging all the apearances all the time
typedef icl::DTracker<4,2> DaTracker;

struct DTrackerNode {
  // Usefull typedefes
  typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, LaserScan> Policy;
  typedef message_filters::Synchronizer<Policy> Synchronizer;
  typedef icl::unscented_sampler<10> unscented_sampler;

  ros::NodeHandle main_nh;
  ros::NodeHandle priv_nh;
  DaTracker::ptr d_tracker;
  ros::Publisher track_pub, vis_pub;
  boost::shared_ptr<tf::TransformListener> tf_listener;
  boost::shared_ptr<message_filters::Subscriber<Image> > img_sub;
  boost::shared_ptr<message_filters::Subscriber<CameraInfo> > cam_sub;
  boost::shared_ptr<message_filters::Subscriber<LaserScan> > laser_sub;
  boost::shared_ptr<Synchronizer> synch;
	boost::shared_ptr<cv::HOGDescriptor> people_detector;
//  boost::shared_ptr<LegsDetector> leg_detector;
  ros::Timer spin_timer;
  double last_time;
  std::string tracking_frame;
  double sigma_a_dynamic, sigma_v_static, sigma_obs;
  double comfort_distance_sq;
  double cos_camera_fov, camera_min_dist_detect, camera_max_dist_detect;
  double birth_prior_weight;
  double prob_a, prob_b, weight_XX_percent;
  unscented_sampler unsctd_sampler;
  size_t previous_published;

  void sensors_cb(const ImageConstPtr &img, const CameraInfoConstPtr &cam, const LaserScanConstPtr &laser) {
    // See if the filter is available to be updated
    double time = ros::Time::now().toSec();
    // Get needed trasforms
    Eigen::Matrix4d T_lt, T_tc, T_ct, T_cl;
    try {
      icl::lookupTransform(*tf_listener, tracking_frame, laser->header.frame_id, T_lt);
      icl::lookupTransform(*tf_listener, img->header.frame_id, tracking_frame, T_tc);
      icl::lookupTransform(*tf_listener, tracking_frame, img->header.frame_id, T_ct);
      icl::lookupTransform(*tf_listener, laser->header.frame_id, img->header.frame_id, T_cl);
    } catch(tf::TransformException e) {
      std::cout << "Miss a transformation. " << e.what() << std::endl;
      return;
    }
    // The camera position and a point in front of the camera in tracking coordinates
    Eigen::Vector4d camera_pos = T_ct * (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished();
    Eigen::Vector4d camera_front = T_ct * (Eigen::Vector4d() << 0.0, 0.0, 1.0, 1.0).finished();
    double f = cam->P[0];
    Eigen::Matrix<double,3,4> P = (Eigen::Matrix<double,3,4>() << cam->P[0], cam->P[1], cam->P[2], cam->P[3],
         cam->P[4], cam->P[5], cam->P[6], cam->P[7],
         cam->P[8], cam->P[9], cam->P[10], cam->P[11]).finished();
    Eigen::Matrix<double,3,4> K = P * T_tc;

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

    // This variables are needed regardless of the detection method use
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pc;

    // Extract the clusters from the laser
    DaTracker::measurements meas;
    DaTracker::measurement_mats meas_covs;
    DaTracker::weights birth_weights;
    DaTracker::states birth_means;
    DaTracker::state_mats birth_covs;

    // This first block cluster the laser and get the corresponding detections from the image
    // Convert the laser to pointcloud
    laser_pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
    icl::pointcloud_from_laser(laser->ranges, laser->angle_min, laser->angle_increment, 30.0, T_lt, laser_pc);
    // Get the clusters as measurements
    icl::cluster_laser_2(laser_pc, 0.15, 5, 100, comfort_distance_sq, meas, meas_covs);
    // Extract the ROIs in the image
    Eigen::Vector2d front_normalized = (camera_front.block<2,1>(0,0) - camera_pos.block<2,1>(0,0)).normalized();
    // Vector with the detected clas
    std::vector<int> cat;
    for(size_t i = 0; i < meas.size(); i++) {
      std::vector<cv::Point> points;
      // Project all the points into the image space
      const Eigen::Vector2d point_to_measure = meas[i] - camera_pos.block<2,1>(0,0);
      const double d = point_to_measure.norm();
      const double cos_fov = point_to_measure.dot(front_normalized) / d;
      // Check the laser object is in the camera FOV
      if(d < camera_min_dist_detect
         || d > camera_max_dist_detect
         || cos_fov < cos_camera_fov) {
//        meas.erase(meas.begin() + i);
//        meas_covs.erase(meas_covs.begin() + i);
	cat.push_back(0);
        continue;
      }
      // A meter wide
      const double delta_pixles = 0.5 * f / d;
      // Backproject some points
      Eigen::Vector4d point1 = (Eigen::Vector4d() << meas[i](0), meas[i](1), 0.0 , 1.0).finished();
      Eigen::Vector3d pixel1 = K * point1; pixel1 /= pixel1[2];
      Eigen::Vector4d point2 = (Eigen::Vector4d() << meas[i](0), meas[i](1), 2.0 , 1.0).finished();
      Eigen::Vector3d pixel2 = K * point2; pixel2 /= pixel2[2];
      points.push_back(Point(pixel1[0], pixel1[1]));
      points.push_back(Point(pixel2[0], pixel2[1]));
      points.push_back(Point(pixel1[0]+delta_pixles, pixel1[1]));
      points.push_back(Point(pixel2[0]+delta_pixles, pixel2[1]));
      points.push_back(Point(pixel1[0]-delta_pixles, pixel1[1]));
      points.push_back(Point(pixel2[0]-delta_pixles, pixel2[1]));
      // Get the bouding box of the points
      cv::Rect roi = boundingRect(points) & cv::Rect(0, 0, cam->width, cam->height);
      // Check for wrong intersections
      if(roi.width== 0 || roi.height ==0) {
//        meas.erase(meas.begin() + i);
//       meas_covs.erase(meas_covs.begin() + i);
	cat.push_back(0);
        continue;
      }
      try {
        double fw = (double)(people_detector->winSize.width + people_detector->blockSize.width) / (double)roi.width;
        double fh = (double)(people_detector->winSize.height + people_detector->blockSize.height) / (double)roi.height;
        double scale = std::min(fw, fh);
        if(scale * roi.width < people_detector->winSize.width
           || scale * roi.height < people_detector->winSize.height)
          scale = std::max(fw, fh);
        cv::Size new_size(std::floor(roi.width * scale + 0.5),
                          std::floor(roi.height * scale + 0.5));
        cv::Mat window;
        cv::resize(image(roi), window, new_size);
        std::vector<cv::Point> hits;
        std::vector<double> w;
        people_detector->detect(window, hits, w, weight_XX_percent);
/*
        // For any hit create a new birth process
        for(size_t k = 0; k < hits.size(); k++) {
          const double var_distance = 1.11E-3;
          const double translation_var = 0.0004;
          const double rotation_var = 1.11E-5;
          // Translation and rotarion(quaternion)
          Eigen::Vector3d trans = T_cl.block<3,1>(0,3);
          Eigen::Quaternion<double> rot(T_cl.block<3,3>(0,0));
          // The unscented sampler object
          unscented_sampler::mean_type mean_detect;
          // Mean that the unscented sampler will use
          mean_detect << roi.x + (hits[k].x + people_detector->winSize.width / 2) / scale,
                         roi.y + (hits[k].y + people_detector->winSize.height / 2) / scale,
                         d,
                         trans(0),
                         trans(1),
                         trans(2),
                         rot.w(),
                         rot.x(),
                         rot.y(),
                         rot.z();
          const double sigma_detect = std::pow(5.0 / scale, 2);
          unscented_sampler::cov_type cov_detect = sigma_detect * unscented_sampler::cov_type::Identity();
          // Distance variance
          cov_detect(2,2) = var_distance;
          // Translation variance
          cov_detect.block<3,3>(3,3) = translation_var * Eigen::Matrix3d::Identity();
          // Rotation variance
          cov_detect.block<4,4>(6,6) = rotation_var * Eigen::Matrix4d::Identity();
          std::vector<unscented_sampler::mean_type,
                      Eigen::aligned_allocator<unscented_sampler::mean_type> > sigma_points;
          std::vector<double> sigma_weights;
          // Create the sigma points
          unsctd_sampler.get_points(mean_detect, cov_detect, 0.2, 1.0, sigma_points, sigma_weights);
          // Variables to hold the result
          DaTracker::states state_estimations(sigma_weights.size());
          DaTracker::state mean_birth = DaTracker::state::Zero();
          DaTracker::state_mat cov_birth = DaTracker::state_mat::Zero();
          for(size_t j = 0; j < sigma_points.size(); j++) {
            // Get the sigma point
            const unscented_sampler::mean_type &point = sigma_points[j];
            // With all the data, backproject the mesurement to real world coordinates
            double expected_distance = point(2);
            Eigen::Quaternion<double> rotp(point(6), point(7), point(8), point(9));
            rotp.normalize();
            Eigen::Matrix4d Tp = Eigen::Matrix4d::Identity();
            Tp.block<3,3>(0,0) = rotp.matrix();
            Tp.block<3,1>(0,3) << point(3), point(4), point(5);
            // This abuses the fact that the state is dim=4, although only the (x,y) components
            // are updated in this step
            state_estimations[j] = T_lt * Tp * (Eigen::Matrix<double,4,1>() <<
              (point(0) - P(0,2) - P(0,3)) / P(0,0) * expected_distance,
              (point(1) - P(1,2) - P(1,3)) / P(1,1) * expected_distance,
              expected_distance,
              1.0).finished();
            // Velocity components
            state_estimations[j](2) = 0.0;
            state_estimations[j](3) = 0.0;
            mean_birth += sigma_weights[j] * state_estimations[j];
          }
          for(size_t j = 0; j < sigma_points.size(); j++) {
            Eigen::Vector2d dx =  state_estimations[j].block<2,1>(0,0) - mean_birth.block<2,1>(0,0);
            cov_birth.block<2,2>(0,0) += sigma_weights[j] * dx * dx.transpose();
          }
          cov_birth.block<2,2>(2,2) = sigma_v_static * Eigen::Matrix2d::Identity();
          // Add the birth posibility
          birth_weights.push_back(1.0 / (1.0 + std::exp(prob_a * w[k] + prob_b)) / hits.size());
          birth_means.push_back(mean_birth);
          birth_covs.push_back(cov_birth);
        }
*/
        if(hits.size() > 0) {
          DaTracker::state mean_birth = DaTracker::state::Zero();
          mean_birth.block<2,1>(0,0) = meas[i];
          birth_means.push_back(mean_birth);
          DaTracker::state_mat cov_birth = sigma_v_static * DaTracker::state_mat::Identity();
          cov_birth.block<2,2>(0,0) = meas_covs[i];
          birth_covs.push_back(cov_birth);
          //birth_weights.push_back(birth_prior_weight / (1.0 + std::exp(prob_a * w[0] + prob_b)) / hits.size());
          birth_weights.push_back(birth_prior_weight);

	  cat.push_back(1);
        } else {
	  cat.push_back(-1);
	}
      }
      catch(...) {
        ROS_WARN("Somehting happend. Catch some exception.");
        meas.erase(meas.begin() + i);
        meas_covs.erase(meas_covs.begin() + i);
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
    d_tracker->clutter_rfs = 1.0;
    // Set the birth process prior
    d_tracker->set_birth_prior(birth_weights, birth_means, birth_covs);
    // Correct with the measurements
    d_tracker->correct(meas, cat);
    // Prune
    d_tracker->prune();
    // For debuging purposes
    d_tracker->debug_plot(true, time, T_lt, laser_pc, meas);
    // Get the estimated state
    pcl::PointCloud<icl::PointWithVelocity>::Ptr pc(new pcl::PointCloud<icl::PointWithVelocity>);
    d_tracker->get_state(pc, true, false);
    // Finish the pointcloud definitions
    pc->width = 1;
    pc->height = pc->size();
    pc->is_dense = true;
    pc->header.frame_id = tracking_frame;
    pc->header.stamp.fromSec(last_time);
    // Publish the results
    track_pub.publish(pc);
    // Create the visualisation array
		visualization_msgs::MarkerArray mrks;
    // Fill the visualisation msg
    fill_visualisation_msg(pc, previous_published, mrks);
    previous_published = pc->size();
    // Publish visualisation msg
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
    priv_nh.param("sigma_obs", sigma_obs, 0.01);
    d_tracker->R = sigma_obs * DaTracker::measurement_mat::Identity();

    // Parameters of the PHD filter
    priv_nh.param("birth_prior_weight", birth_prior_weight, 1E-5);
    priv_nh.param("probability_survival", d_tracker->p_survival, 0.99);
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

    // Camera FOV, in radians
    double fov;
    priv_nh.param("camera_fov", fov, 0.994837674);
    cos_camera_fov = std::cos(fov);
    priv_nh.param("camera_min_dist_detect", camera_min_dist_detect, 2.0);
    priv_nh.param("camera_max_dist_detect", camera_max_dist_detect, 11.0);

    // Parameters to convert SVM output to probability measurement
    priv_nh.param("svn__probability_a", prob_a, -0.95394079);
    priv_nh.param("svn__probability_b", prob_b, -1.18738746);
    // Calculate the weight threshold to get 50% certainty of a person
    double p_star;
    priv_nh.param("detection_certainty", p_star, 0.6);
    weight_XX_percent = (std::log(1.0 / p_star) - prob_b) / prob_a;
    // Should never be positive, but just in case
    if(prob_a > 0)
      weight_XX_percent *= -1;

    // The transformation listener
    tf_listener.reset(new tf::TransformListener());

    // Initialise the last_time variable with the current time
    last_time = ros::Time::now().toSec();
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

    std::string laser_topic;
    priv_nh.param("laser_topic", laser_topic, std::string("scan"));
    // Subscribe to the image and laser
    std::string camera_topic, camera_info_topic;
    priv_nh.param("camera_topic", camera_topic, std::string("camera"));
    priv_nh.param("camera_info_topic", camera_info_topic, std::string("camera_info"));
    img_sub.reset(new message_filters::Subscriber<Image>(main_nh, camera_topic, 1));
    cam_sub.reset(new message_filters::Subscriber<CameraInfo>(main_nh, camera_info_topic, 1));
    laser_sub.reset(new message_filters::Subscriber<LaserScan>(main_nh, laser_topic, 1));
    synch.reset(new Synchronizer(Policy(10), *img_sub, *cam_sub, *laser_sub));
    synch->registerCallback(boost::bind(&DTrackerNode::sensors_cb, this, _1, _2, _3));

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


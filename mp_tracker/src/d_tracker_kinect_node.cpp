#ifdef PHD_FILTER
#include "d_tracker.h"
#else
#include "gb_tracker.h"
#endif
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
#include <pcl/features/integral_image_normal.h>
//#include <pcl/surface/mls.h>
//#include <pcl/surface/impl/mls.hpp>
#include <pcl/search/organized.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <util.h>
#include "pointcloud_process.h"
#include "depth_connected_components.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <unscented_sampler.hpp>
#include <icl_robot/plane_filter.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/progress.hpp>

// To avoid too many namespcae with the sensors
using namespace sensor_msgs;
namespace enc = sensor_msgs::image_encodings;

const unsigned char colours[][3] = {{255,255,255}, {255, 255, 128}, {255, 255, 0},
                                    {255, 128, 0}, {255, 0, 0},
                                    {255, 0, 128}, {255, 0, 255},
                                    {128, 0, 255}, {0, 0, 255},
                                    {0, 128, 255}, {0, 255, 255},
                                    {0, 255, 128}, {0, 255, 0}};

// Helpfull to avoid chainging all the apearances all the time
#ifdef PHD_FILTER
typedef icl::DTracker<6,3> DaTracker;
#else
typedef icl::GBTracker<6,3> DaTracker;
#endif

#ifndef PHD_FILTER
DaTracker::tracker_component creation(const DaTracker::measurement &obs,
                                      int classification,
                                      const DaTracker::colour_projection_type &colour) {
  typedef typename DaTracker::colour_projection_type colour_projection_type;
  DaTracker::tracker_component tracker;
  // Create the Gaussian component
  tracker.mean = DaTracker::state::Zero();
  tracker.mean.block<3,1>(0,0) = obs;
  tracker.cov = (0.05*0.05)*DaTracker::state_mat::Identity();
  // Create the gaussians components for the colour
  tracker.colour_projection_mean = colour;
  tracker.colour_projection_std.resize(colour.size());
  tracker.colour_projection_std.setConstant(20.0);
  // Create the Beta component
  if(classification > 0) {
    tracker.a = 8.0;
    tracker.b = 2.0;
  }
  else if(classification < 0) {
    tracker.a = 2.0;
    tracker.b = 8.0;
  }
  else {
    tracker.a = 0.5;
    tracker.b = 0.5;
  }
  return tracker;
}
#endif

struct multi_tresholdd {
  float distance;
  float angle;
  float colour;

  multi_tresholdd(float a, float b, float c): distance(a), angle(b), colour(c) {}

  bool operator<(const multi_tresholdd &b) {
    return this->distance < b.distance
            && this->colour < b.colour
            && this->angle > b.angle;
  }
};

multi_tresholdd point_compare(const pcl::PointXYZRGBNormal &a, const pcl::PointXYZRGBNormal &b) {
  float dr = a.r - b.r;
  float dg = a.g - b.g;
  float db = a.b - b.b;
  return multi_tresholdd(pcl::euclideanDistance(a, b),
                          std::cos(a.normal_x * b.normal_x
                                  +a.normal_y * b.normal_y
                                  +a.normal_z * b.normal_z),
                          std::sqrt(dr*dr + dg*dg + db*db));
}

struct DTrackerNode {
  // Usefull typedefes
  typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, pcl::PointCloud<pcl::PointXYZRGB> > Policy;
  typedef message_filters::Synchronizer<Policy> Synchronizer;
//  typedef icl::unscented_sampler<11> unscented_sampler;

  ros::NodeHandle main_nh, priv_nh;
  DaTracker::ptr d_tracker;
  ros::Publisher track_pub, vis_pub, debug_pub;
  boost::shared_ptr<tf::TransformListener> tf_listener;
//  boost::shared_ptr<message_filters::Subscriber<Image> > img_sub;
//  boost::shared_ptr<message_filters::Subscriber<CameraInfo> > cam_sub;
//  boost::shared_ptr<message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB> > > pointcloud_sub;
//  boost::shared_ptr<Synchronizer> synch;
  ros::Subscriber pc_sub;
//	boost::shared_ptr<cv::HOGDescriptor> people_detector;
  boost::shared_ptr<icl::PlaneFilter<PointXYZRGB> > floor_filter;
  boost::shared_ptr<icl::PlaneFilter<PointXYZRGB> > ceiling_filter;
  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
  std::vector<size_t> downsampler;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered;
  ros::Timer spin_timer;
  double last_time;
  std::string tracking_frame;
  double sigma_a_dynamic, sigma_v_static, sigma_obs;
  double comfort_distance_sq;
  double birth_prior_weight;
  double prob_a, prob_b, weight_XX_percent;
//  unscented_sampler unsctd_sampler;
  size_t previous_publish_estimations, previous_published_meas;

  void sensors_cb(/*const ImageConstPtr &img, const CameraInfoConstPtr &cam,*/ const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc) {
    boost::progress_timer t;
    // See if the filter is available to be updated
    const double time = pc->header.stamp.toSec();
    // Get needed trasforms
    Eigen::Matrix4d T_pt, T_tc, T_ct, T_cp;
    try {
      icl::lookupTransform(*tf_listener, tracking_frame, pc->header.frame_id, T_pt);
//      icl::lookupTransform(*tf_listener, img->header.frame_id, tracking_frame, T_tc);
//      icl::lookupTransform(*tf_listener, tracking_frame, img->header.frame_id, T_ct);
//      icl::lookupTransform(*tf_listener, pc->header.frame_id, img->header.frame_id, T_cp);
    } catch(tf::TransformException e) {
      ROS_WARN_STREAM("Miss a transformation. " << e.what());
      return;
    }
    // Preprocess the pointcloud: Downsample
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

    // Calculate normals
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final_pc(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

#if 0
    mls.setInputCloud(filtered);
    pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal>::Ptr org(new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal>);
    mls.setSearchMethod(boost::dynamic_pointer_cast<search::Search<PointXYZRGB> >(org));
    mls.setSearchRadius(0.03);
    mls.process(*final_pc);
#endif

    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(filtered);
    ne.compute(*normal);

    final_pc->resize(filtered->size());
    final_pc->is_dense = false;
    final_pc->header = filtered->header;
    final_pc->height = filtered->height;
    final_pc->width = filtered->width;
    for(size_t i = 0; i < filtered->size(); i++) {
      final_pc->points[i].x = filtered->points[i].x;
      final_pc->points[i].y = filtered->points[i].y;
      final_pc->points[i].z = filtered->points[i].z;
      final_pc->points[i].normal_x = normal->points[i].normal_x;
      final_pc->points[i].normal_y = normal->points[i].normal_y;
      final_pc->points[i].normal_z = normal->points[i].normal_z;
      final_pc->points[i].b = filtered->points[i].b;
      final_pc->points[i].g = filtered->points[i].g;
      final_pc->points[i].r = filtered->points[i].r;
    }

    // Blur the colour components of the pointcloud
    cv::Mat raw_data(final_pc->width*final_pc->height, 1, CV_8UC3, (void*)&final_pc->points[0].rgba, sizeof(pcl::PointXYZRGBNormal));
    cv::Mat image = raw_data.clone().reshape(0,final_pc->height);
    cv::Mat blured_image(final_pc->height, final_pc->width, CV_8UC3);
    cv::GaussianBlur(image, blured_image, cv::Size(11,11), 0, 0);
    blured_image.reshape(0, raw_data.rows).copyTo(raw_data);

#ifdef PHD_FILTER
    // Extract the clusters from the laser
    DaTracker::weights birth_weights;
    DaTracker::states birth_means;
    DaTracker::state_mats birth_covs;
#endif
    // Get the clusters as measurements
    std::vector<pcl::PointIndices> clusters;
    const multi_tresholdd eps(0.2, 0.4, 10.0);

    icl::extractEuclideanClustersFromImage<pcl::PointXYZRGBNormal, multi_tresholdd>(*final_pc, clusters, eps, 5, 1000000, &point_compare);
    for(size_t i = 0; i < clusters.size(); i++) {
      for(size_t j = 0; j < clusters[i].indices.size(); j++) {
        const size_t &indx = clusters[i].indices[j];
        final_pc->points[indx].r = colours[i%13][0];
        final_pc->points[indx].g = colours[i%13][1];
        final_pc->points[indx].b = colours[i%13][2];
      }
    }
    debug_pub.publish(final_pc);

//    pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal>::Ptr org(new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal>);
//    org->setInputCloud(final_pc);
//    pcl::extractEuclideanClusters(*final_pc, boost::dynamic_pointer_cast<search::Search<PointXYZRGBNormal> >(org), 0.45, clusters, 100, 10000000);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > mean_centroids(clusters.size());
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > cov_centroids(clusters.size());
    // Generate the final centroids
    for(size_t i = 0; i < clusters.size(); i++) {
        mean_centroids[i] = Vector3d::Zero();
        Eigen::Matrix<double, 3, Eigen::Dynamic> xp(3, clusters[i].indices.size());
        for(size_t j = 0; j < clusters[i].indices.size(); j++) {
          const size_t &indx = clusters[i].indices[j];
          mean_centroids[i] += final_pc->points[indx].getVector3fMap().cast<double>();
        }
        mean_centroids[i] /= clusters[i].indices.size();
        for(size_t j = 0; j < clusters[i].indices.size(); j++) {
          const size_t &indx = clusters[i].indices[j];
          xp.col(j) = final_pc->points[indx].getVector3fMap().cast<double>() - mean_centroids[i];
        }
        cov_centroids[i] = xp * xp.transpose() / (clusters[i].indices.size() - 1.0);
    }
    ROS_INFO_STREAM("HERE clusters: " << clusters.size());

    // Extract the ROIs in the image and generate measurements
    DaTracker::measurements meas;
    for(size_t i = 0; i < clusters.size(); i++) {
      // If the mean position is too up (kinect y points down), discard this measurement
      // At this point this is condition enough to be a measurement
      Eigen::Vector4d point = T_pt * (Eigen::Vector4d() << mean_centroids[i], 1.0).finished();
      meas.push_back(point.block<3,1>(0,0));
#ifdef PHD_FILTER
      // Variables to hold the result
      DaTracker::state mean_birth = DaTracker::state::Zero();
      mean_birth.block<3,1>(0,0) = point.block<3,1>(0,0);
      DaTracker::state_mat cov_birth = sigma_v_static * DaTracker::state_mat::Identity();
      // Add the birth posibility
      birth_weights.push_back(birth_prior_weight);
      birth_means.push_back(mean_birth);
      birth_covs.push_back(cov_birth);
#endif
    }
    // Do the filtering
    double dt = time - last_time;
    last_time = time;
    // Predict evolution of the system
    Eigen::Matrix<double,6,3> w;
    w << dt*dt / 2.0,         0.0,         0.0,
                 0.0, dt*dt / 2.0,         0.0,
                 0.0,         0.0, dt*dt / 2.0,
                  dt,         0.0,         0.0,
                 0.0,          dt,         0.0,
                 0.0,         0.0,          dt;
    d_tracker->Q = sigma_a_dynamic * (w * w.transpose());
    d_tracker->F(0,3) = dt;
    d_tracker->F(1,4) = dt;
    d_tracker->F(2,5) = dt;
    d_tracker->predict();
#ifdef PHD_FILTER
    // Set the birth process prior
    d_tracker->set_birth_prior(birth_weights, birth_means, birth_covs);
#endif
    // Correct with the measurements
#ifdef PHD_FILTER
    d_tracker->correct(meas);
#else
    std::vector<double> d1;
    DaTracker::colour_projections_type d2;
    d_tracker->correct(meas, d1, d2);
#endif
    ROS_INFO_STREAM("HERE measurements: " << meas.size());
    // For debuging purposes
//    Eigen::Matrix4f trans = T_ct.cast<float>();
//    pcl::transformPointCloud(*filtered, *filtered, trans);
//    d_tracker->debug_plot(false, time, T_ct, filtered, meas);
    // Prune
#ifdef PHD_FILTER
    d_tracker->prune();
#endif

    // Get the estimated state
    pcl::PointCloud<icl::PointWithVelocity>::Ptr publish_pc(new pcl::PointCloud<icl::PointWithVelocity>);
#ifdef PHD_FILTER
    d_tracker->get_state(publish_pc, true, false);
#else
    d_tracker->get_state(publish_pc);
#endif
    // Finish the pointcloud definitions
    publish_pc->width = 1;
    publish_pc->height = publish_pc->size();
    publish_pc->is_dense = true;
    publish_pc->header.frame_id = tracking_frame;
    publish_pc->header.stamp = pc->header.stamp;
    // Publish the results
    track_pub.publish(publish_pc);
    ROS_INFO_STREAM("HERE, estimated points: " << publish_pc->size());
    // Create the visualisation array
		visualization_msgs::MarkerArray mrks;
    // Fill with measurements
/*
#define SHOW meas
    for(size_t i = 0; i < std::max(SHOW.size(), previous_published_meas); i++) {
      // A sphere in the position
			visualization_msgs::Marker marker;
			marker.header = publish_pc->header;
			marker.ns = "d_tracker_measurement";
			marker.id = i;
			marker.type = visualization_msgs::Marker::CUBE;
      if(i < SHOW.size()) {
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = SHOW[i](0);
				marker.pose.position.y = SHOW[i](1);
				marker.pose.position.z = SHOW[i](2);
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
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
*/
    // Fill with the esitmation
    fill_visualisation_msg(publish_pc, previous_publish_estimations, mrks);
    previous_publish_estimations = publish_pc->size();
    // Publish visualisation mesages
    vis_pub.publish(mrks);
  }

  DTrackerNode():
    main_nh(),
    priv_nh("~") {
//    , unsctd_sampler() {

    // Basic frames
    priv_nh.param("tracking_frame", tracking_frame, std::string("odom"));

    // Use beta-gaussian or gaussian only
#ifdef PHD_FILTER
    d_tracker.reset(new DaTracker(false, false, false));
#else
    d_tracker.reset(new DaTracker());
#endif
    d_tracker->use_beta = false;
    d_tracker->use_colour = false;
#ifdef PHD_FILTER
    d_tracker->clear_birth_prior();
#endif

    // Use range-bearing or cartesian observations
    d_tracker->sigma_h = 0.01;

    // Basic process model
    priv_nh.param("sigma_v_static", sigma_v_static, 0.25);
    priv_nh.param("sigma_a_dynamic", sigma_a_dynamic, 4.0);
    d_tracker->F = DaTracker::state_mat::Identity();
#ifdef PHD_FILTER
    d_tracker->T = sigma_v_static * DaTracker::state_mat::Identity();
#endif
    // Basic process observation
    d_tracker->H = (DaTracker::measurement_state_mat() << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                          0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                                          0.0, 0.0, 1.0, 0.0, 0.0, 0.0).finished();
    priv_nh.param("sigma_obs", sigma_obs, 0.05);
    d_tracker->R = sigma_obs * DaTracker::measurement_mat::Identity();

    // Parameters of the PHD filter
    priv_nh.param("birth_prior_weight", birth_prior_weight, 1E-5);
#ifdef PHD_FILTER
    priv_nh.param("probability_survival", d_tracker->p_survival, 0.99);
    priv_nh.param("probability_detect", d_tracker->p_detect, 0.95);
    priv_nh.param("clutter", d_tracker->clutter_rfs, 1E-3);
    // Set the parameters
    priv_nh.param("truncate_threshold", d_tracker->truncate_threshold, 1E-6);
    priv_nh.param("merge_gauss_threshold", d_tracker->merge_gauss_threshold, 5E-1);
    priv_nh.param("max_components", d_tracker->max_components, 200.0);
#else
    d_tracker->state_creator = &creation;
#endif

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
    floor_filter.reset(new icl::PlaneFilter<PointXYZRGB>(-0.00214303, -0.999838, -0.01785, 1.2, 0.0, true));
    ceiling_filter.reset(new icl::PlaneFilter<PointXYZRGB>(-0.00214303, -0.999838, -0.01785, -1.0, 0.0, false));

    // Normal estimation parameters
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.2);
    ne.setNormalSmoothingSize(5.0);

//    mls.setComputeNormals(true);
//    mls.setPolynomialFit(true);

    downsampler.resize(320*240);
    for(size_t j = 0; j < 240; j++) {
      for(size_t i = 0; i < 320; i++) {
        downsampler[j*320 + i] = j * 2 * 640 + (i % 2) * (640 + 2) + (i / 2) * 4;
      }
    }
    filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    filtered->points.resize(downsampler.size());

//    // Set up the people detector
//		people_detector.reset(new cv::HOGDescriptor());
/*
		if(!people_detector->load("/data/ros/people_detect_and_tracking/mp_tracker/data/hogdetector.yaml")) {
			throw std::runtime_error("Could not load /data/ros/people_detect_and_tracking/mp_tracker/data/hogdetector.yaml");
		}
*/
//    // Daimler people detector works the best
//    people_detector->winSize = cv::Size(48, 96);
//		people_detector->setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());

    std::string pointcloud_topic;
    priv_nh.param("pointcloud_topic", pointcloud_topic, std::string("pointcloud"));
//    // Subscribe to the image and laser
//    std::string camera_topic, camera_info_topic;
//    priv_nh.param("camera_topic", camera_topic, std::string("camera"));
//    priv_nh.param("camera_info_topic", camera_info_topic, std::string("camera_info"));
//    img_sub.reset(new message_filters::Subscriber<Image>(main_nh, camera_topic, 4));
//    cam_sub.reset(new message_filters::Subscriber<CameraInfo>(main_nh, camera_info_topic, 4));
//    pointcloud_sub.reset(new message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB> >(main_nh, pointcloud_topic, 4));
//    synch.reset(new Synchronizer(Policy(10), *img_sub, *cam_sub, *pointcloud_sub));
//    synch->registerCallback(boost::bind(&DTrackerNode::sensors_cb, this, _1, _2, _3));
    pc_sub = main_nh.subscribe(pointcloud_topic, 10, &DTrackerNode::sensors_cb, this); 
    // Create the publisher
    track_pub = main_nh.advertise<pcl::PointCloud<icl::PointWithVelocity> >("tracked", 10);
    debug_pub = main_nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("debug_pc", 10);
    vis_pub = main_nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",10);
    previous_publish_estimations = 0;
    previous_published_meas = 0;

#ifdef PHD_FILTER
    // See if we load the plotting plugin
    std::string debug_script;
    priv_nh.param("debug_script", debug_script, std::string(""));
    if(!debug_script.empty()) {
      ROS_INFO_STREAM("Loading file: " << debug_script);
      d_tracker->load_debug_plugin(debug_script);
      if(d_tracker->python_loaded)
        ROS_INFO("Debuging python module loaded");
    }
#endif
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

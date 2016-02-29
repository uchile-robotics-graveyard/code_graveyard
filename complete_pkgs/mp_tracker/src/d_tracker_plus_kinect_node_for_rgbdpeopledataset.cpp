#define OBS_BETA
#include "d_tracker.h"
#include "util.h"
#include "pointcloud_process.h"
#include "compressed_features.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <util.h>
#include "depth_connected_components.h"
#include <opencv2/opencv.hpp>
#include <icl_robot/plane_filter.h>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <boost/program_options.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#ifdef DEPTH_DETECTOR
#ifndef PCL17
namespace pcl {
struct RGB {
	union {
    union {
      struct {
        uint8_t b,g,r,a;
      };
      float rgb;
    };
    uint32_t rgba;
  };
};
}

POINT_CLOUD_REGISTER_POINT_STRUCT (
		pcl::RGB,
		(uint32_t, rgba, rgba)
);
#endif
#include "people/ground_based_people_detection_app.h"
#endif

// To avoid too many namespcae with the sensors
using namespace cv;
using namespace std;
using namespace Eigen;
using namespace pcl;
namespace po = boost::program_options;

#ifndef NOROS
ros::Time now;
#endif
size_t previous_published_tracks;

// Helpfull to avoid chainging all the apearances all the time
typedef icl::DTracker<4,2> DaTracker;

struct DTrackerNode {
  DaTracker::ptr d_tracker;
#ifdef DEPTH_DETECTOR
  people::PersonClassifier<pcl::RGB> person_classifier;
  people::GroundBasedPeopleDetectionApp<PointXYZRGB> people_detector;
#else
	boost::shared_ptr<HOGDescriptor> people_detector;
#endif
  boost::shared_ptr<icl::PlaneFilter<PointXYZRGB> > floor_filter;
  Vector4d floor_hessian;
  boost::shared_ptr<icl::PlaneFilter<PointXYZRGB> > ceiling_filter;

  vector<size_t> downsampler;
  double sigma_a_dynamic, sigma_v_static, sigma_obs;
  double comfort_distance_sq;
  double birth_prior_weight;
  double prob_a, prob_b, weight_is_person, weight_is_not_person;

  // For visualization purposes
  visualization_msgs::MarkerArray mrks;
  size_t previous_published, previous_published_meas;

  Mat current_image;
  Matrix<double,3,4> P;

  // Assumes the state has the estimated height of a person
  double image_likelihood(const DaTracker::state &x) {
    Vector4d feet_point, head_point;
    feet_point << x.block<2,1>(0,0), 0.0, 1.0;
    head_point = feet_point;
    head_point(2) += x(2);
    Vector3d pixel_feet = P * feet_point;
    pixel_feet /= pixel_feet[2];
    Vector3d pixel_head = P * head_point;
    pixel_head /= pixel_head[2];
    double height = fabs(pixel_feet(1) - pixel_head(1));
    double width = people_detector->winSize.width / people_detector->winSize.height * height;
    double px = feet_point(0) - width / 2.0;
    double py = head_point(1);

    Mat roi = current_image(Rect(px, py, width, height));
  }

  void sensors_cb(Mat &img1,
                  const PointCloud<PointXYZRGB>::Ptr &pc1,
                  const Matrix4d &T1,
                  Mat &img2,
                  const PointCloud<PointXYZRGB>::Ptr &pc2,
                  const Matrix4d &T2, 
                  Mat &img3,
                  const PointCloud<PointXYZRGB>::Ptr &pc3,
                  const Matrix4d &T3) {

    mrks.markers.resize(0);
    DaTracker::weights birth_weights;
    DaTracker::states birth_means;
    DaTracker::state_mats birth_covs;
    DaTracker::measurements meas;
#ifdef OBS_BETA
    std::vector<int> classes;
#endif
    // Process all pointclouds
    for(size_t n = 0; n < 3; n++) {
      PointCloud<PointXYZRGB>::Ptr pc = (n == 0 ? pc1: (n == 1 ? pc2 : pc3));
      const Matrix4d &T = (n == 0 ? T1: (n == 1 ? T2 : T3));
#ifdef DEPTH_DETECTOR
      // Perform people detection on the new cloud:
      std::vector<people::PersonCluster<PointXYZRGB> > clusters;
      people_detector.setInputCloud(pc);
      VectorXf floor_params(4);
      floor_params = floor_hessian.cast<float>();
      people_detector.setGround(floor_params);
      people_detector.compute(clusters);

      for(std::vector<pcl::people::PersonCluster<PointXYZRGB> >::iterator it = clusters.begin(); it != clusters.end(); ++it) {
        Vector4d point = T * (Vector4d() << it->getCenter().cast<double>(), 1.0).finished();
        if(it->getPersonConfidence() > weight_is_not_person) {
          meas.push_back(point.block<2,1>(0,0));
        }
      }
#else
      Mat &imagergb = (n == 0 ? img1: (n == 1 ? img2 : img3));
      current_image = imagergb;
      // Preprocess the pointcloud: Remove the floor
      floor_filter->setInputCloud(pc);
      floor_filter->filter(*pc);
#ifndef PEOPLE_HEAD_SUBCLUSTER
      // Preprocess the pointcloud: Remove the ceiling
      ceiling_filter->setInputCloud(pc);
      ceiling_filter->filter(*pc);
#endif
      pc->is_dense = false;

      // Get the clusters as measurements
      vector<PointIndices> clusters;
      vector<Vector3d, aligned_allocator<Vector3d> > mean_centroids;
      vector<Matrix3d, aligned_allocator<Matrix3d> > cov_centroids;
      icl::cluster_pointcloud_special(pc, 0.14, 1500, 100000000, comfort_distance_sq, clusters, mean_centroids, cov_centroids);

      // Extract the ROIs in the image and generate measurements
      for(size_t i = 0; i < clusters.size(); i++) {
        // A person shouldn't have the x components outside (0.1, 0.5)
        // If the mean position is too up (kinect y points down), discard this measurement
        Vector4d point = T * (Vector4d() << mean_centroids[i], 1.0).finished();
        if(point(2) <= -0.8 || point(2) >= 0.3) {
          continue;
        }
        // At this point this is condition enough to be a measurement
        meas.push_back(point.block<2,1>(0,0));

        // Start creating the birth process
        vector<Point> points;
        // Project all the points into the image space
        for(size_t k = 0; k < clusters[i].indices.size(); k++) {
          // Backproject some points
          const PointXYZRGB &p = pc->points[clusters[i].indices[k]];
          // Assumes the depth sensors is on the same frame of reference as the colour camera
          Vector3d floor_point = p.getVector3fMap().cast<double>();
          Vector3d pixel = P * (Vector4d() << p.getVector3fMap().cast<double>(), 1.0).finished();
          pixel /= pixel[2];
          points.push_back(Point(pixel[0], pixel[1]));
          // Project the pixel into the floor
          Vector3d point_floor = floor_point - (floor_point.dot(floor_hessian.block<3,1>(0,0)) + floor_hessian(3)) * floor_hessian.block<3,1>(0,0);
          Vector3d pixel_floor = P * (Vector4d() << point_floor, 1.0).finished();
          pixel_floor /= pixel_floor(2);
          points.push_back(Point(pixel_floor(0), pixel_floor(1)));
        }
        // Get the bouding box of the points
        Rect roi = boundingRect(points);
//        roi += Point(-32, -32);
//        roi += Size(64, 64);
        roi &= Rect(0, 0, imagergb.cols, imagergb.rows);
        // Check for wrong intersections
        if(roi.width == 0 || roi.height == 0) {
          cerr << "Problem with ROI" << endl;
#ifdef OBS_BETA
          classes.push_back(0);
#endif
          continue;
        }
        try {
          Mat subimg1rgb = imagergb(roi).t();

          Mat map_x(subimg1rgb.size(), CV_32FC1), map_y(subimg1rgb.size(), CV_32FC1), subimg2rgb;
          for(int u = 0; u < subimg1rgb.rows; u++) {
            for(int v = 0; v < subimg1rgb.cols; v++) {
              map_x.at<float>(u,v) = v;
              map_y.at<float>(u,v) = subimg1rgb.rows-u;
            }
          }
          remap(subimg1rgb, subimg2rgb, map_x, map_y, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
          Mat windowrgb;
          resize(subimg2rgb, windowrgb, people_detector->winSize);

//          vector<Mat> channels;
//          cv::split(windowrgb, channels);
//          VectorXd compressed_feature = feat_calc.evaluate(channels);
          vector<Point> hits;
          vector<double> w;
          Mat window;
          cvtColor(windowrgb, window, CV_RGB2GRAY);
          people_detector->detect(window, hits, w, weight_is_not_person);
          // It should be 1 or 0 hits
#ifdef OBS_BETA
          if(hits.size() == 0) {
            classes.push_back(-1);
          }
          else if(w[0] > weight_is_person) {
#else
          if(hits.size() > 0) {
#endif
            // Variables to hold the result
            DaTracker::state mean_birth = DaTracker::state::Zero();
            mean_birth.block<2,1>(0,0) = point.block<2,1>(0,0);
            DaTracker::state_mat cov_birth = sigma_v_static * DaTracker::state_mat::Identity();
            cov_birth.block<2,2>(0,0) = cov_centroids[i].block<2,2>(0,0);
            // Add the birth posibility
            birth_weights.push_back(birth_prior_weight / (1.0 + exp(prob_a * w[0] + prob_b)));
            birth_means.push_back(mean_birth);
            birth_covs.push_back(cov_birth);
#ifdef OBS_BETA
            classes.push_back(1);
#endif
          }
          else {
#ifdef OBS_BETA
            classes.push_back(0);
#endif
          }
        }
        catch(...) {
        }
      }
#endif
    }
    // Do the filtering
    const double dt = 3.33E-2;
    // Predict evolution of the system
    Matrix<double,4,2> w;
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
#ifdef OBS_BETA
    d_tracker->correct(meas, classes);
#else
    d_tracker->correct(meas);
#endif
    // Prune
    d_tracker->prune();

    // Get the estimated state
    PointCloud<icl::PointWithVelocity>::Ptr publish_pc(new PointCloud<icl::PointWithVelocity>);
    d_tracker->get_state(publish_pc, false, false);

#ifndef NOROS
    publish_pc->header.frame_id = "base";
    publish_pc->header.stamp = now;

    // Fill with the estimated state
    visualization_msgs::Marker marker_est;
    marker_est.header = publish_pc->header;
    marker_est.ns = "estimations";
    marker_est.id = 0;
    marker_est.type = visualization_msgs::Marker::POINTS;
    marker_est.action = visualization_msgs::Marker::ADD;
    marker_est.pose.position.x = 0.0;
    marker_est.pose.position.y = 0.0;
    marker_est.pose.position.z = 0.0;
    marker_est.pose.orientation.w = 1.0;
    marker_est.scale.x = 0.1;
    marker_est.scale.y = 0.1;
//    marker_est.color.r = 0.0;
//    marker_est.color.g = 0.0;
//    marker_est.color.b = 0.0;
    marker_est.color.a = 0.8;
    marker_est.lifetime = ros::Duration();
    for(size_t i = 0; i < d_tracker->components_t.size(); i++) {
      geometry_msgs::Point p;
      p.x = d_tracker->components_t[i].mean(0);
      p.y = d_tracker->components_t[i].mean(1);
      p.z = -1.2;
      marker_est.points.push_back(p);
      std_msgs::ColorRGBA color;
      color.a = 0.8;
      double prob = d_tracker->components_t[i].a / (d_tracker->components_t[i].a + d_tracker->components_t[i].b);
      int c = prob * 5.0;
      float r = (prob * 5.0) - c;
      switch(c) {
      case 0:
        color.r = 1.0;
        color.g = 1.0-r;
        color.b = 1.0-r;
        break;
      case 1:
        color.r = 1.0;
        color.g = r;
        color.b = 0.0;
        break;
      case 2:
        color.r = 1.0 - r;
        color.g = 1.0;
        color.b = 0.0;
        break;
      case 3:
        color.r = 0.0;
        color.g = 1.0;
        color.b = r;
        break;
      case 4:
        color.r = 0.0;
        color.g = 1.0-r;
        color.b = 1.0;
        break;
      case 5:
        color.r = 0.0;
        color.g = 0.0;
        color.b = r;
        break;
      default:
        color.r = 0.0;
        color.g = 0.0;
        color.b = 0.0;
        ROS_WARN_STREAM("Should not reach here. a: " << d_tracker->components_t[i].a << " b: " << d_tracker->components_t[i].b );
      }
      marker_est.colors.push_back(color);
    }
    mrks.markers.push_back(marker_est);

    // Fill with measurements
#define SHOW meas
    for(size_t i = 0; i < std::max(SHOW.size(), previous_published_meas); i++) {
			visualization_msgs::Marker marker;
			marker.header = publish_pc->header;
			marker.ns = "d_tracker_measurement";
			marker.id = i;
			marker.type = visualization_msgs::Marker::CUBE;
      if(i < SHOW.size()) {
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = SHOW[i](0);
				marker.pose.position.y = SHOW[i](1);
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
    previous_published_meas = SHOW.size();

    fill_visualisation_msg(publish_pc, previous_published, mrks);
    previous_published = publish_pc->size();
#endif
    PointCloud<icl::PointWithVelocity>::iterator it = publish_pc->begin();
    bool first = true;
    while(it != publish_pc->end()) {
      if(first)
        cout << "- ";
      else
        cout << "  ";
      first = false;
      cout << "- [" << it->id << ", " << it->x << ", " << it->y << ", " << it->velocity_x << ", " << it->velocity_y << "]" << endl;
      it++;
    }
    if(first)
      cout << "- false" << endl;
  }

  DTrackerNode(int argc, char **argv): previous_published(0), previous_published_meas(0) {
#ifdef OBS_BETA
    d_tracker.reset(new DaTracker(true, false, false));
#else
    d_tracker.reset(new DaTracker(false, false, false));
#endif
    d_tracker->clear_birth_prior();

    // Use range-bearing or cartesian observations
    d_tracker->sigma_h = 0.01;

    // Basic process model
    sigma_v_static = 0.25;
    sigma_a_dynamic = 1.0;
    d_tracker->F = DaTracker::state_mat::Identity();
    d_tracker->T = sigma_v_static * DaTracker::state_mat::Identity();
    // Basic process observation
    d_tracker->H = (DaTracker::measurement_state_mat() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished();
    d_tracker->R = (0.05*0.05)*DaTracker::measurement_mat::Identity();

    // Parameters of the PHD filter
    d_tracker->p_survival = 0.99;
    d_tracker->p_detect = 0.90;
    d_tracker->clutter_rfs = 0.001;

    // Set the parameters
    d_tracker->truncate_threshold = 1E-10;
    d_tracker->merge_gauss_threshold = 0.05;
    d_tracker->max_components = 200.0;
#ifdef OBS_BETA
    d_tracker->certainty_person_extraction = 0.8;
    d_tracker->certainty_person_threshold = 0.3;
#endif
    birth_prior_weight = 1E-6;

    // Comfort distance

    comfort_distance_sq = 0.45 * 0.45;
    // Initialise the pointcloud filter
    floor_filter.reset(new icl::PlaneFilter<PointXYZRGB>(0.999913581903804, -0.0125094876565810, -0.00404245505527142, 1.05, 0.0, true));
    floor_hessian =  (Vector4d() << 0.999913581903804, -0.0125094876565810, -0.00404245505527142, 1.05).finished();
    ceiling_filter.reset(new icl::PlaneFilter<PointXYZRGB>(0.999913581903804, -0.0125094876565810, -0.00404245505527142, -0.95, 0.0, false));

    P << 5.2921508098293293e+02,                    0.0, 3.2894272028759258e+02, 0.0,
                            0.0, 5.2556393630057437e+02, 2.6748068171871557e+02, 0.0,
                            0.0,                    0.0,                    1.0, 0.0;

#ifdef DEPTH_DETECTOR
    std::string svm_filename("data/trainedLinearSVMForPeopleDetectionWithHOG.yaml");
    person_classifier.loadSVMFromFile(svm_filename);
    people_detector.setVoxelSize(0.06);
    people_detector.setIntrinsics(P.block<3,3>(0,0).cast<float>());
    people_detector.setClassifier(person_classifier);
    people_detector.setHeightLimits(1.3, 2.3);
    people_detector.setSensorPortraitOrientation(true);
    // I don't know how to set these values in this case :(
    prob_a = -1.0;
    prob_b = -2.0;
#else
    // Set up the people detector
		people_detector.reset(new HOGDescriptor());
//    people_detector->load("/data/ros/people_detect_and_tracking/mp_tracker/data/hogdetector.yaml");
//    // Parameters to convert SVM output to probability measurement
//    prob_a = -3.09761222;
//    prob_b = -0.5520975;
    // Daimler people detector works the best
    people_detector->winSize = Size(48, 96);
		people_detector->setSVMDetector(HOGDescriptor::getDaimlerPeopleDetector());
    // Parameters to convert SVM output to probability measurement
    prob_a = -0.95394079;
    prob_b = -1.18738746;
#endif

    // Calculate the weight threshold to get 50% certainty of a person
    double p_star = 0.1, p_star2 = 0.5;
    
    // Now thet everything has defaut values, parser the command line arguments
    po::options_description desc("Allowed Options");
    desc.add_options()
      ("help", "Help message")
      ("prob-survival", po::value<double>(&d_tracker->p_survival), "Probability of survival of the PHD filter")
      ("prob-detection", po::value<double>(&d_tracker->p_detect), "Probability of detection of the PHD filter")
      ("clutter", po::value<double>(&d_tracker->clutter_rfs), "Clutter of the PHD filter")
      ("truncate-threshold", po::value<double>(&d_tracker->truncate_threshold), "Truncate threshold of the prunning process")
      ("merge-gaussians-treshold", po::value<double>(&d_tracker->merge_gauss_threshold), "Merging threshold for gaussian components")
      ("merge-beta-treshold", po::value<double>(&d_tracker->merge_beta_threshold), "Merging threshold for beta components")
      ("max-components", po::value<double>(&d_tracker->max_components), "Max components of the prunning process")
#ifdef OBS_BETA
      ("certainty-person-threshold", po::value<double>(&d_tracker->certainty_person_threshold), "Lower probability threshold for beta components in the pruning process")
      ("is-person-probability", po::value<double>(&d_tracker->is_person_probability), "Threshold probability of being a person")
      ("certainty-person-extraction", po::value<double>(&d_tracker->certainty_person_extraction), "Threshold for P(is_person_probability)")
#endif
      ("birth-prior-weight", po::value<double>(&birth_prior_weight), "Weight to be assigned to the birth prior componets")
      ("comfort-distance-sq", po::value<double>(&comfort_distance_sq), "Square of the comfort distance between people")
      ("lower-probability-person-detection", po::value<double>(&p_star), "Lower bound of a detection being a person")
      ("upper-probability-person-detection", po::value<double>(&p_star2), "Complete certainty of a detection being a person")
    ;
    // Parse command line optins
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    weight_is_not_person = (log(1.0 / p_star - 1.0) - prob_b) / prob_a;
    weight_is_person = (log(1.0 / p_star2 - 1.0) - prob_b) / prob_a;
    // Should never be positive, but just in case
    if(prob_a > 0) {
      weight_is_not_person *= -1;
      weight_is_person *= -1;
    }

    // Print header of the yaml file with description of the settings
    cout << "# People tracking using the following parameters:" << endl;
    cout << "# prob-survival: " << d_tracker->p_survival << endl;
    cout << "# prob-detection: " << d_tracker->p_detect << endl;
    cout << "# clutter: " << d_tracker->clutter_rfs << endl;
    cout << "# truncate-threshold: " << d_tracker->truncate_threshold << endl;
    cout << "# merge-gaussians-treshold: " << d_tracker->merge_gauss_threshold << endl;
    cout << "# merge-beta-treshold: " << d_tracker->merge_beta_threshold << endl;
    cout << "# max-components: " << d_tracker->max_components << endl;
#ifdef OBS_BETA
    cout << "# is-person-probability: " << d_tracker->is_person_probability << endl;
    cout << "# certainty-person-threshold: " << d_tracker->certainty_person_threshold << endl;
    cout << "# certainty-person-extraction: " << d_tracker->certainty_person_extraction << endl;
#endif
    cout << "# birth-prior-weight: " << birth_prior_weight << endl;
    cout << "# comfort-distance-sq: " << comfort_distance_sq << endl;
    cout << "# lower-probability-person-detection: " << p_star << endl;
    cout << "# upper-probability-person-detection: " << p_star2 << endl;
  }
};

int main(int argc, char** argv) {
  previous_published_tracks = 0;
#ifndef NOROS
  ros::init(argc, argv, "density_tracker");
  ros::NodeHandle main_nh;

  ros::Publisher pc_pub1 = main_nh.advertise<PointCloud<PointXYZRGB> >("points1", 10);
  ros::Publisher pc_pub2 = main_nh.advertise<PointCloud<PointXYZRGB> >("points2", 10);
  ros::Publisher pc_pub3 = main_nh.advertise<PointCloud<PointXYZRGB> >("points3", 10);
  ros::Publisher vis_pub = main_nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",10);
#endif
  // The main node
  DTrackerNode node(argc, argv);
  const size_t num_imgs = 1133;
  const size_t num_tracks = 34;

  float angle1 = -43.0  * M_PI / 180;
  float angle2 = 47  * M_PI / 180;
  Matrix4d Tb = (Matrix4d() << 1.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 1.0, 0.0,
                               0.0,-1.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 1.0).finished();
  Matrix4d T1 = Tb * (Translation3d(0.0, 0.0, 0.08) * AngleAxisd(angle1, Vector3d(0.0, 1.0, 0.0)) * AngleAxisd(-M_PI / 2.0, Vector3d(0.0, 0.0, 1.0))).matrix();
  Matrix4d T2 = Tb * (Translation3d(0.0, 0.0, 0.08) * AngleAxisd(-M_PI / 2.0, Vector3d(0.0, 0.0, 1.0))).matrix();
  Matrix4d T3 = Tb * (Translation3d(0.0, 0.0, 0.08) * AngleAxisd(angle2, Vector3d(0.0, 1.0, 0.0)) * AngleAxisd(-M_PI / 2.0, Vector3d(0.0, 0.0, 1.0))).matrix();

#if 1
  // Read the ground truth
  vector<map<int, boost::tuple<int, int, int, int, int> > > tracks;
  for(size_t i = 0; i < num_tracks; i++) {
    stringstream ss;
    ss << "/data/bagfiles/rgbd_people/mensa_seq0_1.1/track_annotations/Track_" << setw(2) << setfill('0') << (i+1) << ".txt";
    fstream file(ss.str().c_str(), ios::in);
    map<int, boost::tuple<int, int, int, int, int> > track;
//    file.ignore(60,'\n');
    while(file.good()) {
      try {
        string seq_file;
        double tt, x_depth, y_depth, width_depth, height_depth, x_rgb, y_rgb, width_rbg, height_rgb;
        int vis;
        file >> seq_file;
        if(seq_file.empty() || seq_file[0] == '#') {
          file.ignore(100000, '\n');
          continue;
        }
        file >> tt >> x_depth >> y_depth >> width_depth >> height_depth >> x_rgb >> y_rgb >> width_rbg >> height_rgb >> vis;
        if(vis == 0)
          continue;
        file.ignore(1, '\n');
        file.peek();
        int seq = atoi(seq_file.substr(5,4).c_str());
        int cam = atoi(seq_file.substr(10,1).c_str());
        boost::tuple<int,int,int,int,int> data(cam, (int)x_depth, (int)y_depth, (int)width_depth, (int)height_depth);
        track[seq] = data;
        tracks.push_back(track);
      } catch (...) {
        cout << "Problem processing file " << ss.str() << endl;
      }
    }
  }
#endif

  map<int, map<int, pair<float, float> > > final_tracks;
  for(size_t i = 0; i < num_imgs; ++i) {
#ifndef NOROS
    now = ros::Time::now();
#endif
    stringstream ss1, ss2, ss3;
    // Read the depth images
    ss1 << "/data/bagfiles/rgbd_people/mensa_seq0_1.1/depth/seq0_"  << setw(4) << setfill('0') << i << "_0.pgm";
    Mat depth_img1 = imread(ss1.str(), CV_LOAD_IMAGE_ANYDEPTH);
    ss2 << "/data/bagfiles/rgbd_people/mensa_seq0_1.1/depth/seq0_"  << setw(4) << setfill('0') << i << "_1.pgm";
    Mat depth_img2 = imread(ss2.str(), CV_LOAD_IMAGE_ANYDEPTH);
    ss3 << "/data/bagfiles/rgbd_people/mensa_seq0_1.1/depth/seq0_"  << setw(4) << setfill('0') << i << "_2.pgm";
    Mat depth_img3 = imread(ss3.str(), CV_LOAD_IMAGE_ANYDEPTH);

    // Read the RGB images
    stringstream ssrgb1, ssrgb2, ssrgb3;
    ssrgb1 << "/data/bagfiles/rgbd_people/mensa_seq0_1.1/rgb/seq0_"  << setw(4) << setfill('0') << i << "_0.ppm";
    Mat rgb_img1 = imread(ssrgb1.str(), CV_LOAD_IMAGE_COLOR);
    ssrgb2 << "/data/bagfiles/rgbd_people/mensa_seq0_1.1/rgb/seq0_"  << setw(4) << setfill('0') << i << "_1.ppm";
    Mat rgb_img2 = imread(ssrgb2.str(), CV_LOAD_IMAGE_COLOR);
    ssrgb3 << "/data/bagfiles/rgbd_people/mensa_seq0_1.1/rgb/seq0_"  << setw(4) << setfill('0') << i << "_2.ppm";
    Mat rgb_img3 = imread(ssrgb3.str(), CV_LOAD_IMAGE_COLOR);

    PointCloud<PointXYZRGB>::Ptr pc1(new PointCloud<PointXYZRGB>), pc2(new PointCloud<PointXYZRGB>), pc3(new PointCloud<PointXYZRGB>);
    icl::createPointcloud(depth_img1, rgb_img1, Matrix4f::Identity(), pc1);
    icl::createPointcloud(depth_img2, rgb_img2, Matrix4f::Identity(), pc2);
    icl::createPointcloud(depth_img3, rgb_img3, Matrix4f::Identity(), pc3);

//    smoothPointCloudImage(pc1, Matrix4f::Identity(), 7, 0.2, LINEAR_FIT);
//    smoothPointCloudImage(pc2, Matrix4f::Identity(), 7, 0.2, LINEAR_FIT);
//    smoothPointCloudImage(pc3, Matrix4f::Identity(), 7, 0.2, LINEAR_FIT);

    // Run the filter
    node.sensors_cb(rgb_img1, pc1, T1, rgb_img2, pc2, T2, rgb_img3, pc3, T3);
#if 1
    vector<int> valid_tracks;
    // Get the real position
    for(size_t j = 0; j < num_tracks; j++) {
      map<int, boost::tuple<int, int, int, int, int> > track = tracks[j];
      map<int, boost::tuple<int, int, int, int, int> >::iterator it = track.find(i);
      if(it != track.end()) {
        map<int, pair<float, float> > final_track = final_tracks[j];
        boost::tuple<int, int, int,int,int> data = it->second;
        int cam = data.get<0>();
        int x = data.get<1>();
        int w = data.get<3>();
        int y = data.get<2>();
        int h = data.get<4>();
        const int window = 15;
        Matrix4f T = (cam == 0 ? T1.cast<float>() : ( cam == 1 ? T2.cast<float>() : T3.cast<float>()));
        PointCloud<PointXYZRGB>::Ptr pct = (cam == 0 ? pc1 : ( cam == 1 ? pc2 : pc3));
        Mat img = (cam == 0 ? depth_img1 : ( cam == 1 ? depth_img2 : depth_img3));
        Vector4f final = Vector4f::Zero();
        int sign = 1;
        for(int l = 0; l < 1; l++) {
          int valid = 0;
          int offset = 2*sign * ((l + 1) / 2) * window;
          for(int u = -window; u < window; ++u) {
            for(int v = -window; v < window; ++v) {
              int up = x + w/2 + u;
              int vp = y + h/2 + v + offset;
              if(up >= 0 && vp >= 0) {
#ifdef PCL17
                if(isFinite(pct->at(vp, up))) {
#else
                if(hasValidXYZ(pct->at(vp, up))) {
#endif
                  final += T * (Vector4f() << pct->at(vp, up).getVector3fMap(), 1.0).finished();
                  valid ++;
                }
              }
            }
          }
          final /= valid;
          if(valid > 0) {
            final_track[i] = make_pair(final(0), final(1));
            final_tracks[j] = final_track;
            valid_tracks.push_back(j);
            break;
          }
          sign *= -1;
        }
      }
    }
/*
    // Create visualization message with valid tracks
    for(size_t j = 0; j < max(previous_published_tracks, valid_tracks.size()); j++) {
			visualization_msgs::Marker marker;
			marker.header.frame_id = "base";
      marker.header.stamp = now;
			marker.ns = "tracks";
			marker.id = j;
			marker.type = visualization_msgs::Marker::LINE_STRIP;
      if(j < valid_tracks.size()) {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        marker.lifetime = ros::Duration();
        map<int, pair<float, float> >::iterator it = final_tracks[valid_tracks[j]].begin();
        for(; it != final_tracks[valid_tracks[j]].end(); it++) {
          geometry_msgs::Point p;
          p.x = it->second.first;
          p.y = it->second.second;
          p.z = -1.2;
          marker.points.push_back(p);
        }
      } else {
				marker.action = visualization_msgs::Marker::DELETE;
      }
			node.mrks.markers.push_back(marker);

    }
    previous_published_tracks = valid_tracks.size();
*/
#ifndef NOROS
    transformPointCloud(*pc1, *pc1, T1.cast<float>());
    transformPointCloud(*pc2, *pc2, T2.cast<float>());
    transformPointCloud(*pc3, *pc3, T3.cast<float>());
    pc1->header.frame_id = "base";
    pc1->header.stamp = now;
    pc2->header = pc1->header;
    pc3->header = pc1->header;
    pc_pub1.publish(pc1);
    pc_pub2.publish(pc2);
    pc_pub3.publish(pc3);
    vis_pub.publish(node.mrks);
    // Spin ROS
    ros::spinOnce();
#endif
#endif
  }
#if 0
  // Print all the tracks
  for(size_t j = 0; j < num_tracks; j++) {
    cout << "# Track " << j << endl;
    map<int, pair<float, float> >::iterator it = final_tracks[j].begin();
    VectorXd t(final_tracks[j].size()),
             x(final_tracks[j].size()),
             y(final_tracks[j].size());

    size_t i = 0;
    for(; it != final_tracks[j].end(); it++) {
      t(i) = it->first;
      x(i) = it->second.first;
      y(i) = it->second.second;
    }
    it = final_tracks[j].begin();
    for(; it != final_tracks[j].end(); it++) {
      if(it != final_tracks[j].begin())
        cout << "  ";
      else
        cout << "- ";
      cout << "- [" << it->first << ", " << it->second.first << ", " << it->second.second << "]" << endl;
    }
  }
#endif
  return 0;
}


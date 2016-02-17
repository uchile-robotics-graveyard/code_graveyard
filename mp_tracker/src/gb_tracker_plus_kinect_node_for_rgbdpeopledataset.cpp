#include "gb_tracker.h"
#include "util.h"
#include "pointcloud_process.h"
#include "compressed_features.h"
#ifndef NOROS
#include <pcl_ros/point_cloud.h>
#include <icl_robot/plane_filter.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#endif
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#ifdef NOROS
#include "plane_filter.h"
#endif
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <util.h>
#include "depth_connected_components.h"
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
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

icl::GBTracker<4,2>::tracker_component creation(const icl::GBTracker<4,2>::measurement &obs,
                                                int classification,
                                                const icl::GBTracker<4,2>::colour_projection_type &colour) {
  typedef typename icl::GBTracker<4,2>::colour_projection_type colour_projection_type;
  icl::GBTracker<4,2>::tracker_component tracker;
  // Create the Gaussian component
  tracker.mean = icl::GBTracker<4,2>::state::Zero();
  tracker.mean.block<2,1>(0,0) = obs;
  tracker.cov = (0.05*0.05)*icl::GBTracker<4,2>::state_mat::Identity();
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

// Helpfull to avoid chainging all the apearances all the time
typedef icl::GBTracker<4,2> DaTracker;

struct GBTrackerNode {
  DaTracker::ptr d_tracker;
#ifdef DEPTH_DETECTOR
  people::PersonClassifier<pcl::RGB> person_classifier;
  people::GroundBasedPeopleDetectionApp<PointXYZRGBA> people_detector;
#else
	boost::shared_ptr<HOGDescriptor> people_detector;
#endif
  boost::shared_ptr<icl::PlaneFilter<PointXYZRGBA> > floor_filter;
  Vector4d floor_hessian;
  boost::shared_ptr<icl::PlaneFilter<PointXYZRGBA> > ceiling_filter;

  vector<size_t> downsampler;
  double sigma_a_dynamic, sigma_v_static, sigma_obs;
  double comfort_distance_sq;
  double birth_prior_weight;
  double prob_a, prob_b, weight_is_not_person;

  double f_val_to_prob(double f) {
    return 1.0 / (1.0 + std::exp(prob_a*f + prob_b));
  }

  // For visualization purposes
#ifndef NOROS
  visualization_msgs::MarkerArray mrks;
#endif
  size_t previous_published, previous_published_meas;

  CompressedFeatures feat_calc;

  vector<double> plane_intercept;

  Matrix<double,3,4> P;

  void sensors_cb(Mat &img1,
                  const PointCloud<PointXYZRGBA>::Ptr &pc1,
                  const Matrix4d &T1,
                  Mat &img2,
                  const PointCloud<PointXYZRGBA>::Ptr &pc2,
                  const Matrix4d &T2, 
                  Mat &img3,
                  const PointCloud<PointXYZRGBA>::Ptr &pc3,
                  const Matrix4d &T3) {
#ifndef NOROS
    mrks.markers.resize(0);
#endif
    DaTracker::measurements meas;
    DaTracker::colour_projections_type colour_meas;
    std::vector<double> f_values;
    // Process all pointclouds
    for(size_t n = 0; n < 3; n++) {
      PointCloud<PointXYZRGBA>::Ptr pc = (n == 0 ? pc1: (n == 1 ? pc2 : pc3));
      const Matrix4d &T = (n == 0 ? T1: (n == 1 ? T2 : T3));
      Mat &imagergb = (n == 0 ? img1: (n == 1 ? img2 : img3));
#ifdef DEPTH_DETECTOR
      // Perform people detection on the new cloud:
      std::vector<people::PersonCluster<PointXYZRGBA> > clusters;
      people_detector.setInputCloud(pc);
      VectorXf floor_params(4);
      floor_params = floor_hessian.cast<float>();
      people_detector.setGround(floor_params);
      people_detector.compute(clusters);

      for(std::vector<pcl::people::PersonCluster<PointXYZRGBA> >::iterator it = clusters.begin(); it != clusters.end(); ++it) {
        Vector4d point = T * (Vector4d() << it->getCenter().cast<double>(), 1.0).finished();
        if(d_tracker->use_beta || it->getPersonConfidence() > weight_is_not_person) {
          meas.push_back(point.block<2,1>(0,0));
          f_values.push_back(f_val_to_prob(it->getPersonConfidence()));
        }
      }
#else
      // Preprocess the pointcloud: Remove the floor
      floor_filter->setInputCloud(pc);
      floor_filter->filter(*pc);
      // Preprocess the pointcloud: Remove the ceiling
#ifndef PEOPLE_HEAD_SUBCLUSTER
      ceiling_filter->setInputCloud(pc);
      ceiling_filter->filter(*pc);
#endif
      pc->is_dense = false;

      // Get the clusters as measurements
      vector<PointIndices> clusters;
      vector<Vector3d, aligned_allocator<Vector3d> > mean_centroids;
      vector<Matrix3d, aligned_allocator<Matrix3d> > cov_centroids;
      icl::cluster_pointcloud_special(pc, 0.14, 240, 40000, comfort_distance_sq, clusters, mean_centroids, cov_centroids, floor_hessian);

      // Extract the ROIs in the image and generate measurements
      for(size_t i = 0; i < clusters.size(); i++) {
        // A person shouldn't have the x components outside (0.1, 0.5)
        // If the mean position is too up (kinect y points down), discard this measurement
        Vector4d point = T * (Vector4d() << mean_centroids[i], 1.0).finished();
        if(point(2) <= -0.8 || point(2) >= 0.3) {
          continue;
        }
        // At this point this is condition enough to be a measurement
        if(d_tracker->use_beta)
          meas.push_back(point.block<2,1>(0,0));

        // Start creating the birth process
        vector<Point> points;
        // Project all the points into the image space
        for(size_t k = 0; k < clusters[i].indices.size(); k++) {
          // Backproject some points
          const PointXYZRGBA &p = pc->points[clusters[i].indices[k]];
          // Assumes the depth sensors is on the same frame of reference as the colour camera
          Vector3d floor_point = p.getVector3fMap().cast<double>();
          Vector3d pixel = P * (Vector4d() << point, 1.0).finished();
          pixel /= pixel(2);
          points.push_back(Point(pixel(0), pixel(1)));
          // Project the pixel into the floor
          Vector3d point_floor = floor_point - (floor_point.dot(floor_hessian.block<3,1>(0,0)) + floor_hessian(3)) * floor_hessian.block<3,1>(0,0);
          Vector3d pixel_floor = P * (Vector4d() << point_floor, 1.0).finished();
          pixel_floor /= pixel_floor(2);
          points.push_back(Point(pixel_floor(0), pixel_floor(1)));
        }
        // Get the bouding box of the points
        Rect roi = boundingRect(points);
        roi += Point(-32, -32);
        roi += Size(64, 64);
        roi &= Rect(0, 0, imagergb.cols, imagergb.rows);
        // Check for wrong intersections
        if(roi.width == 0 || roi.height == 0) {
          typedef typename icl::GBTracker<4,2>::colour_projection_type colour_projection_type;
          cerr << "Problem with ROI" << endl;
          f_values.push_back(0);
          colour_meas.push_back(colour_projection_type::Zero(d_tracker->num_colour_components));
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
          remap(subimg1rgb, subimg2rgb, map_x, map_y, CV_INTER_NN, BORDER_CONSTANT, Scalar(0, 0, 0));
          Mat windowrgb;
          resize(subimg2rgb, windowrgb, people_detector->winSize);

          vector<Mat> channels;
          cv::split(windowrgb, channels);
          VectorXd compressed_feature = feat_calc.evaluate(channels);
          colour_meas.push_back(compressed_feature.array());
          vector<Point> hits;
          vector<double> w;
          Mat window;
          cvtColor(windowrgb, window, CV_RGB2GRAY);
#ifndef LOGISTIC_MODEL
          people_detector->detect(window, hits, w, weight_is_not_person);
          // It should be 1 or 0 hits
          if(hits.size() == 0) {
            f_values.push_back(0);
          }
          else {
            if(!d_tracker->use_beta)
              meas.push_back(point.block<2,1>(0,0));
            f_values.push_back(f_val_to_prob(w[0]));
          }
#else
          vector<float> desc_;
          people_detector->compute(window, desc_);
          assert(desc_.size() == plane_intercept.size() - 1);
          double f = 0.0;
          for(size_t i = 0; i < plane_intercept.size() - 1; i++) {
            f += plane_intercept[i] * desc_[i];
          }
          f += plane_intercept[plane_intercept.size()-1];
          f_values.push_back(f_val_to_prob(f));
          if(!d_tracker->use_beta && f > weight_is_not_person)
            meas.push_back(point.block<2,1>(0,0));
#endif
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
    d_tracker->F(0, 2) = dt;
    d_tracker->F(1, 3) = dt;
    d_tracker->predict();
    // Correct with the measurements
    d_tracker->correct(meas, f_values, colour_meas);

    // Get the estimated state
    PointCloud<icl::PointWithVelocity>::Ptr publish_pc(new PointCloud<icl::PointWithVelocity>);
    d_tracker->get_state(publish_pc);

#if 0
    if(publish_pc->size() > 0) {
/*
      // Create a cv::Mat from the pointcloud
      Mat points(Size(6, publish_pc->size()), CV_32FC1, &publish_pc->points[0], sizeof(icl::PointWithVelocity));
      // The matrix corresponding to the position only
      Mat pos = points.colRange(0, 2);
      // The matrix corresponding to the velocity only
      Mat vel = points.colRange(4, 6);
      // The matrix with position and velocity
      Mat data;
      hconcat(pos, vel, data);

      // kmean clustering
      int k = 3;
      kmeans(data, k, labels, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1.0), 5, KMEANS_RANDOM_CENTERS, centres);
*/
/*
      // Label expansion
      vector<int> labels(publish_pc->size());
      fill(labels.begin(), labels.end(), -1);
      int new_label = 0;
      for(size_t i = 0; i < publish_pc->size(); i++) {
        Vector4f pi = (Vector4f() << publish_pc->points[i].x, publish_pc->points[i].y, publish_pc->points[i].velocity_x, publish_pc->points[i].velocity_y).finished();
        // Mahalanobis distance, the matrix penalise objects in the moving direction of the person
        Matrix4f U = Matrix4f::Identity();
        U.block<2,1>(0,0) = pi.block<2,1>(2,0).normalized();
        U(0,1) = -U(1,0);
        U(1,1) = U(0,0);
        Matrix4f V = Matrix4f::Identity();
        V(0,0) = 1.0 / (1.0 + pi.block<2,1>(2,0).norm());
        Matrix4f M = U * V * U.transpose();
        if(labels[i] == -1)
          labels[i] = new_label++;
        // Calculate the distance with all other objects
        for(size_t j = i+1; j < publish_pc->size(); j++) {
          Vector4f pj = (Vector4f() << publish_pc->points[j].x, publish_pc->points[j].y, publish_pc->points[j].velocity_x, publish_pc->points[j].velocity_y).finished();
          Vector4f dp = pi - pj;
          // Solve to avoid inversion
          float dist = dp.transpose() * M.ldlt().solve(dp);
          if(dist < 1.0) {
            labels[j] = labels[i];
          }
        }
      }
*/
    }
#endif
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
    if(!d_tracker->use_beta) {
      marker_est.color.r = 0.0;
      marker_est.color.g = 0.0;
      marker_est.color.b = 0.0;
    }

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
      if(d_tracker->use_beta) {
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

  GBTrackerNode(int argc, char **argv): previous_published(0), previous_published_meas(0), feat_calc(50) {
    d_tracker.reset(new DaTracker());
#ifdef DEPTH_DETECTOR
    d_tracker->use_colour = false;
#endif
    d_tracker->state_creator = &creation;
    d_tracker->num_colour_components = 50;

    // Use range-bearing or cartesian observations
    d_tracker->sigma_h = 0.01;

    // Basic process model
    sigma_v_static = 0.25;
    sigma_a_dynamic = 1.0;
    d_tracker->F = DaTracker::state_mat::Identity();
    d_tracker->colour_std_change = 1.0;
    // Basic process observation
    d_tracker->H = (DaTracker::measurement_state_mat() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished();
    d_tracker->R = (0.05*0.05)*DaTracker::measurement_mat::Identity();
    d_tracker->colour_observation_std = 2.0;

    // Comfort distance
    comfort_distance_sq = 0.5 * 0.5;
    // Initialise the pointcloud filter
    floor_filter.reset(new icl::PlaneFilter<PointXYZRGBA>(0.999913581903804, -0.0125094876565810, -0.00404245505527142, 1.05, 0.0, true));
    floor_hessian =  (Vector4d() << 0.999913581903804, -0.0125094876565810, -0.00404245505527142, 1.05).finished();

    ceiling_filter.reset(new icl::PlaneFilter<PointXYZRGBA>(0.999913581903804, -0.0125094876565810, -0.00404245505527142, -1.00, 0.0, false));

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

//    // Daimler people detector works the best
    people_detector->winSize = Size(48, 96);
		people_detector->setSVMDetector(HOGDescriptor::getDaimlerPeopleDetector());
    // Parameters to convert SVM output to probability measurement
#ifndef LOGISTIC_MODEL 
    prob_a = -0.95394079;
    prob_b = -1.18738746;
#else
    // Detector from Raw HOG descriptor trained with the logistic regression
    prob_a = -1.0;
    prob_b = 0.0;
#endif
    // Precalculate the random projection
    feat_calc.generate_features(people_detector->winSize.width, people_detector->winSize.height);
#endif

    // Now thet everything has defaut values, parser the command line arguments
    po::options_description desc("Allowed Options");
    int use_beta = 0;
    desc.add_options()
      ("help", "Help message")
      ("num-frames-missdetect", po::value<size_t>(&d_tracker->num_frames_missdetected), "Number of frames to remove a tracker")
      ("num-frames-create", po::value<size_t>(&d_tracker->min_frames_creation), "Number of frames to create a tracker")
      ("comfort-distance-sq", po::value<double>(&comfort_distance_sq), "Square of the comfort distance between people")
      ("probability-var-increase", po::value<double>(&d_tracker->probability_var_increase), "Increase of the variance of the beta probability")
      ("lower-probability-person-detection", po::value<double>(&d_tracker->is_not_confident_value), "Lower bound of a detection being a person")
      ("upper-probability-person-detection", po::value<double>(&d_tracker->is_confident_value), "Complete certainty of a detection being a person")
      ("is-person-probability", po::value<double>(&d_tracker->is_person_probability), "Probability of being a person")
      ("probability-var-extraction", po::value<double>(&d_tracker->probability_var_extraction), "Confidence on being a person")
      ("colour-observation-std", po::value<double>(&d_tracker->colour_observation_std), "Uncertainty on the projection observation")
      ("colour-std-change", po::value<double>(&d_tracker->colour_std_change), "Increase at each time step")
      ("use-beta", po::value<int>(&use_beta), "Use beta")
    ;
    // Parse command line optins
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    weight_is_not_person = (log(1.0 / d_tracker->is_not_confident_value - 1.0) - prob_b) / prob_a;
    // Should never be positive, but just in case
    if(prob_a > 0) {
      weight_is_not_person *= -1;
    }

    // Set the usage of the beta mixture
    d_tracker->use_beta = (use_beta != 0);

    // Print header of the yaml file with description of the settings
    cout << "# People tracking using the following parameters:" << endl;
    cout << "# comfort-distance-sq: " << comfort_distance_sq << endl;
    cout << "# probability-var-increase: " << d_tracker->probability_var_increase << endl;
    cout << "# lower-probability-person-detection: " << d_tracker->is_not_confident_value << endl;
    cout << "# upper-probability-person-detection: " << d_tracker->is_confident_value << endl;
    cout << "# num-frames-missdetect: " << d_tracker->num_frames_missdetected << endl;
    cout << "# num-frames-create: " << d_tracker->min_frames_creation << endl;
    cout << "# is-person-probability: " << d_tracker->is_person_probability << endl;
    cout << "# probability-var-extraction: " << d_tracker->probability_var_extraction << endl;
    cout << "# colour-observation-std: " << d_tracker->colour_observation_std << endl;
    cout << "# colour-std-change: " << d_tracker->colour_std_change << endl;
    cout << "# use-beta: " << use_beta << endl;

    // Read the definition of plane
    ifstream plane_file("data/logistic_model.yaml");
    YAML::Parser parser(plane_file);
    YAML::Node doc;
    parser.GetNextDocument(doc);
    plane_intercept.resize(doc[0].size()+1);
    for(size_t i = 0; i < doc[0].size(); i++) {
       doc[0][i] >> plane_intercept[i];
    }
    doc[1] >> plane_intercept[doc[0].size()];
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char** argv) {
  previous_published_tracks = 0;

#ifndef NOROS
  ros::init(argc, argv, "density_tracker");
  ros::NodeHandle main_nh;

  ros::Publisher pc_pub1 = main_nh.advertise<PointCloud<PointXYZRGBA> >("points1", 10);
  ros::Publisher pc_pub2 = main_nh.advertise<PointCloud<PointXYZRGBA> >("points2", 10);
  ros::Publisher pc_pub3 = main_nh.advertise<PointCloud<PointXYZRGBA> >("points3", 10);
  ros::Publisher vis_pub = main_nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",10);
#endif
  
  // The main node
  boost::shared_ptr<GBTrackerNode> node(new GBTrackerNode(argc, argv));
  const size_t num_imgs = 1133;
  const size_t num_tracks = 34;

  float angle1 = -43.0  * M_PI / 180.0;
  float angle2 = 47.0  * M_PI / 180.0;
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
      string seq_file;
      double tt, x_depth, y_depth, width_depth, height_depth, x_rgb, y_rgb, width_rbg, height_rgb;
      int vis;
      file >> seq_file >> tt >> x_depth >> y_depth >> width_depth >> height_depth >> x_rgb >> y_rgb >> width_rbg >> height_rgb >> vis;
      if(vis == 0 || seq_file.empty())
        continue;
      file.ignore(1, '\n');
      file.peek();
      int seq = atoi(seq_file.substr(5,4).c_str());
      int cam = atoi(seq_file.substr(10,1).c_str());
      boost::tuple<int,int,int,int,int> data(cam, (int)x_depth, (int)y_depth, (int)width_depth, (int)height_depth);
      track[seq] = data;
    }
    tracks.push_back(track);
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

    PointCloud<PointXYZRGBA>::Ptr pc1(new PointCloud<PointXYZRGBA>), pc2(new PointCloud<PointXYZRGBA>), pc3(new PointCloud<PointXYZRGBA>);
    icl::createPointcloud(depth_img1, rgb_img1, Matrix4f::Identity(), pc1);
    icl::createPointcloud(depth_img2, rgb_img2, Matrix4f::Identity(), pc2);
    icl::createPointcloud(depth_img3, rgb_img3, Matrix4f::Identity(), pc3);

//    icl::simpleConvertVGAtoQVGA(pc1);
//    icl::simpleConvertVGAtoQVGA(pc2);
//    icl::simpleConvertVGAtoQVGA(pc3);

//    icl::smoothPointCloudImage(pc1, Matrix4f::Identity(), 8, 0.15, icl::QUADRATIC_FIT);
//    icl::smoothPointCloudImage(pc2, Matrix4f::Identity(), 8, 0.15, icl::QUADRATIC_FIT);
//    icl::smoothPointCloudImage(pc3, Matrix4f::Identity(), 8, 0.15, icl::QUADRATIC_FIT);

    // Run the filter
    node->sensors_cb(rgb_img1, pc1, T1, rgb_img2, pc2, T2, rgb_img3, pc3, T3);
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
        PointCloud<PointXYZRGBA>::Ptr pct = (cam == 0 ? pc1 : ( cam == 1 ? pc2 : pc3));
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
#ifndef NOROS
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
			node->mrks.markers.push_back(marker);
    }
    previous_published_tracks = valid_tracks.size();
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
    vis_pub.publish(node->mrks);
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


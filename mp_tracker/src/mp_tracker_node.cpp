#include "mp_tracker.h"
#include "util.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/thread/mutex.hpp>
#include <nav_msgs/GetMap.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <fstream>
#include <cstdlib>
#include <stdexcept>
#include <functional>
#include <unscented_sampler.hpp>
#include <util.h>
#include "pointcloud_process.h"

#include <boost/array.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace sensor_msgs;
struct MPTrackerNode {
  typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, LaserScan> Policy;
  typedef message_filters::Synchronizer<Policy> Synchronizer;
  ros::NodeHandle main_nh;
  ros::NodeHandle priv_nh;
  icl::MPTracker::ptr mp_tracker;
  boost::shared_ptr<message_filters::Subscriber<Image> > img_sub;
  boost::shared_ptr<message_filters::Subscriber<CameraInfo> > cam_sub;
  boost::shared_ptr<message_filters::Subscriber<LaserScan> > laser_sub;
  boost::shared_ptr<Synchronizer> synch;
  image_transport::ImageTransport it;
#ifdef DEBUG
  image_transport::CameraSubscriber dbg_sub;
#endif
  ros::Publisher track_pub;
  ros::Publisher viz_pub;
	boost::shared_ptr<tf::TransformListener> tf_listener;
#ifdef USE_HOG
#ifdef USE_LATENT
	boost::shared_ptr<CvLatentSvmDetector> people_detector;
#else
	boost::shared_ptr<cv::HOGDescriptor> people_detector;
#endif
#else
	boost::shared_ptr<cv::CascadeClassifier> people_detector;
#endif
  double total_expected_birth;
  double detect_threshold;
  double detection_pixel_variance, detection_height_variance, detection_height_border;
  double expected_height, var_height;
  double last_time;
  boost::mutex mtx;
  ros::Timer spin_timer;
  int num_points_published;
  bool omit_empty_measurements;

  int tmpl_match_algorithm;
  int tmpl_roi_size;
  boost::function<bool (const double&, const double&)> tmpl_output_compare;
  double template_match_epsilon;
  double template_dist_epsilon;

  std::string tracking_frame;
  std::string robot_frame;

  bool upscale_image;
  std::vector<std::pair<cv::Rect, cv::Mat> > previous_detections;

  icl::unscented_sampler<6> unsctd_sampler;

  std::vector<Eigen::VectorXd> last_points;
  size_t indx_last_points;

  void sensors_cb(const ImageConstPtr &img, const CameraInfoConstPtr &cam, const LaserScanConstPtr &laser) {
    double time = ros::Time::now().toSec();

    // Get needed trasforms
		Eigen::Matrix4d Tl, Tcl, Tc, Tci, Tr;
    try {
  		icl::lookupTransform(*tf_listener, img->header.frame_id, laser->header.frame_id, Tcl);
  		icl::lookupTransform(*tf_listener, tracking_frame, laser->header.frame_id, Tl);
  		icl::lookupTransform(*tf_listener, tracking_frame, img->header.frame_id, Tc);
  		icl::lookupTransform(*tf_listener, img->header.frame_id, tracking_frame, Tci);
  		icl::lookupTransform(*tf_listener, tracking_frame, robot_frame, Tr);
    } catch(tf::TransformException e) {
      std::cout << "Miss a transformation. " << e.what() << std::endl;
      return;
    }

    // Camera matrix in Eigen form. Eigen::Map is not used as the types are different (ROS uses float, while in this module double is used)
    double f = cam->P[0];
    Eigen::Matrix<double,3,4> P = (Eigen::Matrix<double,3,4>() << cam->P[0], cam->P[1], cam->P[2], cam->P[3],
         cam->P[4], cam->P[5], cam->P[6], cam->P[7],
         cam->P[8], cam->P[9], cam->P[10], cam->P[11]).finished();
    Eigen::MatrixXd K = P * Tci;

    // Get the opencv image from the ROS message
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, enc::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat &image_raw = cv_ptr->image;
    if(upscale_image) {
      cv::Mat tmp_;
      cv::resize(image_raw, tmp_, cv::Size(), 2.0, 2.0, CV_INTER_LANCZOS4);
      image_raw = tmp_;
    }
#ifdef DEBUG
    cv::Mat roi = cv::Mat::zeros(image_raw.size(), CV_8UC1);
#endif
    // First, store which windows are going to be explored
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pc(new pcl::PointCloud<pcl::PointXYZ>);
    icl::pointcloud_from_laser(laser->ranges, laser->angle_min, laser->angle_increment, 30.0, Eigen::Matrix4d::Identity(), laser_pc);
    Eigen::Matrix<double,3,4> tmp_ = (P * Tcl);
    std::vector<cv::Rect> possibilities;
    icl::cluster_roi(laser_pc, tmp_, 0.30, 5, 100, 0.2, false, possibilities);
    for(size_t i = 0; i < possibilities.size(); i++) {
      cv::Rect &r = possibilities.at(i);
      r.x -= 0.75 * r.width;
//      r.y += detection_height_border / 2.0;
      r.width *= 2.5;
//      r.height +=  detection_height_border;
      if(r.x + r.width < 0 || r.x >= cam->width - 1
        || r.y + r.height < 0 || r.y >= cam->height - 1) {
        possibilities.erase(possibilities.begin() + i);
        i--;
      }
      if(r.x < 0)
        r.x = 0.0;
      if(r.y < 0)
        r.y = 0.0;
      r.width = (r.width >= cam->width - r.x ? cam->width - r.x - 1 : r.width);
      r.height = (r.height >= cam->height - r.y ? cam->height - r.y - 1 : r.height);
    }
#ifdef DEBUG
    std::cout << "# Num possibilities: " << possibilities.size() << std::endl;
    for(std::vector<cv::Rect>::iterator i = possibilities.begin(); i != possibilities.end(); i++) {
      roi(*i).setTo(255);
    }
#endif

#ifdef USE_LATENT
    cv::Mat image = image_raw;
#else
    cv::Mat image;
    cv::cvtColor(image_raw, image, CV_RGB2GRAY);
#endif

#ifdef DEBUG
    cv::Mat dbg_img = cv::Mat::zeros(image_raw.size(), CV_8UC3);
    cv::bitwise_and(image_raw, cv::Scalar(255,255,255), dbg_img, roi);
#endif

    // Detect full bodies from the image
#ifdef USE_HOG
    // Run HOG people detector over the predefined windows
    std::vector<cv::Rect> bodies;
    std::vector<double> weights;
    bodies.reserve(possibilities.size());
    weights.reserve(possibilities.size());
    for(size_t i = 0; i < possibilities.size(); i++) {
#ifdef DEBUG
/*
      try {
        if(possibilities[i].height + current_row < image.rows) {
          if(possibilities[i].width + current_col > image.cols) {
            current_row = max_row;
            current_col = 0;
          }
          cv::Rect cropped = possibilities[i];
          cv::Rect sub_region(current_col, current_row, cropped.width, cropped.height);
          if(sub_region.x + sub_region.width > image.cols)
            sub_region.width = image.cols - sub_region.x - 1;
          if(sub_region.y + sub_region.height > image.rows)
            sub_region.height = image.rows - sub_region.y - 1;
          cropped.width = sub_region.width;
          cropped.height = sub_region.height;
          cv::Mat region = dbg_img(sub_region);
          image_raw(cropped).copyTo(region);
          max_row = std::max(max_row, current_row + possibilities[i].height);
          current_col += possibilities[i].width;
        }
      } catch(...) {} // I don't care about errors here, just for debug
*/
#endif
      cv::Mat window;
      // Scale the window to the size of the detector
      std::vector<cv::Point> hits;
#ifdef USE_LATENT
      double fw = 50.0 / (double)possibilities[i].width;
      double fh = 100.0 / (double)possibilities[i].height;
#else
      double fw = (double)(people_detector->winSize.width + people_detector->blockSize.width) / (double)possibilities[i].width;
      double fh = (double)(people_detector->winSize.height + people_detector->blockSize.height) / (double)possibilities[i].height;
#endif
      double scale = std::min(fw, fh);
#ifdef USE_LATENT
      if(scale * possibilities[i].width < 72
         || scale * possibilities[i].height < 120)
#else
      if(scale * possibilities[i].width < people_detector->winSize.width
         || scale * possibilities[i].height < people_detector->winSize.height)
#endif
        scale = std::max(fw, fh);
      cv::Size new_size(std::floor(possibilities[i].width * scale + 0.5),
                        std::floor(possibilities[i].height * scale + 0.5));
      cv::resize(image(possibilities[i]), window, new_size);
      // Detect people
#ifdef USE_LATENT
      CvMemStorage* storage = cvCreateMemStorage(0);
      IplImage win_tmp = (IplImage)window;
      CvSeq* hits_ = cvLatentSvmDetectObjects(&win_tmp, people_detector.get(), storage, detect_threshold);
      for(size_t i = 0; i < hits_->total; i++) {
        CvObjectDetection det = *(CvObjectDetection*)cvGetSeqElem(hits_, i);
        cv::Rect hit = det.rect;
        cv::Rect body;
        body.x = hit.x / scale + possibilities[i].x;
        body.y = hit.y / scale + possibilities[i].y;
        body.width = 50 / scale;
        body.height = 100 / scale;
        bodies.push_back(body);
        weights.push_back(1.0);
      }
      cvReleaseMemStorage(&storage);
#else
      std::vector<double> w;
      people_detector->detect(window, hits, w, detect_threshold);

      // If a detectin is made, save it
      for(size_t j = 0; j < hits.size(); j++) {
        cv::Rect body;
        body.x = hits[j].x / scale + possibilities[i].x;
        body.y = hits[j].y / scale + possibilities[i].y;
        body.width = people_detector->winSize.width / scale;
        body.height = people_detector->winSize.height / scale;
        bodies.push_back(body);
        weights.push_back(w[j]);
      }
#endif
    }
    // Group similar rectangles
    groupRectangles(bodies, 1, 0.2);

    std::vector<icl::measurement_2d_fullbody> meas;
    meas.reserve(bodies.size());
    for(size_t i = 0; i < bodies.size(); i++) {
        const cv::Rect &body = bodies.at(i);
        icl::measurement_2d_fullbody m;
        m.u = body.x + body.width / 2.0;
        m.v = body.y + body.height / 2.0;
        m.h = body.height;
        m.weight = weights[i];
        if(upscale_image) {
          m.u /= 2.0;
          m.v /= 2.0;
          m.h /= 2.0;
        }
        meas.push_back(m);
#ifdef DEBUG
        cv::rectangle(dbg_img, body.tl(), body.br(), cv::Scalar(0, 0, 255));
        cv::circle(dbg_img, cv::Point(meas[i].u, meas[i].v), 5, cv::Scalar(0, 0, 255), -1);
#endif
    }
#else
#ifdef DEBUG
    cv::bitwise_and(image_raw, cv::Scalar(255,255,255), dbg_img, roi);
#endif
    cv::Mat masked = cv::Mat::zeros(image_raw.size(), CV_8UC3);
    cv::bitwise_and(image_raw, cv::Scalar(255,255,255), masked, roi);
    // Detect people
    std::vector<cv::Rect> hits;
    people_detector->detectMultiScale(masked, hits);
    std::vector<icl::measurement_2d_fullbody> meas(hits.size());
    for(size_t i = 0; i < hits.size(); i++) {
      meas[i].u = hits[i].x  + (hits[i].width >> 1);
      meas[i].v = hits[i].y  + (hits[i].height >> 1);
      meas[i].h = hits[i].height;
      meas[i].weight = 1.0;
#ifdef DEBUG
      cv::rectangle(dbg_img, hits[i].tl(), hits[i].br(), CV_RGB(0, 0, 255));
#endif
    }
#endif

    int extra_detections = 0;
#ifdef IMAGE_TRACKING
    // Look for previously detected targets
    if(!previous_detections.empty()) {
      for(size_t i = 0; i < previous_detections.size(); i++) {
        // Create the result image
        cv::Rect &body = previous_detections[i].first;
        cv::Mat &tmpl = previous_detections[i].second;
        // Select where to look for the template
        cv::Rect roi(body.x - tmpl_roi_size,
                     body.y - tmpl_roi_size,
                     body.width + 2*tmpl_roi_size,
                     body.height + 2*tmpl_roi_size);
        if(roi.x < 0) roi.x = 0;
        if(roi.x + roi.width > image.cols) roi.width = image.cols - roi.x;
        if(roi.y < 0) roi.y = 0;
        if(roi.y + roi.height > image.rows) roi.height = image.rows - roi.y;
        cv::Mat result, mask = image(roi);
        // Match the template
        cv::matchTemplate(mask, tmpl, result, tmpl_match_algorithm);
        // Localise the best match
        double min_val, max_val; cv::Point min_point, max_point;
        cv::minMaxLoc(result, &min_val, &max_val, &min_point, &max_point);
        const cv::Point &detect_point = (tmpl_match_algorithm == CV_TM_SQDIFF
                                         || tmpl_match_algorithm == CV_TM_SQDIFF_NORMED
                                        ? min_point : max_point) + roi.tl();
        const double &detect_val = (tmpl_match_algorithm == CV_TM_SQDIFF
                                    || tmpl_match_algorithm == CV_TM_SQDIFF_NORMED
                                   ? min_val : max_val);
        // Check it's a unique detection
        bool undetected = true;
        int u = detect_point.x + previous_detections[i].second.cols / 2.0;
        int v = detect_point.y + previous_detections[i].second.rows / 2.0;
        for(size_t j = 0; j < meas.size(); j++) {
          float du = meas[j].u - u;
          float dv = meas[j].v - v;
          float dist_sq = du*du + dv*dv;
          if(dist_sq < template_dist_epsilon) {
            undetected = false;
            break;
          }
        }
        const double &template_match_epsilon_ = template_match_epsilon;
        // Check that the detection is not detected by the HOG.
        // Gives preference to detection from the HOG detector
        if(undetected && tmpl_output_compare(detect_val, template_match_epsilon_)) {
#ifdef DEBUG
          cv::rectangle(
                dbg_img,
                detect_point,
                cv::Point(detect_point.x + previous_detections[i].second.cols,
                          detect_point.y + previous_detections[i].second.rows),
                CV_RGB(0, 255, 0)
          );
#endif
          icl::measurement_2d_fullbody tracked_meas;
          tracked_meas.u = u;
          tracked_meas.v = v;
          tracked_meas.h = previous_detections[i].second.rows;
          meas.push_back(tracked_meas);
          extra_detections++;
        }
      }
    }
#endif
    // If we skip pictures with no detections
    if(omit_empty_measurements && !meas.size()) {
      return;
    }

    // Create the birth process mixture
    std::vector<icl::person_state::ptr> birth;
#ifdef UT_BIRTH_MODEL
#ifdef DEBUG
std::cout << "birth_model = []" << std::endl;
#endif
#endif
    for(size_t i = 0; i < meas.size() - extra_detections; i++) {
#ifdef UT_BIRTH_MODEL
      Eigen::Matrix<double,6,1> mean_detect;
      mean_detect << meas[i].u, meas[i].v, meas[i].h, 0.0, 0.0, expected_height;
      Eigen::Matrix<double,6,6> cov_detect = detection_pixel_variance*Eigen::Matrix<double,6,6>::Identity();
      cov_detect(2,2) = detection_height_variance;
      // Velocities
      cov_detect(3,3) = 1.0;
      cov_detect(4,4) = 1.0;
      // Real height
      cov_detect(5,5) = var_height;
      std::vector<Eigen::Matrix<double,6,1>, Eigen::aligned_allocator<Eigen::Matrix<double,6,1> > > sigma_points;
      std::vector<double> sigma_weights;
      // Create the sigma points
      unsctd_sampler.get_points(mean_detect, cov_detect, 0.2, 1.0, sigma_points, sigma_weights);
      std::vector<Eigen::Matrix<double,5,1>, Eigen::aligned_allocator<Eigen::Matrix<double,5,1> > > state_estimations(sigma_weights.size());
      Eigen::Matrix<double,5,1> expected_mean_posvelheight = Eigen::Matrix<double,5,1>::Zero();
      Eigen::Matrix<double,5,5> expected_cov_posvelheight = Eigen::Matrix<double,5,5>::Zero();
      for(size_t j = 0; j < sigma_points.size(); j++) {
        const Eigen::Matrix<double,6,1> &point = sigma_points[j];
        double expected_distance = point(5) / (point(2)) * f;
        state_estimations[j].block(0,0,4,1) = Tc * (Eigen::Matrix<double,4,1>() <<
          (point(0) - P(0,2) - P(0,3)) / P(0,0) * expected_distance,
          (point(1) - P(1,2) - P(1,3)) / P(1,1) * expected_distance,
          expected_distance,
          1.0).finished();
        // Velocity components
        state_estimations[j](2) = point(3);
        state_estimations[j](3) = point(4);
        // Height component
        state_estimations[j](4) = point(5);
        expected_mean_posvelheight += sigma_weights[j] * state_estimations[j];
      }
      for(size_t j = 0; j < sigma_points.size(); j++) {
        Eigen::Matrix<double,5,1> dx =  state_estimations[j] - expected_mean_posvelheight;
        expected_cov_posvelheight += sigma_weights[j] * dx * dx.transpose();
      }
      icl::person_state::ptr new_person(new icl::person_state());
      new_person->weight = total_expected_birth / meas.size();
      new_person->posvelheight = expected_mean_posvelheight;
      new_person->posvelheight_cov = expected_cov_posvelheight;
      birth.push_back(new_person);
#ifdef DEBUG
std::cout << "birth_model.append(([" <<
    expected_mean_posvelheight(0) << ", " <<
    expected_mean_posvelheight(1) << ", " <<
    expected_mean_posvelheight(2) << ", " <<
    expected_mean_posvelheight(3) << ", " <<
    expected_mean_posvelheight(4) << "], [";
for(size_t l = 0; l < 5; l++) {
  std::cout << "[" ;
  for(size_t m = 0; m < 5; m++)
    std::cout << expected_cov_posvelheight(l,m) << ", ";
  std::cout << "]," << std::endl;

}
std::cout << "]))" << std::endl;
#endif
#else
      double expected_distance = expected_height / (meas[i].h-32) * f;
      Eigen::VectorXd pos_map = Tc * (Eigen::VectorXd(4) <<
        (meas[i].u - P(0,2) - P(0,3)) / P(0,0) * expected_distance,
        (meas[i].v - P(1,2) - P(1,3)) / P(1,1) * expected_distance,
        expected_distance,
      1).finished();
      Eigen::VectorXd tmp_ = K*pos_map; tmp_ /= tmp_(2);
      Eigen::VectorXd pos_robot = Tr.inverse() * pos_map;
      icl::person_state::ptr new_person(new icl::person_state());
      new_person->weight = total_expected_birth / meas.size();
      new_person->posvelheight = Eigen::Matrix<double,5,1>::Zero();
      new_person->posvelheight.block(0,0,2,1) = pos_map.block(0,0,2,1);
      new_person->posvelheight(4) = expected_height;
      Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2,2);
      Q.block(0,0,2,1) = pos_robot.block(0,0,2,1).normalized();
      Q.block(0,1,2,1) = (Eigen::VectorXd(2) << Q(1,0), -Q(0,0)).finished();
      Eigen::MatrixXd A = (Eigen::VectorXd(2) << 1.0, 0.25).finished().asDiagonal();
      new_person->posvelheight_cov <<
        1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.5, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.5, 0.0,
        0.0, 0.0, 0.0, 0.0, var_height;
      new_person->posvelheight_cov.block(0,0,2,2) = Q*A*Q.inverse();
      birth.push_back(new_person);
#endif
    }

#ifdef IMAGE_TRACKING
    std::vector<Eigen::VectorXd> state;
#endif
#ifdef DEBUG
std::cout << "# num measurements: " << meas.size() << std::endl;
#endif
    // Do the filtering
    {
      boost::mutex::scoped_lock process_lock(mtx);
      double dt = time - last_time;
#ifdef DEBUG
std::cout << "# dt for camera phd update: " << dt << std::endl;
#endif
      last_time = time;
      mp_tracker->predict(dt, robotPosition(Tr), time);
      mp_tracker->correct_feature_2d_fullbody_ut(meas, K, Tr, birth);
      // Copy the estimation from the full body correction to the states_tp1
      mp_tracker->states_tp1.resize(mp_tracker->states_t.size());
      std::copy(mp_tracker->states_t.begin(), mp_tracker->states_t.end(), mp_tracker->states_tp1.begin());
      mp_tracker->correct_laser_mc(laser->ranges, laser->angle_min, laser->angle_increment, Tl);
#ifndef SAMPLE_FROM_MIXTURE
      // Prune only if the sampling is not from the GMM directly
      mp_tracker->prune();
#endif
#ifdef IMAGE_TRACKING
      mp_tracker->get_state(state);
#endif
    }
#ifdef IMAGE_TRACKING
    previous_detections.resize(0);
    for(size_t i = 0; i < state.size(); i++) {
      Eigen::Vector4d pos, pos_rel_robot;
      if(state.at(i).rows() == 0) {
        // Sometime this happens. Dunno why
        ROS_WARN_STREAM("Some problem with the extracted states.");
        continue;
      }
      // Position in 3D space is (x,y,h/2)
      pos << state.at(i)(0), state.at(i)(1), state.at(i)(4) / 2.0, 1.0;
      Eigen::Vector3d pixel = K * pos; pixel /= pixel(2);
      pos_rel_robot = Tci * pos;
      double height = state.at(i)(4) * f / pos_rel_robot(2);
      double width = 0.5 * f / pos_rel_robot(2);
      cv::Rect body(
          pixel(0) - width / 2.0,
          pixel(1) - height / 2.0 + 2.0 * pos_rel_robot(2),
          width,
          height);
      if(body.x < 0) body.x = 0;
      if(body.x + body.width > image.cols) body.width = image.cols - body.x;
      if(body.y < 0) body.y = 0;
      if(body.y + body.height > image.rows) body.height = image.rows - body.y;
#ifdef DEBUG
std::cout << "# position relative to the robot: " << pos_rel_robot.transpose() << std::endl;
#endif
      // TODO check that pos_rel_robot is within the FOV of the camera
      if(body.height <= 0 || body.width <= 0 || pos_rel_robot(2) < 1.0)
        continue;
#ifdef DEBUG
      cv::circle(dbg_img, cv::Point(pixel(0), pixel(1)), 10, CV_RGB(255, 255, 255));
      cv::rectangle(dbg_img, body.tl(), body.br(), CV_RGB(255, 0, 0));
#endif
      // Extract the images
      cv::Mat body_img = image(body).clone();
      previous_detections.push_back(std::make_pair(body, body_img));
    }
#endif
#ifdef DEBUG
    cv::imshow("Debug window 2", dbg_img);
#endif
  }

#ifdef DEBUG
  void image_dbg_cb(const ImageConstPtr &img, const CameraInfoConstPtr &cam) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, enc::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat &image = cv_ptr->image;
    cv::Mat dbg = image.clone();

		Eigen::Matrix4d Tc, Tr;
    try {
  		icl::lookupTransform(*tf_listener, img->header.frame_id, tracking_frame, Tc);
    } catch(tf::TransformException e) {
      return;
    }

    Eigen::MatrixXd K = (Eigen::MatrixXd(3,4) << cam->P[0], cam->P[1], cam->P[2], cam->P[3],
         cam->P[4], cam->P[5], cam->P[6], cam->P[7],
         cam->P[8], cam->P[9], cam->P[10], cam->P[11]).finished() * Tc;
    std::vector<Eigen::VectorXd> state;
    {
      boost::mutex::scoped_lock process_lock(mtx);
      mp_tracker->get_state(state);
    }
/*
    for(size_t i = 0; i < state.size(); i++) {
      Eigen::Vector4d x_feet, x_head;
      x_feet << state.at(i)(0), state.at(i)(1), 0.0, 1.0;
      x_head << state.at(i)(0), state.at(i)(1), state.at(i)(4), 1.0;
      Eigen::Vector3d pixel_head = K * x_head;
      Eigen::Vector3d pixel_feet = K * x_feet;
      double height = (pixel_head(1)/pixel_head(2) - pixel_feet(1)/pixel_feet(2));
      double width = 0;
      Eigen::Vector3d pixel = (pixel_head + pixel_feet) / 2.0;
      pixel /= pixel(2);
      cv::Point p1(pixel(0) - width / 2, pixel(1) - height / 2);
      cv::Point p2(pixel(0) + width / 2, pixel(1) + height / 2);
      cv::rectangle(image, p1, p2, CV_RGB(0, 0, 255) );
      cv::circle(image, cv::Point(pixel(0), pixel(1)), 10, CV_RGB(0, 0, 255));
    }
*/
    std::vector<cv::Scalar> colours;
    colours.push_back(CV_RGB(0, 0, 255));
    colours.push_back(CV_RGB(0, 128, 128));
    colours.push_back(CV_RGB(0, 255, 0));
    colours.push_back(CV_RGB(128, 128, 0));
    colours.push_back(CV_RGB(255, 0, 0));
    colours.push_back(CV_RGB(128, 0, 128));
    std::vector<cv::Point> cvpoints;
#ifdef MANAGE_TRACKS
    for(size_t i = 0; i < mp_tracker->active_paths.size(); i++) {
      size_t current_path = mp_tracker->active_paths[i];
      std::map<size_t, std::map<double, Eigen::VectorXd> >::const_iterator points =  mp_tracker->paths.find(current_path);
      if(points == mp_tracker->paths.end() || !points->second.size())
        continue;
      cvpoints.resize(points->second.size());
      size_t j = 0;
      std::map<double, Eigen::VectorXd>::const_iterator it2;
      for(it2 = points->second.begin(); it2 != points->second.end(); it2++) {
        Eigen::VectorXd pixel = K * (Eigen::Vector4d() << it2->second(0), it2->second(1), 0.0, 1.0).finished();
        cvpoints[j].x = pixel(0) / pixel(2);
        cvpoints[j].y = pixel(1) / pixel(2);
        j++;
      }
      const cv::Point *aux = &cvpoints[0];
      int numpoints[] = {cvpoints.size()};
      cv::polylines(image, &aux, numpoints, 1, false, colours[i % colours.size()], 2);
      cvpoints.resize(0);
    }
#endif
/*
    for(size_t i = 0; i < last_points.size(); i++) {
      Eigen::VectorXd pixel = K * (Eigen::Vector4d() << last_points.at(i)(0), last_points.at(i)(1), 0.0, 1.0).finished();
      cv::Point cvpoint;
      cvpoint.x = pixel(0) / pixel(2);
      cvpoint.y = pixel(1) / pixel(2);
      cv::circle(image, cvpoint, 1, CV_RGB(255,255,255), 2);
    }
*/
    cv::imshow("Debug window", image);
  }
#endif

  void spin(const ros::TimerEvent &) {
    pcl::PointCloud<icl::PointWithVelocity>::Ptr pc(new pcl::PointCloud<icl::PointWithVelocity>);
    // Publish
    {
      boost::mutex::scoped_lock process_lock(mtx);
      mp_tracker->get_state(pc, true);
      pc->width = 1;
      pc->height = pc->size();
      pc->is_dense = true;
      pc->header.frame_id = tracking_frame;
      pc->header.stamp = last_time;

      
    }
    track_pub.publish(pc);

    // For visualization purposes
    visualization_msgs::MarkerArray mrk_array;
    size_t i = 0;
    for(; i < mp_tracker->states_t.size(); i++) {
      visualization_msgs::Marker m1;
      m1.header.frame_id = "/map";
      m1.header.stamp = ros::Time::now();
      m1.ns = "tracker";
      m1.id = 2*i;
      m1.type = visualization_msgs::Marker::CUBE;
      m1.action = visualization_msgs::Marker::ADD;
      m1.pose.orientation.x = 0.0;
      m1.pose.orientation.y = 0.0;
      m1.pose.orientation.z = 0.0;
      m1.pose.orientation.w = 1.0;
      m1.scale.x = 0.3;
      m1.scale.y = 0.3;
      m1.scale.z = mp_tracker->states_t.at(i)->posvelheight[4];
      m1.pose.position.x = mp_tracker->states_t.at(i)->posvelheight[0];
      m1.pose.position.y = mp_tracker->states_t.at(i)->posvelheight[1];
      m1.pose.position.z = mp_tracker->states_t.at(i)->posvelheight[4] / 2.0;
      m1.lifetime = ros::Duration();
      m1.color.r = 0.0;
      m1.color.g = 0.0;
      m1.color.b = 1.0;
      m1.color.a = 0.6;

      visualization_msgs::Marker m2;
      m2.header.frame_id = "/map";
      m2.header.stamp = ros::Time::now();
      m2.ns = "tracker";
      m2.id = 2*i+1;
      m2.type = visualization_msgs::Marker::ARROW;
      m2.action = visualization_msgs::Marker::ADD;
      m2.lifetime = ros::Duration();
      m2.color.r = 0.0;
      m2.color.g = 1.0;
      m2.color.b = 0.0;
      m2.color.a = 0.6;
      m2.points.resize(2);
      m2.points[0].x = mp_tracker->states_t.at(i)->posvelheight[0];
      m2.points[0].y = mp_tracker->states_t.at(i)->posvelheight[1];
      m2.points[0].z = mp_tracker->states_t.at(i)->posvelheight[4] / 2.0;
      m2.points[1].x = mp_tracker->states_t.at(i)->posvelheight[0] + 10*mp_tracker->states_t.at(i)->posvelheight[2];
      m2.points[1].y = mp_tracker->states_t.at(i)->posvelheight[1] + 10*mp_tracker->states_t.at(i)->posvelheight[3];
      m2.points[1].z = mp_tracker->states_t.at(i)->posvelheight[4] / 2.0;

      mrk_array.markers.push_back(m1);
      mrk_array.markers.push_back(m2);
    }
    viz_pub.publish(mrk_array);
    num_points_published = pc->points.size();
  }

  MPTrackerNode():
    main_nh(),
    priv_nh("~"),
    mp_tracker(new icl::MPTracker),
    it(main_nh),
    num_points_published(0),
    omit_empty_measurements(true) {

    double detection_pixel_std, detection_height_std;
    priv_nh.param("upscale_image", upscale_image, false);
    priv_nh.param("detect_threshold", detect_threshold, 0.0);
    priv_nh.param("detection_pixel_std", detection_pixel_std, 10.0);
    priv_nh.param("detection_height_std", detection_height_std, 15.0);
    detection_pixel_variance = detection_pixel_std*detection_pixel_std;
    mp_tracker->pixel_error = detection_pixel_variance;
    detection_height_variance = detection_height_std*detection_height_std;
    mp_tracker->height_error = detection_height_variance;
    priv_nh.param("detection_height_border", detection_height_border, 0.0);
    mp_tracker->detection_border = detection_height_border;
    priv_nh.param("expected_height", expected_height, 1.7);
    double std_height;
    priv_nh.param("std_height", std_height, 0.1);
    var_height = std_height*std_height;
    priv_nh.param("expected_birth", total_expected_birth, 1.0);
    priv_nh.param("expected_birth", total_expected_birth, 1.0);
    priv_nh.param("simple_laser_likelihood_gamma", mp_tracker->simple_laser_likelihood_gamma, 0.8);
    std::string tm_algo;
    priv_nh.param("template_match_algorithm", tm_algo, std::string("CV_TM_SQDIFF_NORMED"));
    tmpl_match_algorithm = CV_TM_SQDIFF_NORMED;
    tmpl_output_compare = std::less<double>();
    if(tm_algo == "CV_TM_SQDIFF") {
      tmpl_match_algorithm = CV_TM_SQDIFF;
      tmpl_output_compare = std::less<double>();
    } else if (tm_algo == "CV_TM_CCORR") {
      tmpl_match_algorithm = CV_TM_CCORR;
      tmpl_output_compare = std::greater<double>();
    } else if (tm_algo == "CV_TM_CCORR_NORMED") {
      tmpl_match_algorithm = CV_TM_CCORR_NORMED;
      tmpl_output_compare = std::greater<double>();
    } else if (tm_algo == "CV_TM_CCOEFF") {
      tmpl_match_algorithm = CV_TM_CCOEFF;
      tmpl_output_compare = std::greater<double>();
    } else if (tm_algo == "CV_TM_CCOEFF_NORMED") {
      tmpl_match_algorithm = CV_TM_CCOEFF_NORMED;
      tmpl_output_compare = std::greater<double>();
    }
    tmpl_output_compare(1.0, 2.0);
    priv_nh.param("template_match_epsilon", template_match_epsilon, 2E-4);
    priv_nh.param("template_dist_epsilon", template_dist_epsilon, 1000.0);
    priv_nh.param("template_roi_size", tmpl_roi_size, 15);

    priv_nh.param("use_map_likelihood", mp_tracker->use_map_likelihood, true);
    priv_nh.param("tracking_frame", tracking_frame, std::string("map"));
    priv_nh.param("robot_frame", robot_frame, std::string("base_link"));
    priv_nh.param("cluster_proximity", mp_tracker->cluster_proximity, 0.15);

    priv_nh.param("laser_likelihood_sigma", mp_tracker->sigma, 0.4);
    priv_nh.param("laser_likelihood_angle_epsilon", mp_tracker->angle_epsilon, 3.0 / 180.0 * M_PI);

    priv_nh.param("max_dist_people_detector", mp_tracker->visual_max_dist_detect, 10.0);
    priv_nh.param("min_dist_people_detector", mp_tracker->visual_min_dist_detect, 2.0);
    priv_nh.param("probability_people_survival", mp_tracker->p_survival, 0.95);
    priv_nh.param("probability_people_detect", mp_tracker->p_fullbody_detect, 0.75);
    priv_nh.param("omit_empty_measurements", omit_empty_measurements, true);

    // Set the parameters
    priv_nh.param("truncate_threshold", mp_tracker->truncate_threshold, 1E-6);
    priv_nh.param("merge_threshold", mp_tracker->merge_threshold, 5E-1);
    priv_nh.param("max_components", mp_tracker->max_components, 100.0);
    priv_nh.param("clutter_fullbody", mp_tracker->clutter_fullbody_rfs, 1E-5);

    // Path generation cost
#ifdef MANAGE_TRACKS
    priv_nh.param("max_path_distance", mp_tracker->max_path_distance, 10.0);
    priv_nh.param("heading_path_cost", mp_tracker->heading_path_cost, 0.15);
#endif
#ifdef USE_HOG
#ifdef USE_LATENT
		people_detector.reset(cvLoadLatentSvmDetector("/data/person.xml"));
		if(!people_detector) {
			throw std::runtime_error("Could not load /usr/person.xml");
		}
#else
		people_detector.reset(new cv::HOGDescriptor());
//		if(!people_detector->load("/data/ros/people_detect_and_tracking/mp_tracker/data/hogdetector.yaml")) {
//			throw std::runtime_error("Could not load /data/ros/people_detect_and_tracking/mp_tracker/data/hogdetector.yaml");
//		}
    people_detector->winSize = cv::Size(48, 96);
		people_detector->setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
//		people_detector->setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
#endif
#else
		people_detector.reset(new cv::CascadeClassifier());
		if(!people_detector->load("/data/OpenCV-2.4.2/data/haarcascades/haarcascade_fullbody.xml")) {
			throw std::runtime_error("Could not load /usr/share/opencv/haarcascades/haarcascade_fullbody.xml");
		}
#endif
    mp_tracker->people_detector = people_detector;

    tf_listener.reset(new tf::TransformListener());

    // Initialise the last_time variable with the current time
    last_time = ros::Time::now().toSec();
#ifdef MANAGE_TRACKS
    mp_tracker->start_running = last_time;
#endif

    std::string laser_topic;
    priv_nh.param("laser_topic", laser_topic, std::string("scan"));
    // Subscribe to the image and laser
    std::string camera_topic, camera_info_topic;
    priv_nh.param("camera", camera_topic, std::string("camera"));
    priv_nh.param("camera_info", camera_info_topic, std::string("camera_info"));
    img_sub.reset(new message_filters::Subscriber<Image>(main_nh, camera_topic, 1));
    cam_sub.reset(new message_filters::Subscriber<CameraInfo>(main_nh, camera_info_topic, 1));
    laser_sub.reset(new message_filters::Subscriber<LaserScan>(main_nh, laser_topic, 1));
    synch.reset(new Synchronizer(Policy(10), *img_sub, *cam_sub, *laser_sub));
    synch->registerCallback(boost::bind(&MPTrackerNode::sensors_cb, this, _1, _2, _3));
#ifdef DEBUG
    dbg_sub = it.subscribeCamera(camera_topic, 1, boost::bind(&MPTrackerNode::image_dbg_cb, this, _1, _2));
#endif
    // Create the publisher
    track_pub = main_nh.advertise<pcl::PointCloud<icl::PointWithVelocity> >("tracked", 10);
    viz_pub = main_nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

		nav_msgs::GetMap::Request req;
		nav_msgs::GetMap::Response resp;
/*
    if(mp_tracker->use_map_likelihood) {
      if(!ros::service::call("static_map", req, resp)) {
        ROS_ERROR("No map available!");
        throw(std::runtime_error("Map needed"));
      } else {
        mp_tracker->static_map.origin_x = resp.map.info.origin.position.x + (resp.map.info.width / 2) * resp.map.info.resolution;
        mp_tracker->static_map.origin_y = resp.map.info.origin.position.y + (resp.map.info.height / 2) * resp.map.info.resolution;
        mp_tracker->static_map.size_x = resp.map.info.width;
        mp_tracker->static_map.size_y = resp.map.info.height;
        mp_tracker->static_map.resolution = resp.map.info.resolution;
        mp_tracker->static_map.map.resize(resp.map.data.size());
        std::copy(resp.map.data.begin(), resp.map.data.end(), mp_tracker->static_map.map.begin());
        mp_tracker->static_map.calc_obstacle_distances();
      }
    }
*/
    mp_tracker->static_map.initialised = true;
#ifdef DEBUG
		// OpenCV window
		cv::namedWindow("Debug window");
    cv::namedWindow("Debug window 2");
    cv::startWindowThread();
#endif

    // Create the main timer
    spin_timer = main_nh.createTimer(ros::Duration(0.05), boost::bind(&MPTrackerNode::spin, this, _1));
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_person_tracker");
  // The main node
  MPTrackerNode node;
  // Spin
  ros::MultiThreadedSpinner spinner;
  spinner.spin();
#ifdef MANAGE_TRACKS
  std::cout << "# Paths" << std::endl;
  std::map<size_t, std::map<double, Eigen::VectorXd> >::iterator it;
  for(it = node.mp_tracker->paths.begin(); it != node.mp_tracker->paths.end(); it++) {
    std::cout << "path_" << it->first << "=[" << std::endl;
    std::map<double, Eigen::VectorXd>::iterator it2;
    for(it2 = it->second.begin(); it2 != it->second.end(); it2++) {
      std::cout << "[" << it2->first << ", " << it2->second(0) << ", " << it2->second(1) << ", " << it2->second(2) << ", " << it2->second(3) << ", " << it2->second(4) <<  "], " << std::endl;
    }
    std::cout << "]" << std::endl;
  }
#endif
  return 0;
}

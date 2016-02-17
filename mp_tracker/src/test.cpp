#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <icl_robot/plane_filter.h>
#include <icl_robot/depth_connected_components.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/progress.hpp>
#include <limits>

using namespace pcl;
using namespace ros;
using namespace std;
using namespace sensor_msgs;
using namespace Eigen;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

typedef message_filters::sync_policies::ApproximateTime<PointCloud<PointXYZ>, Image, CameraInfo> Policy;
typedef message_filters::Synchronizer<Policy> Synchronizer;

Publisher track_pub;
VectorXf model_coefficients;
boost::shared_ptr<icl::PlaneFilter<PointXYZ> > floor_filter;
boost::shared_ptr<icl::PlaneFilter<PointXYZ> > ceiling_filter;
boost::shared_ptr<VoxelGrid<PointXYZ> > voxel_filter;
boost::shared_ptr<cv::HOGDescriptor> people_detector;
Mat prevgray, gray, flow, cflow, cflowmap;

static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step, const Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {   
            const Point2f& fxy = flow.at<Point2f>(y, x); 
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }   
}

void image_cb(const ImageConstPtr &image) {
  boost::progress_timer t;
  // Get the opencv image from the ROS message
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image, enc::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  Mat &image_raw = cv_ptr->image;
  cvtColor(image_raw, gray, CV_BGR2GRAY);
//  if(prevgray.data) {
//    calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
//    drawOptFlowMap(flow, cflow, 16, CV_RGB(0, 255, 0));
//    imshow("Display", cflow);
//  }
//  std::swap(prevgray, gray);

  vector<Rect> hits;
  people_detector->detectMultiScale(gray, hits, 0, Size(), Size(), 1.2);
  for(size_t i = 0; i < hits.size(); i++) {
    rectangle(image_raw, hits[i].br(), hits[i].tl(), CV_RGB(0, 0, 255));
  }
  // Show the image
  imshow("Display", image_raw);
}

void sensors_cb(const PointCloud<PointXYZ>::ConstPtr &pc, const ImageConstPtr &image, const CameraInfoConstPtr &cam) {
  // Get the opencv image from the ROS message
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image, enc::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  Mat &image_raw = cv_ptr->image;
  // Project to the camera and calculate bounding box
  Matrix<double, 3, 4> P = (Eigen::Matrix<double,3,4>() << cam->P[0], cam->P[1], cam->P[2], cam->P[3],
       cam->P[4], cam->P[5], cam->P[6], cam->P[7],
       cam->P[8], cam->P[9], cam->P[10], cam->P[11]).finished();

  PointCloud<PointXYZ>::Ptr out(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr voxelised(new PointCloud<PointXYZ>);
  floor_filter->setInputCloud(pc);
  floor_filter->filter(*out);
  ceiling_filter->setInputCloud(pc);
  ceiling_filter->filter(*out);
  voxel_filter->setInputCloud(out);
  voxel_filter->filter(*voxelised);

/*
  vector<Point> points;
  for(PointCloud<PointXYZ>::iterator p = voxelised->begin(); p != voxelised->end(); p++) {
    Vector4d point = (Vector4d() << p->x, p->y, p->z , 1.0).finished();
    Vector3d pixel = P * point; pixel /= pixel[2];
    points.push_back(Point(pixel[0], pixel[1]));
  }
  Rect bb = boundingRect(points);
  bb.y -= 30;
  bb.height += 60;
  if(bb.y + bb.height >= cam->height)
    bb.height = cam->height - bb.y - 1;
  rectangle(image_raw, bb.br(), bb.tl(), CV_RGB(255, 0, 0));
  vector<Rect> hits;
  Mat img(image_raw, bb);
  people_detector->detectMultiScale(img, hits);
  for(size_t i = 0; i < hits.size(); i++) {
    rectangle(image_raw, bb.br() + hits[i].br(), hits[i].tl(), CV_RGB(0, 0, 255));
  }
*/
/*
  KdTree<PointXYZ>::Ptr tree(new KdTreeFLANN<PointXYZ>);
  tree->setEpsilon(0.1);
  tree->setInputCloud(voxelised);

  vector<PointIndices> clusters;
  extractEuclideanClusters(*voxelised, tree, 0.15, clusters, 100, 100000);
//  icl::extractEuclideanClusters(*voxelised, clusters, 0.15, 100, 100000);

  // Merge close clusters
  for(size_t i = 0; i < clusters.size(); i++) {
  }

  // Split cluster by height
  vector<PointIndices> final_cluster;
  for(size_t i = 0; i < clusters.size(); i++) {
    while(clusters[i].indices.size() > 100) {
      double min_y = 1000.0;
      PointXYZ min_pt;
      for(size_t j = 0; j < clusters[i].indices.size(); j++) {
        if(voxelised->at(clusters[i].indices[j]).y < min_y) {
          min_pt = voxelised->at(clusters[i].indices[j]);
          min_y = min_pt.y;
        }
      }
      PointIndices new_subcluster;
      for(size_t j = 0; j < clusters[i].indices.size(); j++) {
        size_t indx = clusters[i].indices[j];
        const PointXYZ &actual_pt = voxelised->at(indx);
        // Check the distance on the ground (excluding y component)
        double dx = actual_pt.x - min_pt.x;
        double dz = actual_pt.z - min_pt.z;
        if((dx * dx + dz * dz) < 0.1) {
          new_subcluster.indices.push_back(indx);
          clusters[i].indices.erase(clusters[i].indices.begin() + j);
          j--;
        }
      }
      if(new_subcluster.indices.size() > 100)
        final_cluster.push_back(new_subcluster);
    }
  }

  for(size_t i = 0; i < final_cluster.size(); i++) {
    vector<Point> points;
    for(size_t j = 0; j < final_cluster[i].indices.size(); j++) {
      const PointXYZ &actual_pt = voxelised->at(final_cluster[i].indices[j]);
      Vector4d point = (Vector4d() << actual_pt.x, actual_pt.y, actual_pt.z , 1.0).finished();
      Vector3d pixel = P * point; pixel /= pixel[2];
      points.push_back(Point(pixel[0], pixel[1]));
    }
    Rect bb = boundingRect(points);
    rectangle(image_raw, bb.br(), bb.tl(), CV_RGB(0, 0, 255));
  }
*/
  // "Show" the pointcloud
  voxelised->header = pc->header;
  track_pub.publish(voxelised);

  // Show the image
//  imshow("Display", img);
}


int main(int argc, char **argv) {
  init(argc, argv, "concept_testing");
  
  NodeHandle main_nh;

  boost::shared_ptr<message_filters::Subscriber<PointCloud<PointXYZ> > > pointcloud_sub;
  boost::shared_ptr<message_filters::Subscriber<Image> > img_sub;
  boost::shared_ptr<message_filters::Subscriber<CameraInfo> > cam_sub;
  boost::shared_ptr<Synchronizer> synch;

  string points_topic = "/openni/depth/points";
  string camera_topic = "/openni/rgb/image_color";
  string camera_info_topic = "/openni/rgb/camera_info";

  // Find the floor parameters
  PointCloud<PointXYZ>::ConstPtr cloud_ = topic::waitForMessage<PointCloud<PointXYZ> >(points_topic);
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  copyPointCloud(*cloud_, *cloud);

  // created RandomSampleConsensus object and compute the appropriated model
/*
  SampleConsensusModelPerpendicularPlane<PointXYZ>::Ptr model(new SampleConsensusModelPerpendicularPlane<PointXYZ>(cloud));
  Vector3f axis;
  axis << 0.0, 1.0, 0.0;
  // Plane in Y axis with 10 degree eps
  for(size_t j = 0; j < 2; j++) {
    model->setAxis(axis);
    model->setEpsAngle(0.53);
    RandomSampleConsensus<PointXYZ> ransac (model);
    ransac.setDistanceThreshold(.1);
    ransac.computeModel();
    ransac.getModelCoefficients(model_coefficients);
    std::vector<int> I;
    ransac.getInliers(I);
    for(size_t i = 0; i < I.size(); i++){
      cloud->points[I[i]].x = numeric_limits<float>::quiet_NaN();
      cloud->points[I[i]].y = numeric_limits<float>::quiet_NaN();
      cloud->points[I[i]].z = numeric_limits<float>::quiet_NaN();
    }
    std::cout << "Model coefficients: " << model_coefficients[0] << ", " << model_coefficients[1] << ", " << model_coefficients[2] << ", " << model_coefficients[3] << std::endl;
    model->setInputCloud(cloud);
 }
*/
  floor_filter.reset(new icl::PlaneFilter<PointXYZ>(-0.00214303, -0.999838, -0.01785, 1.25201, 0.0, true));
  ceiling_filter.reset(new icl::PlaneFilter<PointXYZ>(-0.00214303, -0.999838, -0.01785, -1.0, 0.0, false));
  voxel_filter.reset(new VoxelGrid<PointXYZ>);
  voxel_filter->setLeafSize(0.06, 0.06, 0.06);
  pointcloud_sub.reset(new message_filters::Subscriber<PointCloud<PointXYZ> >(main_nh, points_topic, 1));
  img_sub.reset(new message_filters::Subscriber<Image>(main_nh, camera_topic, 1));
  cam_sub.reset(new message_filters::Subscriber<CameraInfo>(main_nh, camera_info_topic, 1));
  synch.reset(new Synchronizer(Policy(10), *pointcloud_sub, *img_sub, *cam_sub));
  synch->registerCallback(&sensors_cb);

  people_detector.reset(new cv::HOGDescriptor());
  people_detector->setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
//  if(!people_detector->load("/data/ros/people_detect_and_tracking/mp_tracker/data/hogdetector.yaml")) {
//    throw std::runtime_error("Could not load /data/ros/people_detect_and_tracking/mp_tracker/data/hogdetector.yaml");
//  }
//  ros::Subscriber image_sub = main_nh.subscribe<sensor_msgs::Image>(camera_topic, 10, &image_cb);
/*
  people_detector.reset(new cv::CascadeClassifier());
  if(!people_detector->load("/data/OpenCV-2.4.2/data/lbpcascades/lbpcascade_frontalface.xml")) {
    throw std::runtime_error("Could not load /usr/share/opencv/haarcascades/haarcascade_fullbody.xml");
  }
*/

  track_pub = main_nh.advertise<pcl::PointCloud<PointXYZ> >("/info", 10);

  // Create the display windows
  namedWindow("Display");
  startWindowThread();

  // ROS Spin
  spin();

  return 0;
}


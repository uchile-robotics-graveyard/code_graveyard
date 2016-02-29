#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <icl_robot/plane_filter.h>

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace pcl;

boost::shared_ptr<icl::PlaneFilter<PointXYZ> > floor_filter;
boost::shared_ptr<icl::PlaneFilter<PointXYZ> > ceiling_filter;

unsigned short fix(unsigned short old) {
  unsigned char *px = (unsigned char *)(void*)&old;
  unsigned char py[2];
  py[0] = px[1];
  py[1] = px[0];
  unsigned short *pz = (unsigned short *)(void*)py;
  return *pz;
}

void createPointcloud(const Mat &depth_image1,
                      const Matrix4f &T1,
                      const Mat &depth_image2,
                      const Matrix4f &T2,
                      const Mat &depth_image3,
                      const Matrix4f &T3,
                      PointCloud<PointXYZ>::Ptr &output) {
  // Use correct principal point from calibration
  float center_x = 3.3930780975300314e+02;
  float center_y = 2.4273913761751615e+02;
  
  // Allocate the pointcloud
  output->height = depth_image1.cols;
  output->width = 3*depth_image1.rows; 
  output->points.resize(output->width * output->height);

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  float constant_x = 1.0 / 5.9421434211923247e+02;
  float constant_y = 1.0 / 5.9104053696870778e+02;
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  PointCloud<PointXYZ>::iterator pt_iter = output->begin();
  for (int v = 0; v < (int)depth_image1.rows; ++v) {
    for (int u = 0; u < (int)depth_image1.cols; ++u) {   
      pcl::PointXYZ& pt1 = *pt_iter++;
      pcl::PointXYZ& pt2 = *(pt_iter+(depth_image1.cols * depth_image1.rows));
      pcl::PointXYZ& pt3 = *(pt_iter+2*(depth_image1.cols * depth_image1.rows));
      unsigned short depth1 = fix(depth_image1.at<unsigned short>(v, u));
      unsigned short depth2 = fix(depth_image2.at<unsigned short>(v, u));
      unsigned short depth3 = fix(depth_image3.at<unsigned short>(v, u));

      // Missing points denoted by NaNs
      if (depth1 == 0 || depth1 >= 1084) {   
        pt1.x = pt1.y = pt1.z = bad_point;
        continue;
      }   
      double depth1_ = 8.0 * 0.075 * 5.9421434211923247e+02 / (1084 - depth1);
      // Fill in XYZ
      pt1.x = (u - center_x) * depth1_ * constant_x;
      pt1.y = (v - center_y) * depth1_ * constant_y;
      pt1.z = depth1_;
      Vector4f tmp = T1 * (Vector4f() << pt1.getVector3fMap(), 1.0).finished();
      pt1.x = tmp(0);
      pt1.y = tmp(1);
      pt1.z = tmp(2);
      // Missing points denoted by NaNs
      if (depth2 == 0 || depth2 >= 1084) {   
        pt2.x = pt2.y = pt2.z = bad_point;
        continue;
      }
      double depth2_ = 8.0 * 0.075 * 5.9421434211923247e+02 / (1084 - depth2);
      // Fill in XYZ
      pt2.x = (u - center_x) * depth2_ * constant_x;
      pt2.y = (v - center_y) * depth2_ * constant_y;
      pt2.z = depth2_;
      tmp = T2 * (Vector4f() << pt2.getVector3fMap(), 1.0).finished();
      pt2.x = tmp(0);
      pt2.y = tmp(1);
      pt2.z = tmp(2);
      // Missing points denoted by NaNs
      if (depth3 == 0 || depth3 >= 1084) {   
        pt3.x = pt3.y = pt3.z = bad_point;
        continue;
      }   
      double depth3_ = 8.0 * 0.075 * 5.9421434211923247e+02 / (1084 - depth3);
      // Fill in XYZ
      pt3.x = (u - center_x) * depth3_ * constant_x;
      pt3.y = (v - center_y) * depth3_ * constant_y;
      pt3.z = depth3_;
      tmp = T3 * (Vector4f() << pt3.getVector3fMap(), 1.0).finished();
      pt3.x = tmp(0);
      pt3.y = tmp(1);
      pt3.z = tmp(2);
    }
  }

}

int main(int, char**) {
  Mat img1 = imread("/data/bagfiles/rgbd_people/mensa_seq0_1.1/depth/seq0_0015_0.pgm", CV_LOAD_IMAGE_ANYDEPTH);
  Mat img2 = imread("/data/bagfiles/rgbd_people/mensa_seq0_1.1/depth/seq0_0015_1.pgm", CV_LOAD_IMAGE_ANYDEPTH);
  Mat img3 = imread("/data/bagfiles/rgbd_people/mensa_seq0_1.1/depth/seq0_0015_2.pgm", CV_LOAD_IMAGE_ANYDEPTH);
  float angle1 = -43.0  * M_PI / 180;
  float angle2 = 47  * M_PI / 180;
  Affine3f T1 = Translation3f(0.0, 0.0, 0.08) * AngleAxisf(angle1, Vector3f(0.0, 1.0, 0.0)) * AngleAxisf(-M_PI / 2.0, Vector3f(0.0, 0.0, 1.0));
  Affine3f T2 = Translation3f(0.0, 0.0, 0.08) * AngleAxisf(-M_PI / 2.0, Vector3f(0.0, 0.0, 1.0));
  Affine3f T3 = Translation3f(0.0, 0.0, 0.08) * AngleAxisf(angle2, Vector3f(0.0, 1.0, 0.0)) * AngleAxisf(-M_PI / 2.0, Vector3f(0.0, 0.0, 1.0));

  PointCloud<PointXYZ>::Ptr pc(new PointCloud<PointXYZ>);
  //createPointcloud(img1, T1.matrix(), img2, T2.matrix(), img3, T3.matrix(), pc);
  createPointcloud(img1, Matrix4f::Identity(), img2, Matrix4f::Identity(), img3, Matrix4f::Identity(), pc);

  floor_filter.reset(new icl::PlaneFilter<PointXYZ>(0.999913581903804, -0.0125094876565810, -0.00404245505527142, 1.10, 0.0, true));
  ceiling_filter.reset(new icl::PlaneFilter<PointXYZ>(0.999913581903804, -0.0125094876565810, -0.00404245505527142, -1.10, 0.0, false));
  floor_filter->setInputCloud(pc);
  floor_filter->filter(*pc);
  ceiling_filter->setInputCloud(pc);
  ceiling_filter->filter(*pc);
  visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(pc);
  while (!viewer.wasStopped()){}
  io::savePCDFileASCII ("test_pcd.pcd", *pc);
/*
 Mat img(3*img1.rows, img1.cols, img1.type());
  Mat tmp1 = img(Rect(0,0,640,480));
  img1.copyTo(tmp1);
  Mat tmp2 = img(Rect(0,480,640,480));
  img2.copyTo(tmp2);
  Mat tmp3 = img(Rect(0,960,640,480));
  img3.copyTo(tmp3);
  namedWindow("Image");
  imshow("Image", img.t());
  waitKey(0);
*/
  return 0;
}

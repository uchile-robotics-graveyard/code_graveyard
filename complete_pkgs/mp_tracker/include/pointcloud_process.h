#ifndef _POINTCLOUD_H
#define _POINTCLOUD_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#if 0
#include <icl_robot/depth_connected_components.h>
#else
#include "depth_connected_components.h"
#endif
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <opencv2/opencv.hpp>

using namespace pcl;
using namespace std;
using namespace Eigen;
using namespace cv;

namespace icl {

void remove_big_lines(typename PointCloud<PointXYZ>::Ptr &pc);

void cluster_laser_2(typename PointCloud<PointXYZ>::Ptr &pc,
                   double eps,
                   size_t min_cluster_size,
                   size_t max_cluster_size,
                   double comfort_distance_sq,
                   vector<Vector2d, aligned_allocator<Vector2d> > &seg_means,
                   vector<Matrix2d, aligned_allocator<Matrix2d> > &seg_covs); 

void cluster_pointcloud(typename PointCloud<PointXYZ>::Ptr &pc,
                   double eps,
                   size_t min_cluster_size,
                   size_t max_cluster_size,
                   double comfort_distance_sq,
                   vector<PointIndices> &clusters,
                   vector<Vector3d, aligned_allocator<Vector3d> > &seg_means,
                   vector<Matrix3d, aligned_allocator<Matrix3d> > &seg_covs);

// Had to rewrite this function for the RGBD people dataset :(
void cluster_pointcloud_special(typename PointCloud<PointXYZRGB>::Ptr &pc,
                   double eps,
                   size_t min_cluster_size,
                   size_t max_cluster_size,
                   double comfort_distance_sq,
                   vector<PointIndices> &final_cluster,
                   vector<Vector3d, aligned_allocator<Vector3d> > &seg_means,
                   vector<Matrix3d, aligned_allocator<Matrix3d> > &seg_covs,
                   Vector4d ground_coeffs = Vector4d::Zero());

void cluster_laser(typename PointCloud<PointXYZ>::Ptr &pc,
                   double eps,
                   size_t min_cluster_size,
                   size_t max_cluster_size,
                   double comfort_distance_sq,
                   vector<Vector2d, aligned_allocator<Vector2d> > &centroids); 

void cluster_roi(typename PointCloud<PointXYZ>::Ptr &pc, const Matrix<double,3,4> &P, double eps, size_t min_cluster_size, size_t max_cluster_size, double comfort_distance_sq, bool is3D, vector<Rect> &rois);


typedef enum {
  LINEAR_FIT, QUADRATIC_FIT
} smooth_type;

void smoothPointCloudImage(PointCloud<PointXYZ>::Ptr &pc, const Matrix4f &T, int winsize, double eps, smooth_type smoothing);
void createPointcloud(const Mat &depth_image, const Mat &rgb_image, const Matrix4f &T, PointCloud<PointXYZRGB>::Ptr &output);

// Very simple downsampler
void simpleConvertVGAtoQVGA(PointCloud<PointXYZ>::Ptr &pc);
}
#endif

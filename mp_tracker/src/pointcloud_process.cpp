#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>
//#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "depth_connected_components.h"
#include "pointcloud_process.h"
#ifdef PEOPLE_HEAD_SUBCLUSTER
#include "people/head_based_subcluster.h"
#endif

using namespace pcl;
using namespace std;
using namespace Eigen;
using namespace cv;

namespace icl {

void remove_big_lines(typename PointCloud<PointXYZ>::Ptr &pc) {
  // Basic variables
  pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr line_model(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pc));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(line_model);
  ransac.setDistanceThreshold(0.05);
  const double eps = 0.2;
  do {
    // Compute a model for a line
    ransac.computeModel();
    // Extract the points in the line
    std::vector<int> inliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr line(new pcl::PointCloud<pcl::PointXYZ>);
    ransac.getInliers(inliers);
    pcl::copyPointCloud<pcl::PointXYZ>(*pc, inliers, *line);
    // Check the line is large
    search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
    kdtree->setEpsilon(eps);
//    typename search::Search<PointXYZ>::Ptr tree = boost::dynamic_pointer_cast<search::Search<PointXYZ> >(kdtree);
    kdtree->setInputCloud(line);
    // Look for connected clusters
    vector<PointIndices> clusters;
    extractEuclideanClusters(*line, boost::dynamic_pointer_cast<search::Search<PointXYZ> >(kdtree), eps, clusters, 3, 1000);
    bool line_found = false;
    for(size_t i = 0; i < clusters.size(); i++) {
      float max_x = -1E10, max_y = -1E10;
      float min_x = 1E10, min_y = 1E10;
      for(size_t j =0; j < clusters[i].indices.size(); j++) {
        min_x = std::min(min_x, line->points[clusters[i].indices[j]].x);
        min_y = std::min(min_y, line->points[clusters[i].indices[j]].y);
        max_x = std::max(max_x, line->points[clusters[i].indices[j]].x);
        max_y = std::max(max_y, line->points[clusters[i].indices[j]].y);
      }
      double dx = max_x - min_x;
      double dy = max_y - min_y;
      if(dx*dx + dy*dy > 1.0) {
        for(size_t j =0; j < clusters[i].indices.size(); j++) {
          pc->points.erase(pc->begin() + clusters[i].indices[j]);
        }
        line_found = true;
      }
    }
    if(!line_found)
      break;
    line_model->setInputCloud(pc);
  } while(true);
}

void cluster_laser_2(typename PointCloud<PointXYZ>::Ptr &pc,
                   double eps,
                   size_t min_cluster_size,
                   size_t max_cluster_size,
                   double comfort_distance_sq,
                   vector<Vector2d, aligned_allocator<Vector2d> > &seg_means,
                   vector<Matrix2d, aligned_allocator<Matrix2d> > &seg_covs) {
  // Create a KD Tree to speed up the search
  search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
  kdtree->setEpsilon(eps);
//  typename search::Search<PointXYZ>::Ptr tree = boost::dynamic_pointer_cast<search::Search<PointXYZ> >(kdtree);

  kdtree->setInputCloud(pc);

  vector<Vector2d, aligned_allocator<Vector2d> > centroids;

  // Look for connected clusters
  vector<PointIndices> clusters;
  extractEuclideanClusters(*pc, boost::dynamic_pointer_cast<search::Search<PointXYZ> >(kdtree), eps, clusters, min_cluster_size, max_cluster_size);

  // Merge close clusters
  // First, calculate the centroids of the clusters
  centroids.resize(clusters.size());
  for(size_t i = 0; i < clusters.size(); i++) {
    centroids[i] = Vector2d::Zero();
    for(size_t j = 0; j < clusters[i].indices.size(); j++) {
      size_t indx = clusters[i].indices[j];
      const PointXYZ &actual_pt = pc->points[indx];
      centroids[i] += (Vector2d() << actual_pt.x, actual_pt.y).finished();
    }
    centroids[i] /= clusters[i].indices.size();
  }
  // Now merge with the closest cluster if its below a threshold
  for(size_t i = 0; i < clusters.size(); i++) {
    double min_dist = 1000000;
    size_t min_indx = 0;
    // Look for the minumum distance
    for(size_t j = i+1; j < clusters.size(); j++) {
      if(i == j) continue;
      double dist = (centroids[i] - centroids[j]).squaredNorm();
      if(dist < min_dist) {
        min_dist = dist;
        min_indx = j;
      }
    }
    // Merge the clusters if they are close enought
    if(min_dist < comfort_distance_sq) {
      centroids[i] = (clusters[i].indices.size() * centroids[i] + clusters[min_indx].indices.size() * centroids[min_indx]);
      centroids[i] /= (double)(clusters[i].indices.size() + clusters[min_indx].indices.size());
      copy(clusters[min_indx].indices.begin(), clusters[min_indx].indices.end(), back_inserter(clusters[i].indices));
      centroids.erase(centroids.begin() + min_indx);
      clusters.erase(clusters.begin() + min_indx);
    }
  }

  // Generate the final centroids
  seg_means.resize(clusters.size());
  seg_covs.resize(clusters.size());
  for(size_t i = 0; i < clusters.size(); i++) {
    seg_means[i] = Vector2d::Zero();
    Eigen::MatrixXd xp(clusters[i].indices.size(), 2);
    for(size_t j = 0; j < clusters[i].indices.size(); j++) {
      size_t indx = clusters[i].indices[j];
      seg_means[i] += (Vector2d() << pc->points[indx].x, pc->points[indx].y).finished();
    }
    seg_means[i] /= clusters[i].indices.size();
    for(size_t j = 0; j < clusters[i].indices.size(); j++) {
      size_t indx = clusters[i].indices[j];
      xp.row(j) = (Vector2d() << pc->points[indx].x, pc->points[indx].y).finished() - seg_means[i];
    }
    seg_covs[i] = xp.transpose() * xp;
  }
}

void cluster_laser(typename PointCloud<PointXYZ>::Ptr &pc,
                   double eps,
                   size_t min_cluster_size,
                   size_t max_cluster_size,
                   double comfort_distance_sq,
                   vector<Vector2d, aligned_allocator<Vector2d> > &centroids) {
  // Create a KD Tree to speed up the search
  search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
  kdtree->setEpsilon(eps);
//  typename search::Search<PointXYZ>::Ptr tree = boost::dynamic_pointer_cast<search::Search<PointXYZ> >(kdtree);
  kdtree->setInputCloud(pc);

  // Look for connected clusters
  vector<PointIndices> clusters;
  extractEuclideanClusters(*pc, boost::dynamic_pointer_cast<search::Search<PointXYZ> >(kdtree), eps, clusters, min_cluster_size, max_cluster_size);

  // Merge close clusters
  // First, calculate the centroids of the clusters
  centroids.resize(clusters.size());
  for(size_t i = 0; i < clusters.size(); i++) {
    centroids[i] = Vector2d::Zero();
    for(size_t j = 0; j < clusters[i].indices.size(); j++) {
      size_t indx = clusters[i].indices[j];
      const PointXYZ &actual_pt = pc->points[indx];
      centroids[i] += (Vector2d() << actual_pt.x, actual_pt.y).finished();
    }
    centroids[i] /= clusters[i].indices.size();
  }
  // Now merge with the closest cluster if its below a threshold
  for(size_t i = 0; i < clusters.size(); i++) {
    double min_dist = 1000000;
    size_t min_indx = 0;
    // Look for the minumum distance
    for(size_t j = i+1; j < clusters.size(); j++) {
      if(i == j) continue;
      double dist = (centroids[i] - centroids[j]).squaredNorm();
      if(dist < min_dist) {
        min_dist = dist;
        min_indx = j;
      }
    }
    // Merge the clusters if they are close enought
    if(min_dist < comfort_distance_sq) {
      centroids[i] = (clusters[i].indices.size() * centroids[i] + clusters[min_indx].indices.size() * centroids[min_indx]);
      centroids[i] /= (double)(clusters[i].indices.size() + clusters[min_indx].indices.size());
      copy(clusters[min_indx].indices.begin(), clusters[min_indx].indices.end(), back_inserter(clusters[i].indices));
      centroids.erase(centroids.begin() + min_indx);
      clusters.erase(clusters.begin() + min_indx);
    }
  }

  // Split
  vector<PointIndices> final_cluster;
  for(size_t i = 0; i < clusters.size(); i++) {
    while(clusters[i].indices.size() > min_cluster_size) {
      PointXYZ min_pt = pc->points[clusters[i].indices[0]];
      // Split into clusters of radius at most the comfort distance
      PointIndices new_subcluster;
      for(size_t j = 0; j < clusters[i].indices.size(); j++) {
        size_t indx = clusters[i].indices[j];
        const PointXYZ &actual_pt = pc->points[indx];
        // Check the distance on the ground (excluding the up component)
        const double dx = actual_pt.x - min_pt.x;
        const double dy = actual_pt.y - min_pt.y;
        // Use square distance to avoid SQRT
        if((dx * dx + dy * dy) < comfort_distance_sq) {
          new_subcluster.indices.push_back(indx);
          clusters[i].indices.erase(clusters[i].indices.begin() + j);
          j--;
        }
      }
      // If the new subcluster is big enough
      if(new_subcluster.indices.size() > min_cluster_size)
        final_cluster.push_back(new_subcluster);
    }
  }

  // Generate the final centroids
  centroids.resize(final_cluster.size());
  for(size_t i = 0; i < final_cluster.size(); i++) {
      centroids[i] = Vector2d::Zero();
      for(size_t j = 0; j < final_cluster[i].indices.size(); j++) {
        size_t indx = final_cluster[i].indices[j];
        centroids[i] += (Vector2d() << pc->points[indx].x, pc->points[indx].y).finished();
      }
      centroids[i] /= final_cluster[i].indices.size();
  }
}

void cluster_pointcloud(typename PointCloud<PointXYZ>::Ptr &pc,
                   double eps,
                   size_t min_cluster_size,
                   size_t max_cluster_size,
                   double comfort_distance_sq,
                   vector<PointIndices> &final_cluster,
                   vector<Vector3d, aligned_allocator<Vector3d> > &seg_means,
                   vector<Matrix3d, aligned_allocator<Matrix3d> > &seg_covs) {
#if 0
  // Create a KD Tree to speed up the search
  typename KdTree<PointXYZ>::Ptr tree;
  if(pc->width != 1 && pc->height != 1) {
    tree.reset(new OrganizedDataIndex<PointXYZ>);
  }
  else {
    tree.reset(new KdTreeFLANN<PointXYZ>);
  }
  tree->setEpsilon(0.1);
  tree->setInputCloud(pc);

  // Look for connected clusters
  vector<PointIndices> clusters;
  extractEuclideanClusters(*pc, tree, eps, clusters, min_cluster_size, max_cluster_size);
# else
  // This is way faster than using OrganizedDataIndex
  vector<PointIndices> clusters;
  extractEuclideanClustersFromImage(*pc, clusters, (float)eps, min_cluster_size, max_cluster_size);
#endif
  // Merge close clusters
  // First, calculate the centroids of the clusters
  vector<Vector3d, aligned_allocator<Vector3d> > centroids(clusters.size());
  for(size_t i = 0; i < clusters.size(); i++) {
    centroids[i] = Vector3d::Zero();
    for(size_t j = 0; j < clusters[i].indices.size(); j++) {
      size_t indx = clusters[i].indices[j];
      const PointXYZ &actual_pt = pc->points[indx];
      centroids[i] += actual_pt.getVector3fMap().cast<double>();
    }
    centroids[i] /= clusters[i].indices.size();
  }
  // Now merge with the closest cluster if its below a threshold
  size_t valid_clusters = clusters.size();
  for(size_t i = 0; i < valid_clusters; i++) {
    double min_dist = 10000;
    size_t min_indx = 0;
    // Look for the minumum distance
    for(size_t j = i+1; j < valid_clusters; j++) {
      double dist = (centroids[i] - centroids[j]).squaredNorm();
      if(dist < min_dist) {
        min_dist = dist;
        min_indx = j;
      }
    }
    if(min_dist < comfort_distance_sq) {
      centroids[i] = (clusters[i].indices.size() * centroids[i] + clusters[min_indx].indices.size() * centroids[min_indx]);
      centroids[i] /= (double)(clusters[i].indices.size() + clusters[min_indx].indices.size());
      copy(clusters[min_indx].indices.begin(), clusters[min_indx].indices.end(), back_inserter(clusters[i].indices));
      centroids[min_indx] =  centroids[valid_clusters-1];
      clusters[min_indx] = clusters[valid_clusters-1];
      --valid_clusters;
//      centroids.erase(centroids.begin() + min_indx);
//      clusters.erase(clusters.begin() + min_indx);
    }
  }

  // Split
  for(size_t i = 0; i < clusters.size(); i++) {
    size_t valid_points = clusters[i].indices.size();
    while(valid_points > min_cluster_size) {
      double max_y = -1000.0;
      PointXYZ max_pt;
      // If the point type represent a 3D point, find the topmost point
      // using kinect the top most is the lower z value
      for(size_t j = 0; j < valid_points; j++) {
        if(pc->points[clusters[i].indices[j]].y > max_y) {
          max_pt = pc->points[clusters[i].indices[j]];
          max_y = max_pt.y;
        }
      }
      // Split into clusters of radius at most the comfort distance
      PointIndices new_subcluster;
      for(size_t j = 0; j < valid_points; j++) {
        size_t indx = clusters[i].indices[j];
        const PointXYZ &actual_pt = pc->points[indx];
        // Check the distance on the ground (excluding y component)
        const double dx = actual_pt.x - max_pt.x;
        const double dz = actual_pt.z - max_pt.z;
        // Use square distance to avoid SQRT
        if((dx * dx + dz * dz) < comfort_distance_sq) {
          new_subcluster.indices.push_back(indx);
          clusters[i].indices[j] = clusters[i].indices[valid_points-1];
          --valid_points;
//          clusters[i].indices.erase(clusters[i].indices.begin() + j);
          j--;
        }
      }
      clusters[i].indices.resize(valid_points);
      // If the new subcluster is big enough
      if(new_subcluster.indices.size() > min_cluster_size)
        final_cluster.push_back(new_subcluster);
    }
  }

  // Generate the final centroids
  seg_means.resize(final_cluster.size());
  seg_covs.resize(final_cluster.size());
  for(size_t i = 0; i < final_cluster.size(); i++) {
      seg_means[i] = Vector3d::Zero();
      Eigen::Matrix<double, 3, Eigen::Dynamic> xp(3, final_cluster[i].indices.size());
      for(size_t j = 0; j < final_cluster[i].indices.size(); j++) {
        const size_t &indx = final_cluster[i].indices[j];
        seg_means[i] += pc->points[indx].getVector3fMap().cast<double>();
      }
      seg_means[i] /= final_cluster[i].indices.size();
      for(size_t j = 0; j < final_cluster[i].indices.size(); j++) {
        const size_t &indx = final_cluster[i].indices[j];
        xp.col(j) = pc->points[indx].getVector3fMap().cast<double>() - seg_means[i];
      }
      seg_covs[i] = xp * xp.transpose() / (final_cluster[i].indices.size() - 1.0);
  }
}

void cluster_pointcloud_special(typename PointCloud<PointXYZRGB>::Ptr &pc,
                   double eps,
                   size_t min_cluster_size,
                   size_t max_cluster_size,
                   double comfort_distance_sq,
                   vector<PointIndices> &final_cluster,
                   vector<Vector3d, aligned_allocator<Vector3d> > &seg_means,
                   vector<Matrix3d, aligned_allocator<Matrix3d> > &seg_covs,
                   Vector4d ground_coeffs) {
#if 0
  // Create a KD Tree to speed up the search
  vector<PointIndices> clusters;
  // Look for connected clusters
  if(pc->width != 1 && pc->height != 1) {
    // This is way faster than using OrganizedDataIndex
    icl::extractEuclideanClustersFromImage(*pc, clusters, (float)eps, min_cluster_size, max_cluster_size);
  }
  else {
    search::KdTree<PointXYZRGB>::Ptr kdtree(new search::KdTree<PointXYZRGB>);
    kdtree->setEpsilon(eps);
//    typename search::Search<PointXYZRGB>::Ptr tree = boost::dynamic_pointer_cast<search::Search<PointXYZRGB> >(kdtree);
    kdtree->setInputCloud(pc);
    extractEuclideanClusters(*pc, boost::dynamic_pointer_cast<search::Search<PointXYZRGB> >(kdtree), eps, clusters, min_cluster_size, max_cluster_size);
  }
# else
  // This is way faster than using OrganizedDataIndex
  vector<PointIndices> clusters;
  icl::extractEuclideanClustersFromImage(*pc, clusters, eps, min_cluster_size, max_cluster_size);
#endif
#ifdef PEOPLE_HEAD_SUBCLUSTER
  // Head based sub-clustering
  vector<people::PersonCluster<PointXYZRGB> > people_clusters;
  people::HeadBasedSubclustering<PointXYZRGB> subclustering;
  subclustering.setInputCloud(pc);
  Eigen::VectorXf gnd = ground_coeffs.cast<float>();
  subclustering.setGround(gnd);
  subclustering.setInitialClusters(clusters);
  subclustering.setHeightLimits(1.3, 2.3);
  subclustering.setMinimumDistanceBetweenHeads(std::sqrt(comfort_distance_sq));
  subclustering.setSensorPortraitOrientation(true);
  subclustering.subcluster(people_clusters);

  // Create the needed structures
  final_cluster.resize(people_clusters.size());
  for(size_t i = 0; i < people_clusters.size(); i++) {
    final_cluster[i] = people_clusters[i].points_indices_;
  }
#else
  // Merge close clusters
  // First, calculate the centroids of the clusters
  vector<Vector3d, aligned_allocator<Vector3d> > centroids(clusters.size());
  for(size_t i = 0; i < clusters.size(); i++) {
    centroids[i] = Vector3d::Zero();
    for(size_t j = 0; j < clusters[i].indices.size(); j++) {
      size_t indx = clusters[i].indices[j];
      const PointXYZRGB &actual_pt = pc->points[indx];
      centroids[i] += actual_pt.getVector3fMap().cast<double>();
    }
    centroids[i] /= clusters[i].indices.size();
  }
  // Now merge with the closest cluster if its below a threshold
  size_t valid_clusters = clusters.size();
  for(size_t i = 0; i < valid_clusters; i++) {
    double min_dist = 10000;
    size_t min_indx = 0;
    // Look for the minumum distance
    for(size_t j = i+1; j < valid_clusters; j++) {
      double dist = (centroids[i].block<2,1>(1,0) - centroids[j].block<2,1>(1,0)).squaredNorm();
      if(dist < min_dist) {
        min_dist = dist;
        min_indx = j;
      }
    }
    if(min_dist < comfort_distance_sq) {
      centroids[i] = (clusters[i].indices.size() * centroids[i] + clusters[min_indx].indices.size() * centroids[min_indx]);
      centroids[i] /= (double)(clusters[i].indices.size() + clusters[min_indx].indices.size());
      copy(clusters[min_indx].indices.begin(), clusters[min_indx].indices.end(), back_inserter(clusters[i].indices));
      centroids[min_indx] =  centroids[valid_clusters-1];
      clusters[min_indx] = clusters[valid_clusters-1];
      --valid_clusters;
    }
  }

  // Split
  for(size_t i = 0; i < valid_clusters; i++) {
    size_t valid_points = clusters[i].indices.size();
    while(valid_points > min_cluster_size) {
      double min_x = 1000.0;
      double max_x = -1000.0;
      PointXYZRGB max_pt;
      // If the point type represent a 3D point, find the topmost point
      // using kinect the top most is the lower z value
      for(size_t j = 0; j < valid_points; j++) {
        if(pc->points[clusters[i].indices[j]].x > max_x) {
          max_pt = pc->points[clusters[i].indices[j]];
          max_x = max_pt.x;
        }
        min_x = std::min(min_x, (double)pc->points[clusters[i].indices[j]].x);
      }
      // Filter out clusters too short or too tall
      if(max_x > 0.8 || max_x < -0.4 || min_x > -0.2 || max_x - min_x < 1.1)
        break;
      // Split into clusters of radius at most the comfort distance
      PointIndices new_subcluster;
      for(size_t j = 0; j < valid_points; j++) {
        size_t indx = clusters[i].indices[j];
        const PointXYZRGB &actual_pt = pc->points[indx];
        // Check the distance on the ground (excluding y component)
        const double dy = actual_pt.y - max_pt.y;
        const double dz = actual_pt.z - max_pt.z;
        // Use square distance to avoid SQRT
        if((dy * dy + dz * dz) < comfort_distance_sq) {
          new_subcluster.indices.push_back(indx);
          clusters[i].indices[j] = clusters[i].indices[valid_points-1];
          --valid_points;
          j--;
        }
      }
      clusters[i].indices.resize(valid_points);
      // If the new subcluster is big enough
      if(new_subcluster.indices.size() > min_cluster_size)
        final_cluster.push_back(new_subcluster);
    }
  }
#endif

  // Generate the final centroids
  seg_means.resize(final_cluster.size());
  seg_covs.resize(final_cluster.size());
  for(size_t i = 0; i < final_cluster.size(); i++) {
      seg_means[i] = Vector3d::Zero();
      Eigen::Matrix<double, 3, Eigen::Dynamic> xp(3, final_cluster[i].indices.size());
      for(size_t j = 0; j < final_cluster[i].indices.size(); j++) {
        const size_t &indx = final_cluster[i].indices[j];
        seg_means[i] += pc->points[indx].getVector3fMap().cast<double>();
      }
      seg_means[i] /= final_cluster[i].indices.size();
      for(size_t j = 0; j < final_cluster[i].indices.size(); j++) {
        const size_t &indx = final_cluster[i].indices[j];
        xp.col(j) = pc->points[indx].getVector3fMap().cast<double>() - seg_means[i];
      }
      seg_covs[i] = xp * xp.transpose();
  }
}

void cluster_roi(typename PointCloud<PointXYZ>::Ptr &pc, const Matrix<double,3,4> &P, double eps, size_t min_cluster_size, size_t max_cluster_size, double comfort_distance_sq, bool is3D, vector<Rect> &rois) {
  // Create a KD Tree to speed up the search
  search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
  kdtree->setEpsilon(eps);
//  typename search::Search<PointXYZ>::Ptr tree = boost::dynamic_pointer_cast<search::Search<PointXYZ> >(kdtree);
  kdtree->setInputCloud(pc);

  // Look for connected clusters
  vector<PointIndices> clusters;
  extractEuclideanClusters(*pc, boost::dynamic_pointer_cast<search::Search<PointXYZ> >(kdtree), eps, clusters, min_cluster_size, max_cluster_size);

  // Merge close clusters
  // First, calculate the centroids of the clusters
  vector<VectorXd> centroids(clusters.size());
  for(size_t i = 0; i < clusters.size(); i++) {
    centroids[i] = VectorXd::Zero(3);
    for(size_t j = 0; j < clusters[i].indices.size(); j++) {
      size_t indx = clusters[i].indices[j];
      const PointXYZ &actual_pt = pc->points[indx];
      centroids[i] += (Vector3d() << actual_pt.x, (is3D ? actual_pt.z : actual_pt.y), 0.0).finished();
    }
    centroids[i] /= clusters[i].indices.size();
  }
  // Now merge with the closest cluster if its below a threshold
  for(size_t i = 0; i < clusters.size(); i++) {
    double min_dist = 10000;
    size_t min_indx = 0;
    // Look for the minumum distance
    for(size_t j = i+1; j < clusters.size(); j++) {
      if(i == j) continue;
      double dist = (centroids[i] - centroids[j]).squaredNorm();
      if(dist < min_dist) {
        min_dist = dist;
        min_indx = j;
      }
    }
    if(min_dist < comfort_distance_sq) {
      centroids[i] = (clusters[i].indices.size() * centroids[i] + clusters[min_indx].indices.size() * centroids[min_indx]);
      centroids[i] /= (double)(clusters[i].indices.size() + clusters[min_indx].indices.size());
      copy(clusters[min_indx].indices.begin(), clusters[min_indx].indices.end(), back_inserter(clusters[i].indices));
      centroids.erase(centroids.begin() + min_indx);
      clusters.erase(clusters.begin() + min_indx);
    }
  }

  // Split
  vector<PointIndices> final_cluster;
  if(is3D) {
    for(size_t i = 0; i < clusters.size(); i++) {
      while(clusters[i].indices.size() > min_cluster_size) {
        double min_y = 1000.0;
        PointXYZ min_pt;
        // If the point type represent a 3D point, find the topmost point
        // using kinect the top most is the lower y value
        if(is3D) {
          for(size_t j = 0; j < clusters[i].indices.size(); j++) {
            if(pc->points[clusters[i].indices[j]].y < min_y) {
              min_pt = pc->points[clusters[i].indices[j]];
              min_y = min_pt.y;
            }
          }
        } else {
          min_pt = pc->points[clusters[i].indices[0]];
        }
        // Split into clusters of radius at most the comfort distance
        PointIndices new_subcluster;
        for(size_t j = 0; j < clusters[i].indices.size(); j++) {
          size_t indx = clusters[i].indices[j];
          const PointXYZ &actual_pt = pc->points[indx];
          // Check the distance on the ground (excluding y component)
          const double dx = actual_pt.x - min_pt.x;
          const double dz = is3D ? actual_pt.z - min_pt.z : actual_pt.y - min_pt.y ;
          // Use square distance to avoid SQRT
          if((dx * dx + dz * dz) < comfort_distance_sq) {
            new_subcluster.indices.push_back(indx);
            clusters[i].indices.erase(clusters[i].indices.begin() + j);
            j--;
          }
        }
        // If the new subcluster is big enough
        if(new_subcluster.indices.size() > min_cluster_size)
          final_cluster.push_back(new_subcluster);
      }
    }
  } else {
    copy(clusters.begin(), clusters.end(), back_inserter(final_cluster));
  }

  // Calculate the bounding box of the subclusters
  rois.reserve(final_cluster.size());
  if(is3D) {
    // For 3D points, estimate the bounding box
    for(size_t i = 0; i < final_cluster.size(); i++) {
      vector<Point> points;
      // Project all the points into thei mage space
      for(size_t j = 0; j < final_cluster[i].indices.size(); j++) {
        const PointXYZ &actual_pt = pc->points[final_cluster[i].indices[j]];
        Vector4d point = (Vector4d() << actual_pt.x, actual_pt.y, actual_pt.z , 1.0).finished();
        Vector3d pixel = P * point; pixel /= pixel[2];
        points.push_back(Point(pixel[0], pixel[1]));
      }
      // Get the bouding box of the points
      rois.push_back(boundingRect(points));
    }
  } else {
    // For 2D estimate a bounding box between the feet and the head
    for(size_t i = 0; i < final_cluster.size(); i++) {
      vector<Point> points;
      // Project all the points into the image space
      for(size_t j = 0; j < final_cluster[i].indices.size(); j++) {
        const PointXYZ &actual_pt = pc->points[final_cluster[i].indices[j]];
        Vector4d point1 = (Vector4d() << actual_pt.x, actual_pt.y, -actual_pt.z , 1.0).finished();
        Vector3d pixel1 = P * point1; pixel1 /= pixel1[2];
        Vector4d point2 = (Vector4d() << actual_pt.x, actual_pt.y, 2.0 - actual_pt.z , 1.0).finished();
        Vector3d pixel2 = P * point2; pixel2 /= pixel2[2];
        points.push_back(Point(pixel1[0], pixel1[1]));
        points.push_back(Point(pixel2[0], pixel2[1]));
      }
      // Get the bouding box of the points
      rois.push_back(boundingRect(points));
    }
  }
}

// Fix the data from the images
static unsigned short fix(unsigned short old) {
  unsigned char *px = (unsigned char *)(void*)&old;
  unsigned char py[2];
  py[0] = px[1];
  py[1] = px[0];
  unsigned short *pz = (unsigned short *)(void*)py;
  return *pz;
}

// Use correct principal point from calibration
const float center_x = 3.3930780975300314e+02;
const float center_y = 2.4273913761751615e+02;
const float constant_x = 1.0 / 5.9421434211923247e+02;
const float constant_y = 1.0 / 5.9104053696870778e+02;

struct RGBValue {
  union {
    union {
      struct {
        unsigned char r,g,b,a;
      };
      float float_value;
    };
    uint32_t int_value;
  };
};

// Create a pointcloud from an image
void createPointcloud(const Mat &depth_image, const Mat &rgb_image, const Matrix4f &T, PointCloud<PointXYZRGB>::Ptr &output) {  
  // Allocate the pointcloud
  output->height = depth_image.cols;
  output->width = depth_image.rows; 
  output->points.resize(output->width * output->height);
  output->is_dense = false;

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  PointCloud<PointXYZRGB>::iterator pt_iter = output->begin();
  for (int u = 0; u < (int)depth_image.cols; ++u) {   
    for (int v = 0; v < (int)depth_image.rows; ++v) {
      pcl::PointXYZRGB& pt = *pt_iter++;
      unsigned short depth = fix(depth_image.at<unsigned short>(v,u));

      Vec3b color = rgb_image.at<Vec3b>(v,u);
      RGBValue color_;
      color_.r = color[0];
      color_.g = color[1];
      color_.b = color[2];
      color_.a = 0;
      pt.rgba = color_.int_value;
      // Missing points denoted by NaNs
//      double depth_ = 8.0 * 0.075 * 5.9421434211923247e+02 / (1084 - depth);
      double depth_ = 0.1236 * tan(depth / 2842.5 + 1.1863);
      if (depth == 0 || depth > 1092 /*|| depth_ > 7.0 || depth < 1.0*/) {
        pt.x = pt.y = pt.z = bad_point;
        continue;
      }   
      // Fill in XYZ
      Vector4f ept = T * (Vector4f() << (u - center_x) * depth_ * constant_x, (v - center_y) * depth_ * constant_y, depth_, 1.0).finished();
      pt.x = ept(0);
      pt.y = ept(1);
      pt.z = ept(2);
    }
  }
}

static double diff_depth(double z) {
  const double dzdd = (0.12361 + z*z / 0.12361) / 2842.5;
  const double ref = (0.1236 / 2842.5  * (1.0 + pow(tan(250 / 2842.5 + 1.1863), 2)));
  return dzdd / ref;
}

void smoothPointCloudImage(PointCloud<PointXYZ>::Ptr &pc, const Matrix4f &T, int winsize, double eps, smooth_type smoothing) {
  assert(pc->isOrganized());
  vector<PointXYZ> points(pc->points.size());
  const size_t dim_estimator = smoothing == LINEAR_FIT ? 4 : 10;
  float bad_point = std::numeric_limits<float>::quiet_NaN();
#pragma omp parallel for
  for(int indx = 0; indx < pc->size(); ++indx) {
    MatrixXd A = MatrixXd::Zero(4*winsize*winsize, dim_estimator);
    int u = indx % pc->width;
    int v = indx / pc->width;
    size_t valid = 0;
    PointXYZ &cpt = pc->points[u + v*pc->width];
    PointXYZ &minept = points[u + v*pc->width];
    if(!isFinite(cpt)) {
      continue;
    }
    for(int i = -winsize; i < winsize; ++i) {
      for(int j = -winsize; j < winsize; ++j) {
        if(   u+i >= 0
           && v+j >= 0
           && v+i < (int)pc->height
           && u+j < (int)pc->width) {
          const PointXYZ &pt = pc->points[u+i + (v+j)*pc->width];
          const double dx = cpt.x - pt.x;
          const double dy = cpt.y - pt.y;
          const double dz = cpt.z - pt.z;
          if(!isFinite(pt) || sqrt(dx*dx + dy*dy + dz*dz / diff_depth(cpt.z)) > eps) {
            continue;
          }
          const double &x = pt.x;
          const double &y = pt.y;
          const double &z = pt.z;
          if(smoothing == LINEAR_FIT) {
            A.block(valid, 0, 1, 4) << x, y, z, 1.0;
          }
          else {
            A.block(valid, 0, 1, 10) << x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, 1.0;
          }
          valid++;
        }
      }
    }
    if(valid > dim_estimator - 1) {
      MatrixXd Ap = A.block(0,0,valid,dim_estimator).transpose() * A.block(0,0,valid,dim_estimator);
      SelfAdjointEigenSolver<MatrixXd> eig(Ap);
      // Find the eigenvector wiht lower absolute eigenvealue
      size_t s = 0;
      double best = fabs(eig.eigenvalues()(s));
      for(int i = 1; i < eig.eigenvalues().rows(); i++) {
        if(best > fabs(eig.eigenvalues()(i))) {
          best = fabs(eig.eigenvalues()(i));
          s = i;
        }
      }
      VectorXd sol = eig.eigenvectors().col(s);
      // Intersect a ray from the camera with the estimatd surface
//      Vector3d dir = (T.cast<double>() * (Vector4d() << (v - center_x) * constant_x,
//                                                        (u - center_y) * constant_y,
//                                                        1.0, 1.0).finished()).block<3,1>(0,0);
      Vector3d dir = (Vector3d() << cpt.x, cpt.y, cpt.z).finished();
      dir.normalize();
      if(smoothing == LINEAR_FIT) {
        const Vector3d abc = sol.block<3,1>(0,0);
        double d = sol(3);
        double t = -d / dir.dot(abc);
        if(t > 0) {
          minept.x = t*dir(0);
          minept.y = t*dir(1);
          minept.z = t*dir(2);
        }
        else {
          minept.x = minept.y = minept.z = bad_point;
        }
      }
      else {
        Matrix3d Q;
        Vector3d l;
        Q <<       sol(0), sol(3) / 2.0, sol(4) / 2.0,
             sol(3) / 2.0,       sol(1), sol(5) / 2.0,
             sol(4) / 2.0, sol(5) / 2.0,       sol(2);
        l = sol.block<3,1>(6,0);
        const double a = dir.transpose() * Q * dir;
        const double b = l.transpose() * dir;
        const double c = sol(9);
        if(b*b >= 4*a*c) {
          double t0 = (-b + sqrt(b*b - 4*a*c)) / (2*a);
          double t1 = (-b - sqrt(b*b - 4*a*c)) / (2*a);
          double t = (std::min(t0, t1) > 0.0 ? std::min(t0, t1) : std::max(t0, t1));
          if(t > 0.0) {
            minept.x = t*dir(0);
            minept.y = t*dir(1);
            minept.z = t*dir(2);
          }
          else {
            minept.x = minept.y = minept.z = bad_point;
          }
        }
        else {
          minept.x = minept.y = minept.z = bad_point;
        }
      }
    }
    else {
      minept.x = minept.y = minept.z = bad_point;
    }
  }

  // Finally copy all points to the orignal pc
  size_t i = 0;
  for(size_t v = 0; v < pc->height; ++v) {
    for(size_t u = 0; u < pc->width; ++u) {
      pc->points[i] = points[i];
      ++i;
    }
  }
}

void simpleConvertVGAtoQVGA(PointCloud<PointXYZ>::Ptr &pc) {
  static std::vector<size_t> downsampler(0);
  if(downsampler.size() == 0) {
    downsampler.resize(320*240);
    for(size_t j = 0; j < 240; j++) {
      for(size_t i = 0; i < 320; i++) {
        downsampler[j*320 + i] = j * 2 * 640 + (i % 2) * (640 + 2) + (i / 2) * 4;
      }
    }
  }

  for(size_t i = 0; i < downsampler.size(); i++) {
    pc->points[i] = pc->points[downsampler[i]];
  }
  pc->width = 320;
  pc->height = 240;
}

}

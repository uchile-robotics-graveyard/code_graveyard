#ifndef UTILITIES_H
#define UTILITIES_H



#include <functional>
#include <limits>
// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Disjoint set from boost
#include <boost/pending/disjoint_sets.hpp>
// People message
#include "icl_robot/People.h"


//pull in floor detector class for depth images
#include "icl_robot/floor_detector.h"

//pull in normal estimation class for depth images
#include "icl_robot/normal_estimation.h"

//pull in plane filter
#include "icl_robot/plane_filter.h"


// Create a pointcloud from a people message
pcl::PointCloud< pcl::PointXYZ > pcFromPpl( const icl_robot::People &people_msg );

// Find the closest point in a point cloud to a particular point
std::size_t findClosestPt( const pcl::PointCloud< pcl::PointXYZ > &pc, const pcl::PointXYZ &pt );

//return the index of a point found in a point cloud
std::size_t findPtIdx( const pcl::PointCloud< pcl::PointXYZ > &pc, const pcl::PointXYZ &pt );


//function object for comparing points
struct pclPointXYZFind : public std::unary_function< pcl::PointXYZ, bool >
{
  

public:

  
  pclPointXYZFind( const pcl::PointXYZ &a );

  bool operator() ( const pcl::PointXYZ &b ) const;


private:

  
  pcl::PointXYZ a;


};


bool biggerVec( const std::vector< cv::Point > &a, const std::vector< cv::Point > &b );

bool pclPointXYZEq( const pcl::PointXYZ &a, const pcl::PointXYZ &b );

bool pclPointXYZLt( const pcl::PointXYZ &a, const pcl::PointXYZ &b );
bool pclPointXYZGt( const pcl::PointXYZ &a, const pcl::PointXYZ &b );

bool pclPointXYZLtX( const pcl::PointXYZ &a, const pcl::PointXYZ &b );
bool pclPointXYZGtX( const pcl::PointXYZ &a, const pcl::PointXYZ &b );

bool pclPointXYZLtY( const pcl::PointXYZ &a, const pcl::PointXYZ &b );
bool pclPointXYZGtY( const pcl::PointXYZ &a, const pcl::PointXYZ &b );

bool pclPointXYZLtZ( const pcl::PointXYZ &a, const pcl::PointXYZ &b );
bool pclPointXYZGtZ( const pcl::PointXYZ &a, const pcl::PointXYZ &b );


// Do connected comonets
cv::Mat depth_connected_components(const cv::Mat &);
void depth_connected_components(const cv::Mat &, cv::Mat &, int, int);



#endif

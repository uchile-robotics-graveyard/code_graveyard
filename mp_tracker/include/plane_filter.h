#ifndef _PLANE_FILTER_H
#define _PLANE_FILTER_H
/**
 * Header
 *
 *
 *
 */
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace icl {
template<typename PointT> class PlaneFilter: virtual public pcl::Filter<PointT>
{
  using pcl::Filter<PointT>::filter_name_;
  using pcl::Filter<PointT>::getClassName;
  using pcl::Filter<PointT>::input_;
  using pcl::Filter<PointT>::indices_;
  //  using pcl::Filter<PointT>::filter_limit_negative_;
  //  using pcl::Filter<PointT>::filter_limit_min_;
  //  using pcl::Filter<PointT>::filter_limit_max_;
  //  using pcl::Filter<PointT>::filter_field_name_;
  public:
  typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  PlaneFilter() {}
  PlaneFilter(float a, float b, float c, float d, float delta, bool negative = false): a_(a), b_(b), c_(c), d_(d), delta_(delta), negative_(negative), keep_organised_(true) {}

  float signedDistance(const PointT &point) const;
  bool pointNearPlane(const PointT &point) const;

  void setNegative(bool negative) {negative_ = negative;}
  void setOrganised(bool k_organised) {keep_organised_ = k_organised;}

  void setEO1FloorParams();
  void setEO2FloorParams();
  void setTableFloorParams();
  void setCeilParams();

  protected:
  // Plane parameters
  float a_, b_, c_, d_, n_;
  float delta_;
  bool negative_;
  bool keep_organised_;
  void applyFilter(PointCloud &output);
};
}
#include "plane_filter.hpp"
#endif

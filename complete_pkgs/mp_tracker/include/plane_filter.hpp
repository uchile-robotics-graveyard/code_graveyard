#include <limits>

template<typename PointT> void icl::PlaneFilter<PointT>::applyFilter(PointCloud &output)
{
  unsigned int num_points = 0;
  output.points.resize(input_->points.size());
  PointT invalid;
  invalid.x = std::numeric_limits<float>::quiet_NaN();
  invalid.y = std::numeric_limits<float>::quiet_NaN();
  invalid.z = std::numeric_limits<float>::quiet_NaN();
  for(unsigned int i = 0; i < input_->points.size(); i++)
  {
    const PointT &p = input_->points[i];
    const float distance = (p.x * a_ + p.y * b_ + p.z * c_ + d_) / n_;
    if( ( negative_ && ( distance > delta_) ) || ( !negative_ && ( distance < delta_ ) ) ) {
      output.points[num_points++] = p;
    } else if( keep_organised_ ) {
      output.points[num_points++] = invalid;
    }
  }
  if( (input_->width > 1 && input_->height > 1) && keep_organised_ ) {
    output.width = input_->width;
    output.height = input_->height;
    output.is_dense = input_->is_dense;
  } else {
    output.width = 1;
    output.points.resize(num_points);
    output.height = output.points.size();
    output.is_dense = true;
  }
}


template <typename PointT>
inline float icl::PlaneFilter<PointT>::signedDistance(const PointT &point) const
{
  //calculate *signed*, not absolute, distance
  return (point.x * a_ + point.y * b_ + point.z * c_ + d_) / n_;
}


template <typename PointT>
inline bool icl::PlaneFilter<PointT>::pointNearPlane(const PointT &point) const
{
  const float distance = signedDistance(point);
  //add 10cm to delta for edge point removal - inc

  //if point is far away from the plane return false
  //make this constraint harder than above
  if( (negative_ && (distance > delta_ + 0.2f) ) || (!negative_ && (distance < delta_ - 0.2f)) )
    return false;
  else
    return true;

}


template<typename PointT> void icl::PlaneFilter<PointT>::setEO1FloorParams()
{
  a_ = -0.0049;
  b_ = 1.45;
  c_ = -0.1197;
  d_ = -0.9380;
  delta_ = 0.0;
  negative_ = false;
  keep_organised_ = true;

  n_ = sqrt(a_*a_ + b_*b_ + c_*c_);
}

template<typename PointT> void icl::PlaneFilter<PointT>::setEO2FloorParams()
{
  a_ = -0.0049;
  b_ = 1.15;  //1.0 seems to be just above feet, 1.25 seems just below knee
  c_ = -0.1197;
  d_ = -0.9380;
  delta_ = 0.0;
  negative_ = false;
  keep_organised_ = true;

  n_ = sqrt(a_*a_ + b_*b_ + c_*c_);
}

template<typename PointT> void icl::PlaneFilter<PointT>::setTableFloorParams()
{
  a_ = -0.0049;
  b_ = 2.0;  //2.25 cuts off at knee, so we want this just a little below that
  c_ = -0.1197;
  d_ = -0.9380;
  delta_ = 0.0;
  negative_ = false;
  keep_organised_ = true;

  n_ = sqrt(a_*a_ + b_*b_ + c_*c_);
}

//untested
template<typename PointT> void icl::PlaneFilter<PointT>::setCeilParams()
{
  a_ = -0.0049;
  b_ = 5.5;
  c_ = -0.1197;
  d_ = -0.9380;
  delta_ = 0.0;
  negative_ = true;
  keep_organised_ = true;

  n_ = sqrt(a_*a_ + b_*b_ + c_*c_);
}

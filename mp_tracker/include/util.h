#ifndef MP_TRACKER_UTILS
#define MP_TRACKER_UTILS
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <stdint.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_01.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "person.h"

#ifndef NOROS
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#endif

// GNU Sientific Library
#include <gsl/gsl_sf.h>

#include <sys/time.h>
#include <ctime>

#define SQRT_2PI_INV 0.3989422804014327
#ifndef LOG_SQRT_2PI_INV
#define LOG_SQRT_2PI_INV -0.9189385332046727
#endif
#define LOG_2PI  0.79817986835

namespace icl {

struct PointWithVelocity {
	PCL_ADD_POINT4D
	union {
		float velocities[4];
		struct {
			float velocity_x;
			float velocity_y;
			float velocity_z;
		};
	} EIGEN_ALIGN16;
  size_t id;
	inline Eigen::Map<Eigen::Vector3f> getVelocityVector3fMap () { return (Eigen::Vector3f::Map(velocities)); }
	inline const Eigen::Map<const Eigen::Vector3f> getVelocityVector3fMap () const { return (Eigen::Vector3f::Map(velocities)); }
	inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getVelocityVector4fMap () { return (Eigen::Vector4f::MapAligned(velocities)); }
	inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getVelocityVector4fMap () const { return (Eigen::Vector4f::MapAligned(velocities));}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename T>
T dist_rb(T range1, T angle1, T range2, T angle2) {
	return std::sqrt(range1*range1 + range2*range2 - 2*range1*range2*std::cos(angle1 - angle2));
}

template<typename Scalar>
double normal_eval(const Scalar &x, const Scalar &u, const Scalar &s2) {
  Scalar dx = x-u;
  return SQRT_2PI_INV / std::sqrt(s2) * std::exp(0.5 / s2 * dx * dx);
}

template<typename Scalar, int Rows, int Cols>
double normal_eval(const Eigen::Matrix<Scalar,Rows,1> &x, const Eigen::Matrix<Scalar,Rows,1> &u, const Eigen::Matrix<Scalar,Rows,Cols> &S) {
  Eigen::Matrix<Scalar,Rows,1> df = x - u;
  // v = S.inverse()*df or solve[S*v=df]
  Eigen::Matrix<Scalar,Rows,1> v = S.llt().solve(df);
  return  std::pow(SQRT_2PI_INV, Rows) / std::sqrt(S.determinant()) * std::exp(-0.5*df.transpose()*v);
}

//! This function evaluates a gaussian desntity function at x, with mean u and covariance S
template<typename Base1, typename Base2, typename Base3>
double log_normal_eval(const Eigen::MatrixBase<Base1> &x, const Eigen::MatrixBase<Base2> &u, const Eigen::MatrixBase<Base3> &S) {
	Eigen::VectorXd df = x - u;
  return -0.5*df.transpose()*S.inverse()*df + x.rows()*LOG_SQRT_2PI_INV - 0.5 * std::log(S.determinant());
}

//! This function evaluates a gaussian desntity function at x, with mean u and covariance S
template<typename Base>
double log_normal_eval(const Eigen::ArrayBase<Base> &x, const Eigen::ArrayBase<Base> &u, const Eigen::ArrayBase<Base> &S) {
	Eigen::ArrayXd df = x - u;
  return -0.5 * (df.square()*S.inverse() + std::log(S) + LOG_2PI).sum();
}

inline double beta_eval(const double &p, const double &a, double const &b) {
    int status;
    gsl_sf_result result;
    status = gsl_sf_beta_e(a, b, &result);
    if(status != GSL_SUCCESS) {
      throw std::runtime_error("Error calculating Beta(a,b)");
    }
    return 1.0 / result.val * std::pow(p, a - 1.0) * std::pow(1.0 - p, b - 1.0);
}

inline double log_beta_eval(const double &p, const double &a, double const &b) {
    int status;
    gsl_sf_result result;
    status = gsl_sf_lnbeta_e(a, b, &result);
    if(status != GSL_SUCCESS) {
      throw std::runtime_error("Error calculating ln(Beta(a,b))");
    }
    return (a - 1.0)*std::log(p) + (b - 1.0)*std::log(1.0 - p) - result.val;
}

struct particle {
  double weight;
  int indx;
  Eigen::Matrix<double,5,1> state;

  bool operator< (const particle &p2) const {
    return (this->weight < p2.weight);
  }
};

inline double _weight(const person_state::ptr &ps) {return ps->weight;}
inline double _weight(const person_state &ps) {return ps.weight;}
inline double _weight(const particle &p) {return p.weight;}

void sample_normal_uS(const Eigen::VectorXd &u, const Eigen::MatrixXd &S, const int &num_samples, std::vector<Eigen::VectorXd> &samples); 
void sample_from_mixture(const std::vector<person_state::ptr> &mixture, size_t num_samples, std::vector<particle> &out);
void sample_from_samples(const std::vector<particle> &in, size_t num_samples, std::vector<particle> &out);

template<class T> inline size_t _size(const std::vector<T> &v) {return v.size();}
template<typename PointT> inline size_t _size(const boost::shared_ptr<pcl::PointCloud<PointT> > &pc) { return pc->size(); }
inline size_t _size(const Eigen::VectorXd &m) {return m.rows();}

template<class T>
void pointcloud_from_laser(const T &ranges, const double &min_ang, const double &ang_step, const double &max_range, const Eigen::Matrix4d &Tl, pcl::PointCloud<pcl::PointXYZ>::Ptr &pc) {
  pc->width = 1;
  pc->is_dense = true;
  pc->points.reserve(_size(ranges));
  for(size_t i = 0; i < _size(ranges); i++) {
    const double &range = ranges[i];
    if(!std::isfinite(range) || range >= max_range || range <= 0.3)
      continue;
    // There's a 1 degree displacemente between the camera and the laser.
    // This shoudl be included in the confuiguration of the robot
    const double angle = min_ang + i*ang_step;
    Eigen::Vector4d laser_point = Tl * (Eigen::Vector4d() << range * std::cos(angle), range * std::sin(angle), 0.0, 1.0).finished();
	pcl::PointXYZ point;
    point.x = laser_point(0);
    point.y = laser_point(1);
    //pc->points.at(num_point).z = 0.05 * range + 0.30;
    //pc->points.at(num_point).z = 0.03 * range + 0.5;
    point.z = 0.3;
	pc->points.push_back(point);
  }
  pc->height = pc->points.size();
}

template<class T>
void pointcloud_from_laser_2(const T &ranges, const double &min_ang, const double &ang_step, const double &max_range, const Eigen::MatrixXd &Tr, pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const double cluster_proximity, std::vector<int> &segments, std::map<int, double> &segments_lengths, std::map<int, int> &segments_count) {
  pc->width = 1;
  pc->is_dense = true;
  pc->points.resize(_size(ranges));
  segments.resize(_size(ranges));
  std::fill(segments.begin(), segments.end(), -1);
  size_t num_point = 0;
  int previous_indx = 0;
  double previous_range = ranges[0];
  double previous_angle = min_ang;
  double start_segment_range = ranges[0];
  double start_segment_angle = min_ang;
#ifdef DEBUG
std::cout << "laser=[" << std::endl;
#endif
  for(size_t i = 0; i < _size(ranges); i++) {
    const double &range = ranges[i];
    if(range >= max_range || range <= 0.3)
      continue;
    const double angle = min_ang + i*ang_step;
    Eigen::Vector4d laser_point = Tr * (Eigen::Vector4d() << range * std::cos(angle), range * std::sin(angle), 0.0, 1.0).finished();
    pc->points.at(num_point).x = laser_point(0);
    pc->points.at(num_point).y = laser_point(1);
    pc->points.at(num_point).z = 0.0;
    if(num_point == 0) {
      segments[0] = 0;
      start_segment_range = range;
      start_segment_angle = angle;
      segments_lengths[0] = -1;
      segments_count[0] = 1;
    } else {
      if(dist_rb(range, angle, previous_range, previous_angle) < range*cluster_proximity) {
        segments[num_point] = segments[num_point - 1];
        double max_dist = dist_rb(range, angle, start_segment_range, start_segment_angle);
        segments_count[segments[num_point]]++;
        if(max_dist > segments_lengths[segments[num_point]])
          segments_lengths[segments[num_point]] = max_dist;
       } else {
        segments[num_point] = 1 + segments[num_point - 1];
        segments_lengths[segments[num_point]] = -1;
        start_segment_range = range;
        start_segment_angle = angle;
        segments_count[segments[num_point]] = 1;
      }
    }
    previous_indx = num_point;
    previous_range = range;
    previous_angle = angle;
#ifdef DEBUG
std::cout << "[" << laser_point(0) << ", " << laser_point(1) << ", " << segments[num_point] << "]," << std::endl;
#endif
    num_point++;
  }
#ifdef DEBUG
std::cout << "]" << std::endl;
#endif

  pc->height = num_point;
  pc->points.resize(num_point);
}

bool pseudoInverse(const Eigen::MatrixXd &a, Eigen::MatrixXd &result, double epsilon = std::numeric_limits<double>::epsilon());

#ifndef NOROS
template<typename Scalar>
void lookupTransform(const tf::Transformer &tf,
                     const std::string &target_frame,
                     const std::string source_frame,
                     Eigen::Matrix<Scalar,4,4> &transform) {
	transform = Eigen::Matrix<Scalar,4,4>::Identity();
	tf::StampedTransform trans;
	tf.lookupTransform(target_frame, source_frame, ros::Time(0), trans);
	transform.block(0,3,3,1) << trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z();
	transform.block(0,0,3,3) << trans.getBasis()[0][0], trans.getBasis()[0][1], trans.getBasis()[0][2],
				trans.getBasis()[1][0], trans.getBasis()[1][1], trans.getBasis()[1][2],
				trans.getBasis()[2][0], trans.getBasis()[2][1], trans.getBasis()[2][2];
}
#endif
}

inline Eigen::Vector3d robotPosition(const Eigen::Matrix4d &T) {
  return (Eigen::Vector3d() << T(0,3), T(1,3), std::atan2(T(0,1), T(0,0))).finished();
}

// Concepts needed for the templated get_sate implementation
template<typename PointT>
inline void resize(boost::shared_ptr<pcl::PointCloud<PointT> > &pc, size_t new_size) { pc->resize(new_size); }
template<typename D>
inline void assign(pcl::PointCloud<icl::PointWithVelocity>::Ptr &pc, size_t i, const Eigen::DenseBase<D> &val) {
  pc->points[i].x = val(0);
  pc->points[i].y = val(1);
  pc->points[i].z = 0.0;
  pc->points[i].velocity_x = val(2);
  pc->points[i].velocity_y = val(3);
  pc->points[i].velocity_z = 0.0;
}
template<typename Point>
inline double get(typename pcl::PointCloud<Point>::Ptr &pc, size_t i, size_t j) {
  if(j ==0) return (double)pc->points[i].x;
  if(j ==1) return (double)pc->points[i].y;
  if(j ==2) return (double)pc->points[i].z;
  if(j ==3) return (double)pc->points[i].velocity_x;
  if(j ==4) return (double)pc->points[i].velocity_y;
  if(j ==6) return (double)pc->points[i].velocity_z;
}
template<typename Point>
inline void set_id(typename pcl::PointCloud<Point>::Ptr &pc, size_t i, size_t id) {
  pc->points[i].id = id;
}
template<typename Point>
inline size_t get_id(typename pcl::PointCloud<Point>::Ptr &pc, size_t i) {
  return pc->points[i].id;
}
template<typename Point>
inline size_t get_as_eigen(typename pcl::PointCloud<Point>::Ptr &pc, size_t i) {
  Eigen::VectorXd ret;
  ret << pc->points[i].x, pc->points[i].y, pc->points[i].z, pc->points[i].velocity_x, pc->points[i].velocity_y, pc->points[i].velocity_z;
  return ret;
}

template<typename Type>
inline void resize(std::vector<Type> &vec, size_t new_size) { vec.resize(new_size); }
template<typename Type>
inline void assign(std::vector<Eigen::VectorXd> &vec, size_t i, const Eigen::MatrixBase<Type> &val) {
  vec[i] = val;
}
template<typename Vector>
inline double get(std::vector<Vector> &vec, size_t i, size_t j) {
  return vec[i](j);
}
template<typename Vector>
inline void set_id(std::vector<Vector> &vec, size_t i, size_t id) {
  return;
}

template<typename Vector>
inline size_t get_id(std::vector<Vector> &vec, size_t i) {
  return 0;
}
template<typename Type>
inline Eigen::VectorXd get_as_eigen(std::vector<Eigen::VectorXd> &vec, size_t i) {
  return vec[i];
}

inline uint64_t getTimeMs64() {
 struct timeval tv;
 gettimeofday(&tv, NULL);
 uint64_t ret = tv.tv_usec;
 /* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
 ret /= 1000;
 /* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
 ret += (tv.tv_sec * 1000);
 return ret;
}
#ifndef NOROS
void fill_visualisation_msg(const pcl::PointCloud<icl::PointWithVelocity>::Ptr pc, size_t previous_published, visualization_msgs::MarkerArray &mrks);
#endif

POINT_CLOUD_REGISTER_POINT_STRUCT (
		icl::PointWithVelocity,
		(float, x, x)
		(float, y, y)
		(float, z, z)
		(float, velocity_x, velocity_x)
		(float, velocity_y, velocity_y)
		(float, velocity_z, velocity_z)
		(unsigned int, id, id)
);

#endif

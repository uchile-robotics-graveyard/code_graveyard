#include "util.h"
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>

namespace icl {

/*
void lookupTransform(const tf::Transformer &tf, const std::string &target_frame, const std::string source_frame, Eigen::Matrix4d &transform) {
	transform = Eigen::Matrix4d::Identity();
	tf::StampedTransform trans;
	tf.lookupTransform(target_frame, source_frame, ros::Time(0), trans);
	transform.block(0,3,3,1) << trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z();
	transform.block(0,0,3,3) << trans.getBasis()[0][0], trans.getBasis()[0][1], trans.getBasis()[0][2],
				trans.getBasis()[1][0], trans.getBasis()[1][1], trans.getBasis()[1][2],
				trans.getBasis()[2][0], trans.getBasis()[2][1], trans.getBasis()[2][2];
}
*/

void sample_from_samples(const std::vector<particle> &in, size_t num_samples, std::vector<particle> &out) {
  boost::mt19937 rng;
  boost::uniform_01<> unf;
  boost::variate_generator<boost::mt19937, boost::uniform_01<> > var_unf(rng, unf);

  double total_weight = 0;
  for(size_t i = 0; i < in.size(); i++) {
    total_weight += in[i].weight;
  }
  for(size_t i = 0; i < num_samples; i++) {
    double sample_weight = var_unf() * total_weight;
    size_t j = 0;
    for(; j < in.size(); j++) {
      sample_weight -= in[j].weight;
      if(sample_weight <= 0)
        break;
    }
    if(j == in.size()) j--;
    out[i].state = in[j].state;
    out[i].weight = total_weight / num_samples;
  }
}

void sample_from_mixture(const std::vector<person_state::ptr> &mixture, size_t num_samples, std::vector<particle> &out) {
  boost::mt19937 rng;
  boost::uniform_01<> unf;
  boost::normal_distribution<> nor(0,1);
  boost::variate_generator<boost::mt19937, boost::uniform_01<> > var_unf(rng, unf);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > var_nor(rng, nor);

  std::vector<Eigen::MatrixXd> sqrtCovs(mixture.size());
  double total_weight = 0;
  for(size_t i = 0; i < mixture.size(); i++) {
#if 0
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(mixture[i]->posvelheight_cov);
    Eigen::MatrixXd eigvals = solver.eigenvalues().array().sqrt().matrix().asDiagonal();
    Eigen::MatrixXd eigvect = solver.eigenvectors();
    sqrtCovs.at(i) = eigvect * eigvals;
#else
    sqrtCovs.at(i) = mixture[i]->posvelheight_cov.llt().matrixL();
#endif
    total_weight += mixture[i]->weight;
  }
  out.resize(num_samples);
  for(size_t i = 0; i < num_samples; i++) {
    double sample_weight = var_unf() * total_weight;
    size_t j = 0;
    for(; j < mixture.size(); j++) {
      sample_weight -= mixture[j]->weight;
      if(sample_weight <= 0)
        break;
    }
    if(j == mixture.size()) j--;
    Eigen::Matrix<double,5,1> rand_01;
    for(size_t k = 0; k < 5; k++) {
      rand_01(k) = var_nor();
    }
    // Convert the N(0,I) to N(u,S)
    out[i].weight = total_weight / num_samples;
    out[i].state = mixture[j]->posvelheight +  sqrtCovs[j] * rand_01;
  }
}

void sample_normal_uS(const Eigen::VectorXd &u, const Eigen::MatrixXd &S, const int &num_samples, std::vector<Eigen::VectorXd> &samples) {
  boost::mt19937 rng;
  boost::normal_distribution<> nor(0,1);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > var_nor(rng, nor);
#if 0
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(S);
  Eigen::MatrixXd eigvals = solver.eigenvalues().array().sqrt().matrix().asDiagonal();
  Eigen::MatrixXd eigvect = solver.eigenvectors();
  Eigen::MatrixXd sqS = eigvect * eigvals;
#else
  Eigen::MatrixXd sqS = S.llt().matrixL();
#endif
  samples.resize(num_samples);
  for(int i = 0; i < num_samples; i++) {
    // Generate a N(0,I)
    Eigen::VectorXd rand_01(u.rows());
    for(int j = 0; j < u.rows(); j++) {
      rand_01(j) = var_nor();
    }
    // Convert the N(0,1) to N(u,S)
    samples[i] = u +  sqS * rand_01;
  }
}

}

#ifndef NOROS
void fill_visualisation_msg(const pcl::PointCloud<icl::PointWithVelocity>::Ptr pc, size_t previous_published, visualization_msgs::MarkerArray &mrks) {
  for(size_t i = 0; i < std::max(pc->size(), previous_published); i++) {
    // A sphere in the position
    visualization_msgs::Marker marker;
   // marker.header = pc->header;
    marker.header = pcl_conversions::fromPCL(pc->header);
    marker.ns = "d_tracker";
    marker.id = 3*i;
    marker.type = visualization_msgs::Marker::SPHERE;
    if(i < pc->size()) {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = pc->points[i].x;
      marker.pose.position.y = pc->points[i].y;
      marker.pose.position.z = pc->points[i].z;
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 0.8;
      marker.lifetime = ros::Duration(1./15);
    } else {
      marker.action = visualization_msgs::Marker::DELETE;
    }
    mrks.markers.push_back(marker);
    // An arrow for the speed
    visualization_msgs::Marker marker2;
   // marker2.header = pc->header;
        marker2.header = pcl_conversions::fromPCL(pc->header);
    marker2.ns = "d_tracker";
    marker2.id = 3*i+1;
    marker2.type = visualization_msgs::Marker::ARROW;
    if(i < pc->size()) {
      marker2.action = visualization_msgs::Marker::ADD;
      marker2.points.resize(2);
      marker2.points[0].x = pc->points[i].x;
      marker2.points[0].y = pc->points[i].y;
      marker2.points[0].z = pc->points[i].z;
      marker2.points[1].x = pc->points[i].x + pc->points[i].velocity_x;
      marker2.points[1].y = pc->points[i].y + pc->points[i].velocity_y;
      marker2.points[1].z = pc->points[i].z + pc->points[i].velocity_z;
      // Shaft radius
      marker2.scale.x = 0.1;
      // Head radius
      marker2.scale.y = 0.2;
      marker2.scale.z = 0.0;
      marker2.color.r = 1.0;
      marker2.color.g = 0.0;
      marker2.color.b = 1.0;
      marker2.color.a = 0.8;
      marker2.lifetime = ros::Duration(1./15);
    } else {
      marker2.action = visualization_msgs::Marker::DELETE;
    }
    mrks.markers.push_back(marker2);
    // Text for the id
    visualization_msgs::Marker marker3;
//    marker3.header = pc->header;
        marker3.header = pcl_conversions::fromPCL(pc->header);

    marker3.ns = "d_tracker";
    marker3.id = 3*i+2;
    marker3.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    if(i < pc->size()) {
      marker3.pose.position.x = pc->points[i].x + 0.3;
      marker3.pose.position.y = pc->points[i].y + 0.3;
      marker3.pose.position.z = pc->points[i].z + 0.4;
      marker3.scale.x = 0.0;
      marker3.scale.y = 0.0;
      marker3.scale.z = 0.2;
      marker3.color.r = 0.0;
      marker3.color.g = 0.0;
      marker3.color.b = 0.0;
      marker3.color.a = 0.8;
      std::stringstream ss;
      ss << "ID: " << pc->points[i].id;
      marker3.text = ss.str();
      marker3.lifetime = ros::Duration(1./15);
    } else {
      marker3.action = visualization_msgs::Marker::DELETE;
    }
    mrks.markers.push_back(marker3);
  }
}
#endif

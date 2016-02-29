/*
 * MotionModel.hpp
 *
 *  Created on: 26-10-2014
 *      Author: matias
 */

#ifndef MOTIONMODEL_HPP_
#define MOTIONMODEL_HPP_

// C++
#include <string.h>
#include <vector>
#include <math.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>


// ROS
#include "ros/ros.h"

// ROS messages
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace bender_follow_me {

const int state_length =  4;

// x, y, phi, v
typedef Eigen::Matrix<float,state_length,1> state_t;
typedef Eigen::Matrix<float,state_length,state_length> state_cov_t;

struct State {
	state_t mean;
	state_cov_t cov;
	ros::Time time;

	State() {
		clean();
	};

	void clean() {
		mean.fill(0);
		cov.fill(0);
	};

	static int length() {
		return state_length;
	};


};

class MotionModel {
public:
	MotionModel() {

		//_initialized = false;

		// motion model covariance noise
		_Q.fill(0);
		_Q(0,0) = sigma_xy*sigma_xy;
		_Q(1,1) = sigma_xy*sigma_xy;
		_Q(2,2) = sigma_phi*sigma_phi;
		_Q(3,3) = sigma_v*sigma_v;

		//ROS_WARN_STREAM("Q=[\n" << _Q << "]");
	};

	~MotionModel() {};

	// motion model noise
	static const float sigma_xy;  // [m]
	static const float sigma_z;   // [m]
	static const float sigma_phi; // [rad]
	static const float sigma_v;   // [m/s]

	State _x;
	state_cov_t _Q;
	//bool _initialized;

	void initializeState(float x, float y, ros::Time time) {

		// initialize
		_x.mean.fill(0);
		_x.cov.fill(0);
		_x.mean(0) = x;
		_x.mean(1) = y;
		_x.mean(2) = 0;
		_x.mean(3) = 0;

		this->resetUncertainty();

		//ROS_WARN_STREAM("x=[" << _x.mean.transpose() << "]");
		//ROS_WARN_STREAM("Pxx=[\n" << _x.cov << "]");

		_x.time = time;
		//ROS_WARN_STREAM("x.time=" << _x.time);
		//_initialized = true;
	};

	void resetUncertainty() {
		_x.cov(0,0) = sigma_xy*sigma_xy;
		_x.cov(1,1) = sigma_xy*sigma_xy;
		_x.cov(2,2) = sigma_phi*sigma_phi;
		_x.cov(3,3) = sigma_v*sigma_v;
	};

	static void toPoint(const state_t &x, geometry_msgs::Point &p) {
		p.x = x(0);
		p.y = x(1);
		p.z = 0;
	};

	static void predict(const state_t &x_0, state_t &x_1, const double &dt) {

		x_1.fill(0);
		x_1(0) = x_0(0) + x_0(3)*dt*cosf((float)x_0(2));
		x_1(1) = x_0(1) + x_0(3)*dt*sinf((float)x_0(2));
		x_1(2) = x_0(2);
		x_1(3) = fabsf((float)x_0(3));
	};
};

const float MotionModel::sigma_xy  = 0.1;      // [ m ]
const float MotionModel::sigma_z   = 0.1;      // [ m ]
const float MotionModel::sigma_phi = M_PI/1.8; // [rad]
const float MotionModel::sigma_v   = 0.1;      // [m/s]
} /* namespace bender_follow_me */

#endif /* MOTIONMODEL_HPP_ */

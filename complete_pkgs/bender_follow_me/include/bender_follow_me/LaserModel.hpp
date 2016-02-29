/*
 * LaserModel.hpp
 *
 *  Created on: 26-10-2014
 *      Author: matias
 */

#ifndef LASERMODEL_HPP_
#define LASERMODEL_HPP_

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

#include <bender_follow_me/MotionModel.hpp>


namespace bender_follow_me {

typedef Eigen::Vector2f observation_t;
typedef Eigen::Matrix2f observation_cov_t;

class LaserModel {
public:

	// laser model noise
	static const float sigma_x; // [m]
	static const float sigma_y; // [m]

	observation_cov_t _R;

	LaserModel() {

		// laser observation covariance noise
		_R.fill(0);
		_R(0,0) = sigma_x*sigma_x;
		_R(1,1) = sigma_y*sigma_y;

		//ROS_WARN_STREAM("R=[\n" << _R << "]");
	};

	~LaserModel() {};


	/**
	 * Laser Model
	 */
	static void generateExpectedObservation(const state_t &x, observation_t &z) {

		//Z(0) = atan2f((float)Y(1),(float)Y(0));
		//Z(1) = sqrtf((float)(Y(0)*Y(0) + Y(1)*Y(1)));
		z(0) = x(0);
		z(1) = x(1);
	};

	static void observationToState(const observation_t &z, state_t &x) {

		x.fill(0);
		x(0) = z(0);
		x(1) = z(1);
		//X(0) = Z(1)*cosf((float)Z(0));
		//X(1) = Z(1)*sinf((float)Z(0));
	};

	static void detectionToObservation(const geometry_msgs::Point32 &position, const float heading, observation_t &z) {

		z(0) = position.x;
		z(1) = position.y;
		// TODO : HEADING

		//z_real(0) = detections.bearings[0];
		//z_real(1) = detections.ranges[0];
	}

};

// laser model noise
// obs: (no es la var. del laser (que es muy baja), sino que la de los detectores)
const float LaserModel::sigma_x = 0.1; // [m]
const float LaserModel::sigma_y = 0.1; // [m]

} /* namespace bender_follow_me */

#endif /* LASERMODEL_HPP_ */

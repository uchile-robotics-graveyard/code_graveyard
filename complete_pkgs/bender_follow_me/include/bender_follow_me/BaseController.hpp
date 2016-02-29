/*
 * BaseController.hpp
 *
 *  Created on: 14-10-2014
 *      Author: matias
 */

#ifndef BASECONTROLLER_HPP_
#define BASECONTROLLER_HPP_

// C++
#include <string.h>
#include <vector>
#include <math.h>
#include <boost/scoped_ptr.hpp>

// ROS
#include "ros/ros.h"

// ROS messages
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
 #include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <bender_srvs/Transformer.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

namespace bender_follow_me {

class BaseController {
public:
	BaseController(std::string name);
	virtual ~BaseController();

	void callback_laser_scan(const sensor_msgs::LaserScan& scan);
	void callback_in_commands(const geometry_msgs::Twist &cmd);
	void sample_motion_model_velocity(const geometry_msgs::Twist &cmd, float &x, float &y);
	bool robot_collides(float x, float y, const sensor_msgs::LaserScan& scan, float &collision_x, float &collision_y);
	void makePolygonCircle(std::vector<geometry_msgs::Point32>& points, float x_center, float y_center, float radius);
	void publishVisualizationMessages(float motion_x, float motion_y, std::string scan_frame, float collision_x, float collision_y);

private:

	std::string _name;

	// publishers
	ros::Publisher _collision_pub;
	ros::Publisher _footprint_pub;
	ros::Publisher _motion_footprint_pub;
	ros::Publisher _cmd_pub;

	// subscribers
	ros::Subscriber _laser_scan_sub;
	ros::Subscriber _cmd_sub;

	float alpha_1;
	float alpha_2;
	float alpha_3;
	float alpha_4;
	float alpha_5;
	float alpha_6;
	float dt;
	float footprint_radius_squared;
	std::string _frame_in;

	geometry_msgs::Twist last_cmd;
	ros::Time last_cmd_time;
	ros::Time last_sent_command_time;

	// clients
	ros::ServiceClient _transformer_client;

};

} /* namespace bender_follow_me */


#endif /* BASECONTROLLER_HPP_ */

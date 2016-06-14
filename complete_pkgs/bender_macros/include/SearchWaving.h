/*
 * SearchWaving.h
 *
 *  Created on: Jan 20, 2014
 *      Author: matias.pavez.b@gmail.com
 */

#ifndef SEARCHWAVING_H_
#define SEARCHWAVING_H_

// C, C++
#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

// Bender
#include <bender_utils/ParameterServerWrapper.h>
#include <bender_msgs/WaveData.h>
#include <bender_srvs/SearchPerson.h>
#include <bender_srvs/NavGoal.h>


namespace bender_macros {

class SearchWaving {

private:

	std::string _name;
	boost::shared_ptr<tf::TransformListener> _tf_listener;

	// - - - - - control variables - - - - - - - -
	bender_msgs::WaveData _last_wave;
	bool _new_wave;
	unsigned int _current_body_pos_index;
	int _last_body_angle;


	// - - - - - P a r a m e t e r s - - - - - - -
	std::string _map_frame;
	std::string _rgbd_tf;
	std::vector<int> _rotation_angles;
	double _wave_timeout;


	// - - - - - - - - S e r v i c e   C l i e n t s  - - - - - - - - -
	ros::ServiceClient _rotate_client;

	// - - - - - L i s t e n e r s - - - - - - - -
	ros::Subscriber _waving_sub;

  	// - - - - -  S e r v i c e s  - - - - - - - -
	ros::ServiceServer _search_service;




public:
	SearchWaving(std::string name);
	virtual ~SearchWaving();

private:

	bool waitForWave(double timeout);
	bool getNextBodyPosition(unsigned int & angle);
	void rotateBody(int angle);
	bool transformPose(geometry_msgs::PoseStamped in_pose, geometry_msgs::PoseStamped &out_pose);


	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// - - - - - - - - - - - - - - - - S u b s c r i b e r    C a l l b a c k s  - - - - - - - - - - - - - - - - - - -
	void callback_waving(const bender_msgs::WaveData msg);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// - - - - - - - - - - - - - - - - - - - - -  S e r v i c e s  - - - - - - - - - - - - - - - - - - - - - - - - - -

	bool search(bender_srvs::SearchPerson::Request &req, bender_srvs::SearchPerson::Response &res);

};

} /* namespace bender_macros */
#endif /* SEARCHWAVING_H_ */

/*
 * SearchWaving.cpp
 *
 *  Created on: Jan 22, 2014
 *      Author: matias.pavez.b@gmail.com
 */

#include "SearchWaving.h"

// TODO: use a tf listener to transform pose to /map frame
// TODO: use speech

namespace bender_macros {

SearchWaving::SearchWaving(std::string name):_name(name) {

	ros::NodeHandle priv("~");

	// control variables
	_new_wave = false;
	_last_body_angle = 0;
	_current_body_pos_index = 0;

    // - - - - - - - P A R A M E T E R   S E R V E R - - - - - - - - -
    bender_utils::ParameterServerWrapper psw;
    psw.getParameter("rgbd_tf",_rgbd_tf,"/rgbd_waist_depth_frame");
    psw.getParameter("wave_timeout",_wave_timeout,5.0);


	// rotation angles [degrees]
    std::vector<int> default_positions;
	default_positions.push_back(35);
	default_positions.push_back(-35);
	default_positions.push_back(-35);

	// important: return to default position if not found
	default_positions.push_back(+35);

	psw.getParameter("rotation_angles",_rotation_angles, default_positions);

	// - - - - - - - - S e r v i c e   C l i e n t s  - - - - - - - - -
	_rotate_client = priv.serviceClient<bender_srvs::NavGoal>("/bender/nav/goal_server/rotate");
	while ( ros::ok() && !_rotate_client.waitForExistence(ros::Duration(3.0)) );

	// - - - - - - - - L i s t e n e r - - - - - - - - - - - - - - - - -
	_waving_sub = priv.subscribe("/bender/vision/kinect_tracker/wave_user_data", 1, &SearchWaving::callback_waving,this);

	// - - - - - - - - - - - - - - S E R V I C E S - - - - - - - - - - - - - -
	_search_service = priv.advertiseService("/bender/macros/search_waving", &SearchWaving::search,this);

	ROS_INFO_STREAM("[" << _name << "]: Running");
}

SearchWaving::~SearchWaving() {
}

void SearchWaving::callback_waving(const bender_msgs::WaveData msg) {

    _last_wave = msg;
    _new_wave = true;
}

bool SearchWaving::waitForWave(double timeout) {

	_new_wave = false;

	double sleep_time = 0.5;
	int count = 0;

	while (!_new_wave && count*sleep_time < timeout ) {

		count++;
		ros::Duration(sleep_time).sleep();
	}

	if ( count*sleep_time >= timeout) {
		return false;
	}

	return true;
}

bool SearchWaving::getNextBodyPosition(unsigned int & angle) {

	if (_current_body_pos_index >= _rotation_angles.size() ) {
		_current_body_pos_index = 0;
		return false;
	}

	angle = _rotation_angles[_current_body_pos_index];
	_current_body_pos_index++;

	return true;
}

void SearchWaving::rotateBody(int angle) {

	bender_srvs::NavGoal rotate_srv;
	rotate_srv.request.rotation = angle;

	int delta;
    delta = abs(angle - _last_body_angle);
    _last_body_angle = angle;

    ROS_INFO_STREAM("[" << _name << "]: Rotating Body in " << angle << " degrees");

    _rotate_client.call(rotate_srv);
	ros::Duration(delta*1.5/40.0).sleep();
}


bool SearchWaving::search(bender_srvs::SearchPerson::Request &req, bender_srvs::SearchPerson::Response &res) {

	unsigned int angle = 0;

	while ( getNextBodyPosition(angle) ) {

		rotateBody(angle);

		ROS_INFO_STREAM("[" << _name << "]: Looking for waving");
		if (!waitForWave(_wave_timeout)) {
			ROS_INFO_STREAM("[" << _name << "]: Recognize timeout");
			continue;
		}

		ROS_INFO_STREAM("[" << _name << "]: Waving found :) ");
		res.person.header.frame_id = _rgbd_tf;
		res.person.header.stamp = ros::Time::now();
		res.person.pose.position.x = _last_wave.Z/1000.0;
		res.person.pose.position.y = _last_wave.X/1000.0;
		res.person.pose.orientation.w = 1.0;

		res.found = true;
		return true;
	}

	ROS_INFO_STREAM("[" << _name << "]: Waving not found :'( ");
	res.found = false;
	return true;
}

} /* namespace bender_macros */


int main(int argc, char** argv) {

	ros::init(argc, argv, "search_waving");

	boost::scoped_ptr<bender_macros::SearchWaving> node(
			new bender_macros::SearchWaving(ros::this_node::getName())
	);

	ros::AsyncSpinner spinner(2);
	spinner.start();

	while(ros::ok()) {

		ros::waitForShutdown();
	}

	ROS_INFO("Quitting ... \n");
	return 0;
}

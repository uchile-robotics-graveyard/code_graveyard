/*
 * CocktailParty.h
 *
 *  Created on: January 2014
 *      Author: matias.pavez.b@gmail.com
 */

#ifndef COCKTAILPARTY_H_
#define COCKTAILPARTY_H_

// C, C++
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>

// ROS msgs
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// ROS srvs
#include <std_srvs/Empty.h>

// BENDER
#include <bender_srvs/stringReturn.h>

// Bender Arm
#include <bender_srvs/armGoal.h>

// Bender Navigation
#include <bender_msgs/SemanticObject.h>
#include <bender_srvs/SemMap.h>
#include <bender_srvs/NavGoal.h>

// Bender Speech
#include <bender_srvs/synthesize.h>

// Bender Vision
#include <bender_srvs/DoorDetector.h>
#include <bender_srvs/objrecService.h>

// Bender Macros
#include <bender_srvs/SearchPerson.h>
#include <MacroPerson.h>

// TODO: remove this stuff
#include <geometry_msgs/Twist.h>


namespace bender_behaviors {

class CocktailParty {

private:

	ros::NodeHandle priv;
	std::string _name;
	ros::Publisher _initial_pose_pub;

	// - - - - - - - - -  S u b s c r i b e r s  - - - - - - - - - - -
	// none

	// - - - - - - - - S e r v i c e   C l i e n t s  - - - - - - - - -

	// arm
	ros::ServiceClient _grasp_object_client;

	// navigation
	ros::ServiceClient _nav_sem_map_get_client;
	ros::ServiceClient _nav_go_to_client;
	ros::ServiceClient _nav_approach_client;
	ros::ServiceClient _nav_arrived_client;

	// speech
  	ros::ServiceClient _speech_synthesizer_client;
	ros::ServiceClient _request_drink_client;

  	// vision
	ros::ServiceClient _door_open_detector_client;
	ros::ServiceClient _find_order_client;


	// macros
	ros::ServiceClient _search_waving_client;
	ros::ServiceClient _enroll_person_client;


public:
	CocktailParty(std::string name);
	virtual ~CocktailParty();

	void talk(std::string sentence);

	// arm
	void graspObject(geometry_msgs::Point object_position);

	// navigation
	void setInitialPose(std::string place);
	void arenaEntrance(double walk_velocity, double walk_time);
	void goToPose(std::string place);
	void approachTo(bender_macros::MacroPerson person);

	// vision
	void waitForOpenDoor(int door_distance);
	void searchOrder(std::string order, geometry_msgs::Point &object_position);

	// macros
	void enroll(bender_macros::MacroPerson& person);
	void askOrder(bender_macros::MacroPerson& person);
	void searchWaving(bender_macros::MacroPerson& person);

};

} /* namespace bender_behaviors */
#endif /* COCKTAILPARTY_H_ */

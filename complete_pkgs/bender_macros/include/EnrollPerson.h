/*
 * EnrollPerson.h
 *
 *  Created on: 23 Jan 2014
 *      Author: gonzaloolave
 */

#ifndef ENROLLPERSON_H_
#define ENROLLPERSON_H_

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

// Bender
#include <bender_srvs/ID.h>
#include <bender_srvs/FaceInfo.h>
#include <bender_srvs/FaceRecognition.h>
#include <bender_srvs/synthesize.h>
#include <bender_srvs/load_dictionary_service.h>
#include <bender_srvs/stringService.h>
#include <bender_srvs/SearchPerson.h>
#include <bender_srvs/stringReturn.h>
#include <bender_srvs/objrecService.h>
#include <Person.h>

#include <map>
#include <utility>

namespace bender_macros {

class EnrollPerson {

private:

	std::map<int, bender_macros::Person> _person_map;
	int _id;

	// - - - c l i e n t s - - -
	ros::ServiceClient _ask_name;
	ros::ServiceClient _face;

	// - - - s e r v i c e s  - - -
	ros::ServiceServer _enroll;

public:
	EnrollPerson();
	virtual ~EnrollPerson();

private:
	// - - - s e r v i c e s 
	bool enroll_person(bender_srvs::stringReturn::Request &req,
			bender_srvs::stringReturn::Response &res);

};

}

#endif /* ENROLLPERSON_H_ */

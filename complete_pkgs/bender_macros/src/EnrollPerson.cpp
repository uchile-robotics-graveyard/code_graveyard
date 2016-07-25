/*
 * EnrollPerson.cpp
 *
 *  Created on: 23 Jan 2014
 *      Author: gonzaloolave
 */

#include "EnrollPerson.h"

namespace bender_macros {

EnrollPerson::EnrollPerson() {

	ros::NodeHandle nh("~");

	_id = 100;

	// - - - c l i e n t s - - - 
	_ask_name = nh.serviceClient<bender_srvs::stringReturn>("/interaction_speech/request_name");
	_face = nh.serviceClient<bender_srvs::ID>("/enroll_face/enroll");

	// - - - wait for services - - -
	while ( ros::ok() && !_ask_name.waitForExistence(ros::Duration(3.0)) );
	while ( ros::ok() && !_face.waitForExistence(ros::Duration(3.0)) );

	// 
	_enroll = nh.advertiseService("enroll",&EnrollPerson::enroll_person,this);
}

EnrollPerson::~EnrollPerson(){

}

bool EnrollPerson::enroll_person(bender_srvs::stringReturn::Request &req, bender_srvs::stringReturn::Response &res){

	bender_srvs::stringReturn name_srv;
	bender_srvs::ID enroll_srv;
	std::map<int, bender_macros::Person>::iterator it;

	// ask for name
	while( !_ask_name.call(name_srv) ) {

		ROS_ERROR("Failed to call service '/interaction_speech/request_name'");
		ros::Duration(1.0).sleep();
	}

	for (it = _person_map.begin(); it != _person_map.end(); it++) {

		if ( name_srv.response.data == it->second.name ) {

			// TODO: also check if the face is in the image database
			// in that case whe can declare "already known person", 
			// 'continue' otherwise

			// modify this: in this case, return ??
			//talk("i have already known you, " + name_srv.response.data);
			res.data = name_srv.response.data;

			return true;
		}
	}

 	// if the person is unknown to the robot

	// talk("nice to meet you, " + name_srv.response.data);
	res.data = name_srv.response.data;
	_person_map[++_id].name = name_srv.response.data;
	
	enroll_srv.request.ID = _id;

	while(!_face.call(enroll_srv) ) {
		ROS_ERROR("Failed to call service 'enroll_face'");
		ros::Duration(1.0).sleep();
	}

	return true;
}

} /* namespace bender_macros */ 

int main(int argc, char **argv)
{
	ros::init(argc,argv,"enroll_person");

	bender_macros::EnrollPerson * persons = new bender_macros::EnrollPerson();

	ros::spin();

	ROS_INFO("Quitting ... \n");
	return 0;

}

/*
 * Person.h
 *
 *  Created on: 24 Jan 2014
 *      Author: gonzaloolave
 */

#ifndef PERSON_H_
#define PERSON_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

namespace bender_macros {

class Person {

public:
	int id;
	std::string name;
	geometry_msgs::PoseStamped object;
};

} /* namespace bender_macros */
#endif /* PERSON_H_ */

/*
 * MacroPerson.h
 *
 *  Created on: 24 Jan 2014
 *      Author: gonzaloolave
 */

#ifndef MACROPERSON_H_
#define MACROPERSON_H_

#include <geometry_msgs/PoseStamped.h>

namespace bender_macros {

class MacroPerson {

private:
	int id;
	std::string name;
	std::string order;
	geometry_msgs::PoseStamped person_place;

public:
	MacroPerson();
	virtual ~MacroPerson();
	int getId();
	std::string getName();
	std::string getOrder();
	geometry_msgs::PoseStamped getPose();

	void setId(int id);
	void setName(std::string name);
	void setOrder(std::string order);
	void setPose(geometry_msgs::PoseStamped pose);
};

} /* namespace bender_macros */
#endif /* MACROPERSON_H_ */

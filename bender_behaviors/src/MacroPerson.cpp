/*
 * MacroPerson.cpp
 *
 *  Created on: 24 Jan 2014
 *      Author: gonzaloolave
 */

#include "MacroPerson.h"

namespace bender_macros {

MacroPerson::MacroPerson() {

}

MacroPerson::~MacroPerson() {

}

int MacroPerson::getId(){

	return this->id;
}

std::string MacroPerson::getName(){

	return this->name;
}

std::string MacroPerson::getOrder(){

	return this->order;
}

geometry_msgs::PoseStamped MacroPerson::getPose(){

	return this->person_place;
}

void MacroPerson::setId(int id) {
	this->id = id;
}

void MacroPerson::setName(std::string name) {
	this->name = name;
}

void MacroPerson::setOrder(std::string order) {
	this->order = order;
}

void MacroPerson::setPose(geometry_msgs::PoseStamped pose) {
	this->person_place = pose;
}

} /* namespace bender_macros */

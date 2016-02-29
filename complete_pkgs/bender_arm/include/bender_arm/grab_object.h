/*
 * grab_object.h
 *
 *  Created on: 22 Jan 2014
 *      Author: gonzaloolave
 */

#ifndef GRAB_OBJECT_H_
#define GRAB_OBJECT_H_

#include <ros/ros.h>
#include "bender_srvs/Dummy.h"
#include "bender_srvs/PlanningGoalCartesian.h"
#include "bender_srvs/LoadMode.h"
#include "bender_srvs/objrecService.h"
#include "bender_srvs/armGoal.h"
#include "string.h"
#include "std_msgs/String.h"
#include "bender_srvs/stringReturn.h"


ros::ServiceClient escoger;
ros::ServiceClient lpreman1,rpreman1;
ros::ServiceClient lpreman2,rpreman2;
ros::ServiceClient labrir_grip,rabrir_grip;
ros::ServiceClient lgrasp,rgrasp;
ros::ServiceClient lorientar_grip,rorientar_grip;
ros::ServiceClient lcerrar_grip,rcerrar_grip;
ros::ServiceClient lpostman1,rpostman1;

ros::ServiceServer servicio;

std::string left_port="",right_port="";
std::string selected_arm = "none";
ros::NodeHandle * n;

bool manipular_objeto_callback(bender_srvs::armGoal::Request &req, bender_srvs::armGoal::Response &res);
void fetch_detected_drink(double pos[3]);
bool select_arm();


#endif /* GRAB_OBJECT_H_ */

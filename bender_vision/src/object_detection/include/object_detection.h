/*
 * object_detection.h
 *
 *  Created on: 22 Jan 2014
 *      Author: gonzaloolave
 */

#ifndef OBJECT_DETECTION_H_
#define OBJECT_DETECTION_H_


#include <ros/ros.h>
#include "string.h"
#include "std_msgs/String.h"
#include <std_srvs/Empty.h>
#include <bender_srvs/IsOn.h>
#include "bender_srvs/ObjectDetection.h"
#include "bender_srvs/synthesize.h"
#include "bender_srvs/objrecService.h"


ros::ServiceClient sifton;
ros::ServiceClient siftoff;
ros::ServiceClient planeon;
ros::ServiceClient planeoff;
ros::ServiceClient detect_obj;
ros::ServiceClient reset_obj;

ros::ServiceClient talk_srv;
ros::Subscriber talk_sub;

ros::ServiceServer reconocedor;

std::string recognized_drink = "none";
std::string synthesizer_status;

bool reconocer_objeto_callback(bender_srvs::objrecService::Request &req,bender_srvs::objrecService::Response &res);
void synthesizer_status_callback(std_msgs::String status);
bool detect_requested_drink(double pos[3]);
void talk(std::string msg);



#endif /* OBJECT_DETECTION_H_ */

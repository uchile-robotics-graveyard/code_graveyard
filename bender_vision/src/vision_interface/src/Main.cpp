#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <string.h>
#include <std_srvs/Empty.h>
#include "bender_srvs/ObjectDetection.h"
#include <bender_msgs/SiftDetection.h>

using namespace std;

vector<string> name;
vector<double> x;
vector<double> y;
vector<double> z;


void SiftDetectionCallback(SiftDetector::SiftDetection msg)
{
	for(uint j=0;j<msg.x.size();j++){
		for(uint i=0;i<x.size();i++){
			if(strcmp(msg.names[j].c_str(),name[i].c_str())==0)
			{
				x[i]=msg.x[j];
				y[i]=msg.y[j];
				z[i]=msg.z[j];
				return;
			}
		}
		name.push_back(msg.names[j]);
		x.push_back(msg.x[j]);
		y.push_back(msg.y[j]);
		z.push_back(msg.z[j]);
		return;
	}
}
bool DetectedObj(bender_srvs::ObjectDetection::Request &req, bender_srvs::ObjectDetection::Response &res){
	res.name=name;
	res.x=x;
	res.y=y;
	res.z=z;
	return true;
}
bool ResetObj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	name.clear();
	x.clear();
	y.clear();
	z.clear();
	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "arm_vision_interface");
	ros::NodeHandle n;

	ros::Subscriber sub_sift = n.subscribe("/SiftDetection", 10, SiftDetectionCallback);
	ros::ServiceServer obj_server = n.advertiseService("/vision_interface/detect_obj", DetectedObj);
	ros::ServiceServer reset_obj_server = n.advertiseService("/vision_interface/reset_obj", ResetObj);
	ros::spin();
	return 0;
}

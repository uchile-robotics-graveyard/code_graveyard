/*
 * object_detection.cpp
 *
 *  Created on: 22 Jan 2014
 *      Author: gonzaloolave
 */

#include "bender_vision/object_detection.h"


bool detect_requested_drink(double pos[3])
{
	bender_srvs::IsOn sift_on;
	if(!sifton.call(sift_on))
		ROS_ERROR("Failed to call service siftOn");
	bender_srvs::IsOn plane_on;
	if(!planeon.call(plane_on))
		ROS_ERROR("Failed to call service planeOn");

	std_srvs::Empty dumb;
	if(!reset_obj.call(dumb))
		ROS_ERROR("Failed to call service reset_obj");

	bender_srvs::ObjectDetection det_msg;

	bool detected = false;
	ROS_INFO("Reconociendo objetos");
	sleep(10);
	if(!detect_obj.call(det_msg))
		ROS_ERROR("Failed to call service detect_obj");

	else
	{
		ROS_INFO("Reconoci %d objetos",(int)det_msg.response.name.size());
		for(uint i=0; i<det_msg.response.name.size(); i++)
		{
			ROS_INFO("Detecte: %s", det_msg.response.name[i].data());
			if(det_msg.response.name[i].compare(recognized_drink) == 0)
			{
				pos[0] = det_msg.response.x[i];
				pos[1] = det_msg.response.y[i];
				pos[2] = det_msg.response.z[i];
				detected = true;
				break;
			}
		}
	}

	bender_srvs::IsOn plane_off;
	planeoff.call(plane_off);
	bender_srvs::IsOn sift_off;
	siftoff.call(sift_off);

	return detected;
}

bool reconocer_objeto_callback(bender_srvs::objrecService::Request &req,bender_srvs::objrecService::Response &res){

	recognized_drink = req.object;

	double pos[3];
	bool detected = detect_requested_drink(pos);

	if(detected)
	{
		res.find = true;
		res.object.pose.position.x = pos[0];
		res.object.pose.position.y = pos[1];
		res.object.pose.position.z = pos[2];
		res.object.pose.orientation.w = 1.0;

		res.object.header.frame_id = "/bender/sensors/rgbd_head_link";
		res.object.header.stamp = ros::Time::now();

		ROS_INFO("Objetc '%s' found at X: %f Y: %f Z: %f",req.object.c_str(),res.object.pose.position.x,res.object.pose.position.y,res.object.pose.position.z);
	}

	return true;
}

void talk(std::string msg)
{
	bender_srvs::synthesize say;
	say.request.text = msg;
	if(!talk_srv.call(say))
		ROS_ERROR("Failed to call sythesize");
	ros::spinOnce();
	ros::Rate loop_rate(1);
	loop_rate.sleep();
	while(synthesizer_status == "Talking" && ros::ok())
	{
		ROS_INFO("waiting for Bender to stop talking");
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void synthesizer_status_callback(std_msgs::String status)
{
	synthesizer_status = status.data.c_str();
}


int main(int argc, char **argv)
{
	ros::init(argc,argv,"speech_test");
	ros::NodeHandle n;

	talk_srv = n.serviceClient<bender_srvs::synthesize>("speech_synthesizer/synthesize");
	talk_sub = n.subscribe("speech_synthesizer/status",100,synthesizer_status_callback);

	sifton = n.serviceClient<bender_srvs::IsOn>("/siftOn");
	siftoff = n.serviceClient<bender_srvs::IsOn>("/siftOff");
	planeon = n.serviceClient<bender_srvs::IsOn>("/planeOn");
	planeoff = n.serviceClient<bender_srvs::IsOn>("/planeOff");
	detect_obj = n.serviceClient<bender_srvs::ObjectDetection>("/arm_vision_interface/detect_obj");
	reset_obj = n.serviceClient<std_srvs::Empty>("/arm_vision_interface/reset_obj");

	reconocedor = n.advertiseService("find_object",reconocer_objeto_callback);

	ros::spin();


}


#include "bender_srvs/load_dictionary_service.h"
#include "bender_srvs/synthesize.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "bender_srvs/siftOn.h"
#include "bender_srvs/siftOff.h"
#include "bender_srvs/planeOn.h"
#include "bender_srvs/planeOff.h"
#include "bender_srvs/ObjectDetection.h"
#include "bender_srvs/Dummy.h"
#include "bender_srvs/Dummy.h"
#include "bender_srvs/PlanningGoalCartesian.h"
#include "bender_srvs/LoadMode.h"
#include "string.h"
#include <ros/ros.h>

std::string synthesizer_status;
int recognized_type = 0;

ros::Subscriber talk_sub;
ros::ServiceClient talk_srv;

void synthesizer_status_callback(std_msgs::String status)
{
	synthesizer_status = status.data.c_str();
}

void recognizer_callback(std_msgs::String rec)
{
	std::string msg = rec.data.c_str();
	ROS_INFO("Bender heard: %s",rec.data.c_str());
	if (msg == "coke")
		recognized_type = 1;
	if (msg == "fanta")
		recognized_type = 2;
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

int main(int argc, char **argv)
{
	ros::init(argc,argv,"speech_test");
	ros::NodeHandle n;

	ros::ServiceClient recon = n.serviceClient<bender_srvs::load_dictionary_service>("speech_recognizer/load_dictionary");
	ros::ServiceClient start_recon = n.serviceClient<std_srvs::Empty>("speech_recognizer/start");
	ros::ServiceClient stop_recon = n.serviceClient<std_srvs::Empty>("speech_recognizer/stop");
	ros::Subscriber recon_sub = n.subscribe("speech_recognizer/output",100,recognizer_callback);
	talk_srv = n.serviceClient<bender_srvs::synthesize>("speech_synthesizer/synthesize");
	talk_sub = n.subscribe("speech_synthesizer/status",100,synthesizer_status_callback);

	ros::ServiceClient sifton = n.serviceClient<bender_srvs::siftOn>("/siftOn");
	ros::ServiceClient siftoff = n.serviceClient<bender_srvs::siftOff>("/siftOff");
	ros::ServiceClient planeon = n.serviceClient<bender_srvs::planeOn>("/planeOn");
	ros::ServiceClient planeoff = n.serviceClient<bender_srvs::planeOff>("/planeOff");
	ros::ServiceClient detect_obj = n.serviceClient<bender_srvs::ObjectDetection>("/arm_vision_interface/detect_obj");
	ros::ServiceClient reset_obj = n.serviceClient<bender_srvs::Dummy>("/arm_vision_interface/reset_obj");

	ros::ServiceClient preman1 = n.serviceClient<bender_srvs::Dummy>("/left_arm/posicion_premanipulacion1");
	ros::ServiceClient preman2 = n.serviceClient<bender_srvs::Dummy>("/left_arm/posicion_premanipulacion2");
	ros::ServiceClient abrir_grip = n.serviceClient<bender_srvs::Dummy>("/left_arm/abrir_grip");
	ros::ServiceClient grasp = n.serviceClient<bender_srvs::PlanningGoalCartesian>("/left_arm/grasp");
	ros::ServiceClient orientar_grip = n.serviceClient<bender_srvs::Dummy>("/left_arm/orientar_grip");
	ros::ServiceClient cerrar_grip = n.serviceClient<bender_srvs::LoadMode>("/left_arm/cerrar_grip");
	ros::ServiceClient postman1 = n.serviceClient<bender_srvs::Dummy>("/left_arm/posicion_postmanipulacion1");

	bender_srvs::load_dictionary_service msg;
	msg.request.dictionary = "cocktail";
	if(!recon.call(msg))
		ROS_ERROR("Failed to load dictionary");

	talk("what drink do you want?");

	std_srvs::Empty empty;
	if(!start_recon.call(empty))
		ROS_ERROR("Failed to start recognizing");

	ros::Rate loop_rate(1);
	while(recognized_type == 0 && ros::ok())
	{
		ROS_INFO("Bender hasn't recognized any word");
		ros::spinOnce();
		loop_rate.sleep();
	}
	if(!stop_recon.call(empty))
		ROS_ERROR("Failed to stop recognizing");

	ROS_INFO("Recognized type: %d", recognized_type);

	std::string request;
	bool fetch = false;
	if(recognized_type == 1)
	{
		talk("I will fetch you a coke");
		request = "cocacola";
		fetch =true;
	}
	if(recognized_type == 2)
	{
		talk("I will fetch you a fanta");
		request = "fanta";
		fetch = true;
	}
	if(fetch)
	{
		bender_srvs::siftOn sift_on;
		if(!sifton.call(sift_on))
			ROS_ERROR("Failed to call service siftOn");
		bender_srvs::planeOn plane_on;
		if(!planeon.call(plane_on))
			ROS_ERROR("Failed to call service planeOn");

		bender_srvs::Dummy dumb;
		if(!reset_obj.call(dumb))
			ROS_ERROR("Failed to call service reset_obj");

		bender_srvs::ObjectDetection det_msg;

		bool detected = false;
		double x, y, z;
		sleep(10);
		if(!detect_obj.call(det_msg))
			ROS_ERROR("Failed to call service detect_obj");

		else
		{
			ROS_INFO("Reconociendo objetos");
			ROS_INFO("Objetos reconocidos");
			ROS_INFO("Reconoci %d objetos",det_msg.response.name.size());
			for(int i=0; i<det_msg.response.name.size(); i++)
			{
				ROS_INFO("Detecte: %s", det_msg.response.name[i].data());
				if(det_msg.response.name[i].compare(request) == 0)
				{
					x = det_msg.response.x[i];
					y = det_msg.response.y[i];
					z = det_msg.response.z[i];
					detected = true;
					break;
				}
			}
		}
		if(detected)
		{
			bender_srvs::Dummy dum;
			bender_srvs::PlanningGoalCartesian plan_req;
			plan_req.request.x = x;
			plan_req.request.y = y;
			plan_req.request.z = z;
			bender_srvs::LoadMode lm;
			lm.request.loadmode = 0;

			preman1.call(dum);
			preman2.call(dum);
			abrir_grip.call(dum);
			grasp.call(plan_req);
			orientar_grip.call(dum);
			cerrar_grip.call(lm);
			postman1.call(dum);
			preman1.call(dum);

		}
		bender_srvs::planeOff plane_off;
		planeoff.call(plane_off);
		bender_srvs::siftOff sift_off;
		siftoff.call(sift_off);
	}

}





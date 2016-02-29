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
std::string recognized_drink = "none";
#define recognizable_drinks_size 2
char * recognizable_drinks[] = {"coke","fanta"};

ros::ServiceClient recon;
ros::ServiceClient start_recon;
ros::ServiceClient stop_recon;
ros::Subscriber recon_sub;
ros::Subscriber talk_sub;
ros::ServiceClient talk_srv;

ros::ServiceClient sifton;
ros::ServiceClient siftoff;
ros::ServiceClient planeon;
ros::ServiceClient planeoff;
ros::ServiceClient detect_obj;
ros::ServiceClient reset_obj;

ros::ServiceClient preman1;
ros::ServiceClient preman2;
ros::ServiceClient abrir_grip;
ros::ServiceClient grasp;
ros::ServiceClient orientar_grip;
ros::ServiceClient cerrar_grip;
ros::ServiceClient postman1;

void synthesizer_status_callback(std_msgs::String status)
{
	synthesizer_status = status.data.c_str();
}

void recognizer_callback(std_msgs::String rec)
{
	std::string msg = rec.data.c_str();
	ROS_INFO("Bender heard: %s",rec.data.c_str());
	for(int i = 0; i < recognizable_drinks_size; i++)
	{
		if(msg.compare(recognizable_drinks[i]) == 0)
			recognized_drink = recognizable_drinks[i];
	}
}

void talk(char * msg)
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

void recognize_drink_request()
{
	bender_srvs::load_dictionary_service msg;
	msg.request.dictionary = "cocktail";
	if(!recon.call(msg))
		ROS_ERROR("Failed to load dictionary");

	talk("what drink do you want?");

	std_srvs::Empty empty;
	if(!start_recon.call(empty))
		ROS_ERROR("Failed to start recognizing");

	ros::Rate loop_rate(1);
	while(recognized_drink == "none" && ros::ok())
	{
		ROS_INFO("Bender hasn't recognized any word");
		loop_rate.sleep();
		ros::spinOnce();
	}
	if(!stop_recon.call(empty))
		ROS_ERROR("Failed to stop recognizing");

	ROS_INFO("Recognized drink: %s", recognized_drink.c_str());
}

bool detect_requested_drink(double pos[3])
{
	char talk_msg[50];
	sprintf(talk_msg,"I will fetch you a %s",recognized_drink.c_str());
	talk(talk_msg);

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
	ROS_INFO("Reconociendo objetos");
	sleep(10);
	if(!detect_obj.call(det_msg))
		ROS_ERROR("Failed to call service detect_obj");

	else
	{
		ROS_INFO("Reconoci %d objetos",det_msg.response.name.size());
		for(int i=0; i<det_msg.response.name.size(); i++)
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

	bender_srvs::planeOff plane_off;
	planeoff.call(plane_off);
	bender_srvs::siftOff sift_off;
	siftoff.call(sift_off);

	return detected;
}

void fetch_detected_drink(double pos[3])
{
	bender_srvs::Dummy dum;
	bender_srvs::PlanningGoalCartesian plan_req;
	plan_req.request.x = pos[0];
	plan_req.request.y = pos[1];
	plan_req.request.z = pos[2];
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

int main(int argc, char **argv)
{
	ros::init(argc,argv,"speech_test");
	ros::NodeHandle n;

	recon = n.serviceClient<bender_srvs::load_dictionary_service>("speech_recognizer/load_dictionary");
	start_recon = n.serviceClient<std_srvs::Empty>("speech_recognizer/start");
	stop_recon = n.serviceClient<std_srvs::Empty>("speech_recognizer/stop");
	recon_sub = n.subscribe("speech_recognizer/output",100,recognizer_callback);
	talk_srv = n.serviceClient<bender_srvs::synthesize>("speech_synthesizer/synthesize");
	talk_sub = n.subscribe("speech_synthesizer/status",100,synthesizer_status_callback);

	sifton = n.serviceClient<bender_srvs::siftOn>("/siftOn");
	siftoff = n.serviceClient<bender_srvs::siftOff>("/siftOff");
	planeon = n.serviceClient<bender_srvs::planeOn>("/planeOn");
	planeoff = n.serviceClient<bender_srvs::planeOff>("/planeOff");
	detect_obj = n.serviceClient<bender_srvs::ObjectDetection>("/arm_vision_interface/detect_obj");
	reset_obj = n.serviceClient<bender_srvs::Dummy>("/arm_vision_interface/reset_obj");

	preman1 = n.serviceClient<bender_srvs::Dummy>("/left_arm/posicion_premanipulacion1");
	preman2 = n.serviceClient<bender_srvs::Dummy>("/left_arm/posicion_premanipulacion2");
	abrir_grip = n.serviceClient<bender_srvs::Dummy>("/left_arm/abrir_grip");
	grasp = n.serviceClient<bender_srvs::PlanningGoalCartesian>("/left_arm/grasp");
	orientar_grip = n.serviceClient<bender_srvs::Dummy>("/left_arm/orientar_grip");
	cerrar_grip = n.serviceClient<bender_srvs::LoadMode>("/left_arm/cerrar_grip");
	postman1 = n.serviceClient<bender_srvs::Dummy>("/left_arm/posicion_postmanipulacion1");


	recognize_drink_request();
	double pos[3];
	bool detected = detect_requested_drink(pos);


	if(detected)
	{
		fetch_detected_drink(pos);
	}
}






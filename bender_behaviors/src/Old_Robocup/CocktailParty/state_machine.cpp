/*
 * state_machine.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: bender
 */

// Maquina de estados para el Cocktail Party v1.0

#include <Stage2/state_machine.h>

void change_state(){


}

bool ask_for_name_and_enroll(){

	bender_srvs::NavGoal nav_goal;
	bender_srvs::stringReturn name;

	if (!enroll_person.call(name)){
		ROS_ERROR("Failed to call service '/bender_macros/enroll_person'");
		return false;
	}
	_name = name.response.data;
	PersonMap[id_persona].id = id_persona;
	PersonMap[id_persona].name = _name;

	ROS_INFO("Id: %d\n",id_persona);
	ROS_INFO("Nombre: %s\n",_name.c_str());

	return true;
}

bool ask_for_order(){

	bender_srvs::stringReturn order;

	if (!request_drink.call(order)){
		ROS_ERROR("Failed to call service '/drink_request'");
		return false;
	}
	else {
		_order = order.response.data;
		PersonMap[id_persona].order = _order;
		ROS_INFO("Orden: %s\n",_order.c_str());
		return true;
	}
}
bool search_request(){

	bender_srvs::objrecService object_request;

	object_request.request.object = _order;

	if (!find_order.call(object_request)){
		ROS_ERROR("Failed to call service '/bender_macros/find_object'");
	}
	if (object_request.response.find){
		_object_pose[0] = object_request.response.object.pose.position.x;
		_object_pose[1] = object_request.response.object.pose.position.y;
		_object_pose[2] = object_request.response.object.pose.position.z;

		ROS_INFO("'%s' founded at X: %f Y: %f Z: %f", _order.c_str(),_object_pose[0],_object_pose[1],_object_pose[2]);
		return true;
	}
	else {
		ROS_ERROR("Object not founded");
		return false;
	}

}

void grasp_request(){

	bender_srvs::armGoal arm_goal;
	arm_goal.request.object.pose.position.x = _object_pose[0];
	arm_goal.request.object.pose.position.y = _object_pose[1];
	arm_goal.request.object.pose.position.z = _object_pose[2];

	if (!grasp_object.call(arm_goal)){
		ROS_ERROR("Failed to call service '/bender_macros/grasp_object'");
	}
	else
		ROS_INFO("I'm ready");


}

void init_test(){

//	ask_for_name_and_enroll();
	if(ask_for_order())
		search_request();
//	grasp_request();

}

int main(int argc, char **argv){

	ros::init(argc,argv,"cocktailpartyv1");
	ros::NodeHandle nh;
	n = &nh;

	enroll_person = n->serviceClient<bender_srvs::stringReturn>("bender_macros/enroll_person");
	request_drink = n->serviceClient<bender_srvs::stringReturn>("/drink_request");
	grasp_object = n->serviceClient<bender_srvs::armGoal>("bender_macros/grasp_object");
	go_to_pose = n->serviceClient<bender_srvs::NavGoal>("/goalServer/approach");
	find_order = n->serviceClient<bender_srvs::objrecService>("bender_macros/find_object");


	init_test();

	ros::spin();

	return 0;
}



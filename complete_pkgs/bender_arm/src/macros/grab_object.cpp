/*
 * grab_object.cpp
 *
 *  Created on: 22 Jan 2014
 *      Author: gonzaloolave
 */

#include "bender_arm/grab_object.h"

void fetch_detected_drink(double pos[3])
{
	bender_srvs::Dummy dum;
	bender_srvs::PlanningGoalCartesian plan_req;
	plan_req.request.x = pos[0];
	plan_req.request.y = pos[1];
	plan_req.request.z = pos[2];
	bender_srvs::LoadMode lm;
	lm.request.loadmode = 0;

	if (selected_arm == "left"){
		lpreman1.call(dum);
		lpreman2.call(dum);
		labrir_grip.call(dum);
		lgrasp.call(plan_req);
		lorientar_grip.call(dum);
		lcerrar_grip.call(lm);
		lpostman1.call(dum);
		lpreman1.call(dum);
	}
	else if(selected_arm == "right"){
		rpreman1.call(dum);
		rpreman2.call(dum);
		rabrir_grip.call(dum);
		rgrasp.call(plan_req);
		rorientar_grip.call(dum);
		rcerrar_grip.call(lm);
		rpostman1.call(dum);
		rpreman1.call(dum);
	}

}

bool select_arm(double *pos){

	bender_srvs::stringReturn brazo;

	if(pos[1]<0){
		if(n->getParam("/dynamixel_manager/serial_ports/right_arm_port/port_name", right_port)){
			ROS_WARN("RIGHT ARM SETTED");
			rpreman1 = n->serviceClient<bender_srvs::Dummy>("/right_arm/posicion_premanipulacion1");
			rpreman2 = n->serviceClient<bender_srvs::Dummy>("/right_arm/posicion_premanipulacion2");
			rabrir_grip = n->serviceClient<bender_srvs::Dummy>("/right_arm/abrir_grip");
			rgrasp = n->serviceClient<bender_srvs::PlanningGoalCartesian>("/right_arm/grasp");
			rorientar_grip = n->serviceClient<bender_srvs::Dummy>("/right_arm/orientar_grip");
			rcerrar_grip = n->serviceClient<bender_srvs::LoadMode>("/right_arm/cerrar_grip");
			rpostman1 = n->serviceClient<bender_srvs::Dummy>("/right_arm/posicion_postmanipulacion1");
			selected_arm = "right";
			return true;
		}
		else if(n->getParam("/dynamixel_manager/serial_ports/left_arm_port/port_name", left_port)){
			ROS_WARN("LEFT ARM SETTED");
			lpreman1 = n->serviceClient<bender_srvs::Dummy>("/left_arm/posicion_premanipulacion1");
			lpreman2 = n->serviceClient<bender_srvs::Dummy>("/left_arm/posicion_premanipulacion2");
			labrir_grip = n->serviceClient<bender_srvs::Dummy>("/left_arm/abrir_grip");
			lgrasp = n->serviceClient<bender_srvs::PlanningGoalCartesian>("/left_arm/grasp");
			lorientar_grip = n->serviceClient<bender_srvs::Dummy>("/left_arm/orientar_grip");
			lcerrar_grip = n->serviceClient<bender_srvs::LoadMode>("/left_arm/cerrar_grip");
			lpostman1 = n->serviceClient<bender_srvs::Dummy>("/left_arm/posicion_postmanipulacion1");
			selected_arm = "left";
			return true;
		}
		else{
			ROS_ERROR("RIGHT ARM UNSETTED");
			return false;
		}
	}

	else{
		if(n->getParam("/dynamixel_manager/serial_ports/left_arm_port/port_name", left_port)){
			ROS_WARN("LEFT ARM SETTED");
			lpreman1 = n->serviceClient<bender_srvs::Dummy>("/left_arm/posicion_premanipulacion1");
			lpreman2 = n->serviceClient<bender_srvs::Dummy>("/left_arm/posicion_premanipulacion2");
			labrir_grip = n->serviceClient<bender_srvs::Dummy>("/left_arm/abrir_grip");
			lgrasp = n->serviceClient<bender_srvs::PlanningGoalCartesian>("/left_arm/grasp");
			lorientar_grip = n->serviceClient<bender_srvs::Dummy>("/left_arm/orientar_grip");
			lcerrar_grip = n->serviceClient<bender_srvs::LoadMode>("/left_arm/cerrar_grip");
			lpostman1 = n->serviceClient<bender_srvs::Dummy>("/left_arm/posicion_postmanipulacion1");
			selected_arm = "left";
			return true;
			
		}
		else if(n->getParam("/dynamixel_manager/serial_ports/right_arm_port/port_name", right_port)){
			ROS_WARN("RIGHT ARM SETTED");
			rpreman1 = n->serviceClient<bender_srvs::Dummy>("/right_arm/posicion_premanipulacion1");
			rpreman2 = n->serviceClient<bender_srvs::Dummy>("/right_arm/posicion_premanipulacion2");
			rabrir_grip = n->serviceClient<bender_srvs::Dummy>("/right_arm/abrir_grip");
			rgrasp = n->serviceClient<bender_srvs::PlanningGoalCartesian>("/right_arm/grasp");
			rorientar_grip = n->serviceClient<bender_srvs::Dummy>("/right_arm/orientar_grip");
			rcerrar_grip = n->serviceClient<bender_srvs::LoadMode>("/right_arm/cerrar_grip");
			rpostman1 = n->serviceClient<bender_srvs::Dummy>("/right_arm/posicion_postmanipulacion1");
			selected_arm = "right";
			return true;
		}
		else{
			ROS_ERROR("RIGHT ARM UNSETTED");
			return false;
		}
	}
	
	return false;

}


bool manipular_objeto_callback(bender_srvs::armGoal::Request &req, bender_srvs::armGoal::Response &res){

	double pos[3];

	pos[0] = req.object.pose.position.x;
	pos[1] = req.object.pose.position.y;
	pos[2] = req.object.pose.position.z;

	if(!select_arm(pos))
		return false;

	fetch_detected_drink(pos);

	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc,argv,"grasp_object");
	ros::NodeHandle nh;
	n = &nh;


	servicio = n->advertiseService("grasp_object", manipular_objeto_callback);
//	escoger = n->serviceClient<bender_srvs::selectBrazo>("/select_arm");

	ros::spin();

	//	recognize_drink_request();
//	double pos[3];
//	bool detected = detect_requested_drink(pos);
//
//
//	if(detected)
//	{
//		fetch_detected_drink(pos);
//	}
}

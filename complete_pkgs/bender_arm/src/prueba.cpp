#include "ros/ros.h"
#include <math.h>
#include "arm_controller/Command.h"


/********** MAIN *********/
int main(int argc, char **argv){

	/**** Inicializacion ****/
	ros::init(argc, argv, "bottomArm_ControllerNode");
	ros::NodeHandle n;
	
	//Publishers	
	ros::Publisher bottomMotors_pub = n.advertise<arm_controller::Command>("bottom_arm_joints/command",2);

	ros::Rate loop_rate(30);

	float positions[6] = {0.0,-1.57,0.0,0.0,0.0,0.0};
	bool selection[6] = {true,true,true,false,false,false};
	float speeds[6] = {0.3,0.3,0.4,0.3,0.3,0.3};

	while (ros::ok()){
		positions[0]+=.1;
		arm_controller::Command cmdMsg;
		cmdMsg.positions.resize(6);
		cmdMsg.select.resize(6);
		cmdMsg.speed.resize(6);

		for(int i=0;i<6;i++){
			cmdMsg.positions[i] = positions[i];
			cmdMsg.select[i] = selection[i];
			cmdMsg.speed[i] = speeds[i];
		}

		bottomMotors_pub.publish(cmdMsg);


		//fin del ciclo
		ros::spinOnce();
		loop_rate.sleep();	

	}
}

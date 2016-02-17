#include "bender_planning_old/planner.h"
#include <math.h>
#include "bender_msgs/CartesianInfo.h"
#include "bender_msgs/StateInfo.h"


int main(int argc, char **argv){
	ros::init(argc, argv, "left_planner");
	Uarm brazo = Uarm();
	brazo.spinner.start();
	double qi[4];
	double pos[3];
	double posg[3];
	ros::Publisher cartesian_pub=brazo.nh.advertise<bender_msgs::CartesianInfo>("left_arm/cartesian_info", 1000);
	ros::Publisher grasp_pub=brazo.nh.advertise<bender_msgs::CartesianInfo>("left_arm/grasp_info", 1000);
	ros::Publisher state_pub=brazo.nh.advertise<bender_msgs::StateInfo>("left_arm/state_info", 1000);

	double err = 0;
	double ierr = 0;
	double derr = 0;
	double kp = 0.1;
	double ki = 0.03;
	double kd = 0.02;
	double torque_ref = 0.43;
	brazo.nh.param("torque_ref",torque_ref,0.43);

	ros::Rate loop_rate(10);

	while(ros::ok()){
		//publicar info del brazo
		brazo.leerPoseServo(qi);
		bender_msgs::StateInfo state;
		state.s0=qi[0];
		state.s1=qi[1];
		state.s2=qi[2];
		state.s3=qi[3];
		//qi[0]=0;
		//qi[1]=0;
		//qi[2]=0;
		//qi[3]=0;
		brazo.CinematicaDirecta(qi,pos);
		brazo.CinematicaDirectaGrasp(qi,posg);
		bender_msgs::CartesianInfo position;
		bender_msgs::CartesianInfo positiong;
		position.x=pos[0];
		position.y=pos[1];
		position.z=pos[2];
		positiong.x=posg[0];
		positiong.y=posg[1];
		positiong.z=posg[2];
		cartesian_pub.publish(position);
		grasp_pub.publish(positiong);
		state_pub.publish(state);

		//control para apretar
		if(brazo.apretar)
		{
			ROS_INFO("Lazo de apretar activo");
			ros::ServiceClient client = brazo.nh.serviceClient<bender_srvs::States>("left_arm_joints/states");
			bender_srvs::States srv;
			srv.request.select.resize(8);
			for(int i=0; i<6;i++)
				srv.request.select[i]=false;
			for(int i=6; i<8;i++)
				srv.request.select[i]=true;

			double loadL;
			double loadR;

			while(!ros::service::waitForService("left_arm_joints/states", ros::Duration(1.0)) && ros::ok())
			{
				ROS_INFO_STREAM("Waiting for planner service left_arm_joints/states");
			}
			if (client.call(srv))
			{
				loadR = abs(srv.response.load[6]);
				loadL = abs(srv.response.load[7]);
			}
			else
			{
				ROS_ERROR("Failed to call service left_arm_joints/states in Uarm::CerrarGripBrazo");
			}
			derr = err;
			err = torque_ref - 0.5*(loadR+loadL);
			ierr = ierr + err;
			derr = err - derr;
			ROS_INFO("Error: %f, Integral error: %f, Derivada del error:%f",err,ierr,derr);
			brazo.MoverGripAng(0.5*(abs(srv.response.state[6])+abs(srv.response.state[7]))-kp*err-ki*ierr-kd*derr,0.1);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

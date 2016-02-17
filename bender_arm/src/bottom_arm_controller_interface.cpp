#include "ros/ros.h"
#include "dynamixel_msgs/JointState.h"
#include "dynamixel_controllers/SetSpeed.h"
#include "std_msgs/Float64.h"
#include "bender_msgs/Command.h"

ros::Publisher brazo[6];
ros::ServiceClient call_speed[6];

dynamixel_controllers::SetSpeed spe;

void joints_command(bender_msgs::Command bottom);

int main(int argc, char **argv)
{
	ros::init(argc,argv, "right_arm_controller_interface");
    ros::NodeHandle n;

    brazo[0]= n.advertise<std_msgs::Float64>("m_1_controller/command",1000);
    brazo[1]= n.advertise<std_msgs::Float64>("m_2_controller/command",1000);
    brazo[2]= n.advertise<std_msgs::Float64>("m_3_controller/command",1000);
    brazo[3]= n.advertise<std_msgs::Float64>("m_4_controller/command",1000);
    brazo[4]= n.advertise<std_msgs::Float64>("m_5_controller/command",1000);
    brazo[5]= n.advertise<std_msgs::Float64>("m_6_controller/command",1000);

    call_speed[0] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_1_controller/set_speed");
	call_speed[1] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_2_controller/set_speed");
	call_speed[2] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_3_controller/set_speed");
	call_speed[3] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_4_controller/set_speed");
	call_speed[4] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_5_controller/set_speed");
	call_speed[5] = n.serviceClient<dynamixel_controllers::SetSpeed>("m_6_controller/set_speed");

    ros::Subscriber com_read = n.subscribe("bottom_arm_joints/command",1000, joints_command);

    ros::Publisher com_pub = n.advertise<bender_msgs::Command>("bottom_arm_joints/command",1000);

    ros::Rate loop_rate(10);

	ros::spin();
	return 0;
}

void joints_command(bender_msgs::Command bottom)
{
	std_msgs::Float64 pos;
	bottom.positions.resize(8);
	bottom.select.resize(8);
	bottom.speed.resize(8);

	for(int i=0;i<8;i++)
	{
		if(bottom.select[i]!=false)
		{
			spe.request.speed=bottom.speed[i];
			call_speed[i].call(spe);
			// llamar al servicio \joint[i]\setSpeed.call  y publicarle vel
			pos.data=bottom.positions[i];
			brazo[i].publish(pos);

		}
	}

	return;

}

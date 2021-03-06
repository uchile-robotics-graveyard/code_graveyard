#include "ros/ros.h"
#include "dynamixel_msgs/JointState.h"
#include "dynamixel_controllers/SetSpeed.h"
#include "std_msgs/Float64.h"
#include "bender_msgs/Command.h"

ros::Publisher brazo[8];
ros::ServiceClient call_speed[8];

dynamixel_controllers::SetSpeed spe;

void joints_command(bender_msgs::Command left);

int main(int argc, char **argv)
{
	ros::init(argc,argv, "left_arm_controller_interface");
    ros::NodeHandle n;

//    ros::Subscriber hom1 = n.subscribe("hombro_1_controller/state", 1000, joints_pos);
//    ros::Subscriber hom2 = n.subscribe("hombro_2_controller/state", 1000, joints_pos);
//    ros::Subscriber hom3 = n.subscribe("hombro_3_controller/state", 1000, joints_pos);
//    ros::Subscriber cod1 = n.subscribe("codo_1_controller/state", 1000, joints_pos);
//    ros::Subscriber cod2 = n.subscribe("codo_2_controller/state", 1000, joints_pos);
//    ros::Subscriber mun = n.subscribe("muneca_controller/state", 1000, joints_pos);
//    ros::Subscriber ded1 = n.subscribe("dedo_1_controller/state", 1000, joints_pos);
//    ros::Subscriber ded2 = n.subscribe("dedo_2_controller/state", 1000, joints_pos);

    brazo[0]= n.advertise<std_msgs::Float64>("hombro_1_controller/command",1000);
    brazo[1]= n.advertise<std_msgs::Float64>("hombro_2_controller/command",1000);
    brazo[2]= n.advertise<std_msgs::Float64>("hombro_3_controller/command",1000);
    brazo[3]= n.advertise<std_msgs::Float64>("codo_1_controller/command",1000);
    brazo[4]= n.advertise<std_msgs::Float64>("codo_2_controller/command",1000);
    brazo[5]= n.advertise<std_msgs::Float64>("muneca_controller/command",1000);
    brazo[6]= n.advertise<std_msgs::Float64>("dedo_1_controller/command",1000);
    brazo[7]= n.advertise<std_msgs::Float64>("dedo_2_controller/command",1000);

    call_speed[0] = n.serviceClient<dynamixel_controllers::SetSpeed>("hombro_1_controller/set_speed");
   	call_speed[1] = n.serviceClient<dynamixel_controllers::SetSpeed>("hombro_2_controller/set_speed");
   	call_speed[2] = n.serviceClient<dynamixel_controllers::SetSpeed>("hombro_3_controller/set_speed");
   	call_speed[3] = n.serviceClient<dynamixel_controllers::SetSpeed>("codo_1_controller/set_speed");
   	call_speed[4] = n.serviceClient<dynamixel_controllers::SetSpeed>("codo_2_controller/set_speed");
   	call_speed[5] = n.serviceClient<dynamixel_controllers::SetSpeed>("muneca_controller/set_speed");
   	call_speed[6] = n.serviceClient<dynamixel_controllers::SetSpeed>("dedo_1_controller/set_speed");
   	call_speed[7] = n.serviceClient<dynamixel_controllers::SetSpeed>("dedo_2_controller/set_speed");

    ros::Subscriber com_read = n.subscribe("left_arm_joints/command",1000, joints_command);
    // pub de prueba
    ros::Publisher com_pub = n.advertise<bender_msgs::Command>("left_arm_joints/command",1000);

    ros::Rate loop_rate(10);

//    left_pos.arm_states.resize(2);
//    left_load.arm_states.resize(2);
//
//    while(ros::ok())
//    {
//        //ROS_INFO("%f %f", left_arm_state[0],left_arm_state[1]);
//        left_pos.arm_states[0]=left_arm_state[0];
//        left_pos.arm_states[1]=left_arm_state[1];
//        pos_pub.publish(left_pos);
//
//        left_load.arm_states[0]=left_arm_load[0];
//        left_load.arm_states[1]=left_arm_load[1];
//        load_pub.publish(left_load);
//
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
	ros::spin();
	return 0;
}

void joints_command(bender_msgs::Command left)
{
	std_msgs::Float64 pos;
	left.positions.resize(8);
	left.select.resize(8);
	left.speed.resize(8);

	for(int i=0;i<8;i++)
	{
		if(left.select[i]!=false)
		{
			spe.request.speed=left.speed[i];
			call_speed[i].call(spe);
			// llamar al servicio \joint[i]\setSpeed.call  y publicarle vel
			pos.data=left.positions[i];
			brazo[i].publish(pos);

		}
	}

	return;

}

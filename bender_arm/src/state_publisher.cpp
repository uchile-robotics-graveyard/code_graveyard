#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <bender_msgs/baseArmState.h>

ros::Publisher joint_pub;
float baseArmPose [6];


void baseArmCallback(const bender_msgs::baseArmState::ConstPtr& msg)
{
	ROS_INFO("I received: [%f %f %f %f %f %f]", msg->Joint1,msg->Joint2,msg->Joint3,msg->Joint4,msg->Joint5,msg->Joint6);
	//update joint_state
	baseArmPose [0] = msg->Joint1;
	baseArmPose [1] = msg->Joint2;
	baseArmPose [2] = msg->Joint3;
	baseArmPose [3] = msg->Joint4;
	baseArmPose [4] = msg->Joint5;
	baseArmPose [5] = msg->Joint6;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "Bender_Base_Arm_State_Publisher");
    ros::NodeHandle n;
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber baseArmSub = n.subscribe("/PosicionActuadoresBaseArm", 1, baseArmCallback);
  //  tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    for(int i=0;i<6;i++){
    	baseArmPose[i]=0;
    }

    // message declarations
    sensor_msgs::JointState joint_state;

    while (ros::ok()) {

        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(6);
        joint_state.position.resize(6);

        //left_arm
        joint_state.name[0] ="base_to_link2";
        joint_state.position[0] = baseArmPose[0];  
        joint_state.name[1] ="link2_to_link3";
        joint_state.position[1] = baseArmPose[1];
        joint_state.name[2] ="link3_to_link4";
        joint_state.position[2] = baseArmPose[2];
        joint_state.name[3] ="link4_to_link5";
        joint_state.position[3] = baseArmPose[3];
        joint_state.name[4] ="link5_to_pinza_A";
        joint_state.position[4] = baseArmPose[4];
        joint_state.name[5] ="link5_to_pinza_B";
        joint_state.position[5] = baseArmPose[5];

       
        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();

        ros::spinOnce();
    }


    return 0;
}

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <BenderModel/ArmState.h>

ros::Publisher joint_pub;
float RightArmPose [8];
float LeftArmPose [8];

void RightArmCallback(const BenderModel::ArmState::ConstPtr& msg)
{
	ROS_INFO("I received: [%f %f %f %f %f %f %f %f]", msg->Joint1,msg->Joint2,msg->Joint3,msg->Joint4,msg->Joint5,msg->Joint6,msg->Joint7,msg->Joint8);
	//update joint_state
	RightArmPose [0] = msg->Joint1;
	RightArmPose [1] = msg->Joint2;
	RightArmPose [2] = msg->Joint3;
	RightArmPose [3] = msg->Joint4;
	RightArmPose [4] = msg->Joint5;
	RightArmPose [5] = msg->Joint6;
	RightArmPose [6] = msg->Joint7;
	RightArmPose [7] = msg->Joint8;

}
void LeftArmCallback(const BenderModel::ArmState::ConstPtr& msg)
{
	//update joint_state
	LeftArmPose [0] = msg->Joint1;
	LeftArmPose [1] = msg->Joint2;
	LeftArmPose [2] = msg->Joint3;
	LeftArmPose [3] = msg->Joint4;
	LeftArmPose [4] = msg->Joint5;
	LeftArmPose [5] = msg->Joint6;
	LeftArmPose [6] = msg->Joint7;
	LeftArmPose [7] = msg->Joint8;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "Bender_Arm_State_Publisher");
    ros::NodeHandle n;
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber RightArmSub = n.subscribe("/PosicionActuadoresRightArm", 1, RightArmCallback);
    ros::Subscriber LeftArmSub = n.subscribe("/PosicionActuadoresLeftArm", 1, LeftArmCallback);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    for(int i=0;i<8;i++){
    	RightArmPose[i]=0;
    	LeftArmPose[i]=0;
    }

    // message declarations
    sensor_msgs::JointState joint_state;

    while (ros::ok()) {

        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(20);
        joint_state.position.resize(20);

        //left_arm
        joint_state.name[0] ="Left_Shoulder_Y";
        joint_state.position[0] = -LeftArmPose[0];  //NEGATIVO
        joint_state.name[1] ="Left_Shoulder_X";
        joint_state.position[1] = LeftArmPose[1];
        joint_state.name[2] ="Left_Shoulder_Z";
        joint_state.position[2] = LeftArmPose[2];
        joint_state.name[3] ="Left_Elbow_Y";
        joint_state.position[3] = -LeftArmPose[3];  //NEGATIVO
        joint_state.name[4] ="Left_Elbow_Z";
        joint_state.position[4] = LeftArmPose[4];
        joint_state.name[5] ="Left_Wrist_Y";
        joint_state.position[5] = LeftArmPose[5];
        joint_state.name[6] ="Left_2Fingers";
        joint_state.position[6] = LeftArmPose[6];
        joint_state.name[7] ="Left_1Finger";
        joint_state.position[7] = LeftArmPose[7];

        //right_arm
		joint_state.name[8] ="Right_Shoulder_Y";
		joint_state.position[8] = -RightArmPose[0];  //NEGATIVO
		joint_state.name[9] ="Right_Shoulder_X";
		joint_state.position[9] = -RightArmPose[1];  //NEGATIVO
		joint_state.name[10] ="Right_Shoulder_Z";
		joint_state.position[10] = RightArmPose[2];
		joint_state.name[11] ="Right_Elbow_Y";
		joint_state.position[11] = -RightArmPose[3];  //NEGATIVO
		joint_state.name[12] ="Right_Elbow_Z";
		joint_state.position[12] = RightArmPose[4];
		joint_state.name[13] ="Right_Wrist_Y";
		joint_state.position[13] = -RightArmPose[5];  //NEGATIVO
		joint_state.name[14] ="Right_2Fingers";
		joint_state.position[14] = RightArmPose[6];
		joint_state.name[15] ="Right_1Finger";
		joint_state.position[15] = RightArmPose[7];

		//Wheels

		joint_state.name[16] ="Left_Front_Wheel";
		joint_state.position[16] = 0;
		joint_state.name[17] ="Left_Rear_Wheel";
		joint_state.position[17] = 0;
		joint_state.name[18] ="Right_Front_Wheel";
		joint_state.position[18] = 0;
		joint_state.name[19] ="Right_Rear_Wheel";
		joint_state.position[19] = 0;

        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();

        ros::spinOnce();
    }


    return 0;
}

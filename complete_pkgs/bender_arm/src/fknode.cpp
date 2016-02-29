#include <ros/ros.h>
#include "math.h"
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <highgui.h>
#include <iostream>

#include "bender_srvs/States.h"
#include "bender_msgs/FKinfo.h"

using namespace cv;
using namespace std;

void right_forward_kinematics(float a0,float a1,float a2,float a3,float a4,float a5,Mat *fk, Mat *orient){
	Mat H1 = Mat::eye(4,4,CV_32F);
	Mat H2 = Mat::eye(4,4,CV_32F);
	Mat H3 = Mat::eye(4,4,CV_32F);
	Mat H4 = Mat::eye(4,4,CV_32F);
	Mat H5 = Mat::eye(4,4,CV_32F);
	Mat H6 = Mat::eye(4,4,CV_32F);
	Mat H7 = Mat::eye(4,4,CV_32F);
	Mat effector_local = Mat::zeros(4,1,CV_32F);
	Mat H;

	H1.at<float>(0,0) = cos(a0);
	H1.at<float>(0,2) = sin(a0);
	H1.at<float>(2,0) = -sin(a0);
	H1.at<float>(2,2) = cos(a0);
	H1.at<float>(0,3) = 3;
	H1.at<float>(1,3) = -23.5;
	H1.at<float>(2,3) = 123;

	H2.at<float>(1,1) = cos(a1);
	H2.at<float>(1,2) = -sin(a1);
	H2.at<float>(2,1) = sin(a1);
	H2.at<float>(2,2) = cos(a1);
	H2.at<float>(1,3) = -3.8;
	H2.at<float>(2,3) = -3;

	H3.at<float>(0,0) = cos(a2);
	H3.at<float>(0,1) = -sin(a2);
	H3.at<float>(1,0) = sin(a2);
	H3.at<float>(1,1) = cos(a2);
	H3.at<float>(0,3) = 1.5;
	H3.at<float>(2,3) = -16;

	H4.at<float>(0,0) = cos(a3);
	H4.at<float>(0,2) = sin(a3);
	H4.at<float>(2,0) = -sin(a3);
	H4.at<float>(2,2) = cos(a3);
	H4.at<float>(0,3) = -1.8;
	H4.at<float>(2,3) = -18.2;

	H5.at<float>(0,0) = cos(a4);
	H5.at<float>(0,1) = -sin(a4);
	H5.at<float>(1,0) = sin(a4);
	H5.at<float>(1,1) = cos(a4);
	H5.at<float>(0,3) = 0.4;
	H5.at<float>(2,3) = -7.8;

	H6.at<float>(0,0) = cos(a5);
	H6.at<float>(0,2) = sin(a5);
	H6.at<float>(2,0) = -sin(a5);
	H6.at<float>(2,2) = cos(a5);
	H6.at<float>(0,3) = -1.6;
	H6.at<float>(2,3) = -27.9;

	H7.at<float>(2,3) = -16;

	effector_local.at<float>(3,0) = 1;

	H = H1*H2*H3*H4*H5*H6*H7;

	*fk = H*effector_local;

	fk->resize(5);
	fk->at<float>(3,0) = asin(H.at<float>(2,2));
	fk->at<float>(4,0) = -asin(H.at<float>(2,1));

	*orient = Mat::zeros(2,1,CV_32F);
	(*orient).at<float>(0,0) = asin(H.at<float>(2,2));
	(*orient).at<float>(1,0) = -asin(H.at<float>(2,1));

}

void left_forward_kinematics(float a0,float a1,float a2,float a3,float a4,float a5,Mat *fk, Mat *orient){
	Mat H1 = Mat::eye(4,4,CV_32F);
	Mat H2 = Mat::eye(4,4,CV_32F);
	Mat H3 = Mat::eye(4,4,CV_32F);
	Mat H4 = Mat::eye(4,4,CV_32F);
	Mat H5 = Mat::eye(4,4,CV_32F);
	Mat H6 = Mat::eye(4,4,CV_32F);
	Mat H7 = Mat::eye(4,4,CV_32F);
	Mat effector_local = Mat::zeros(4,1,CV_32F);
	Mat H;

	H1.at<float>(0,0) = cos(a0);
	H1.at<float>(0,2) = sin(a0);
	H1.at<float>(2,0) = -sin(a0);
	H1.at<float>(2,2) = cos(a0);
	H1.at<float>(0,3) = 3;
	H1.at<float>(1,3) = 23.5;
	H1.at<float>(2,3) = 123;

	H2.at<float>(1,1) = cos(a1);
	H2.at<float>(1,2) = -sin(a1);
	H2.at<float>(2,1) = sin(a1);
	H2.at<float>(2,2) = cos(a1);
	H2.at<float>(1,3) = 3.8;
	H2.at<float>(2,3) = -3;

	H3.at<float>(0,0) = cos(a2);
	H3.at<float>(0,1) = -sin(a2);
	H3.at<float>(1,0) = sin(a2);
	H3.at<float>(1,1) = cos(a2);
	H3.at<float>(0,3) = 1.5;
	H3.at<float>(2,3) = -16;

	H4.at<float>(0,0) = cos(a3);
	H4.at<float>(0,2) = sin(a3);
	H4.at<float>(2,0) = -sin(a3);
	H4.at<float>(2,2) = cos(a3);
	H4.at<float>(0,3) = -1.8;
	H4.at<float>(2,3) = -18.2;

	H5.at<float>(0,0) = cos(a4);
	H5.at<float>(0,1) = -sin(a4);
	H5.at<float>(1,0) = sin(a4);
	H5.at<float>(1,1) = cos(a4);
	H5.at<float>(0,3) = 0.4;
	H5.at<float>(2,3) = -7.8;

	H6.at<float>(0,0) = cos(a5);
	H6.at<float>(0,2) = sin(a5);
	H6.at<float>(2,0) = -sin(a5);
	H6.at<float>(2,2) = cos(a5);
	H6.at<float>(0,3) = -1.6;
	H6.at<float>(2,3) = -27.9;

	H7.at<float>(2,3) = -16;

	effector_local.at<float>(3,0) = 1;

	H = H1*H2*H3*H4*H5*H6*H7;

	*fk = H*effector_local;

	*orient = Mat::zeros(2,1,CV_32F);
	(*orient).at<float>(0,0) = asin(H.at<float>(2,2));
	(*orient).at<float>(1,0) = -asin(H.at<float>(2,1));

}

int main(int argc, char **argv){

	float a0 = 0.0;
	float a1 = 0.0;
	float a2 = 0.0;
	float a3 = 0.0;
	float a4 = 0.0;
	float a5 = 0.0;

	ros::init(argc, argv, "fknode");

	ros::NodeHandle nh;

	while(!ros::service::waitForService("left_arm_joints/states", ros::Duration(1.0)))
	{
		ROS_INFO_STREAM("Waiting for planner service left_arm_joints/states");
	}

	ros::ServiceClient client = nh.serviceClient<bender_srvs::States>("left_arm_joints/states");
	ros::Publisher pub = nh.advertise<bender_msgs::FKinfo>("left_arm/Forward_Kinematics",1000);

	bender_srvs::States srv;
	srv.request.select.resize(8);
	for(int i = 0; i < 6;i++)
		srv.request.select[i]=true;
	for(int i = 6; i < 8;i++)
		srv.request.select[i]=false;
	//ROS_INFO("Request para lectura de poses rellenado");

	ros::Rate loop_rate(10);

	while (ros::ok())
	{

		if (client.call(srv))
		{
			//ROS_INFO("Llamada a servicio de lectura de poses exitosa");
			a0 = srv.response.state[0];
			a1 = srv.response.state[1];
			a2 = srv.response.state[2];
			a3 = srv.response.state[3];
			a4 = srv.response.state[4];
			a5 = srv.response.state[5];
		}
		else
		{
			ROS_ERROR("Failed to call service left_arm_joints/states in fknode");
		return 1;
		}

		Mat fk;
		Mat orient;

		left_forward_kinematics(a0,-a1,a2,-a3,-a4,a5,&fk,&orient);

		bender_msgs::FKinfo fk_msg;

		fk_msg.x = fk.at<float>(0,0);
		fk_msg.y = fk.at<float>(1,0);
		fk_msg.z = fk.at<float>(2,0);
		fk_msg.pitch = orient.at<float>(0,0);
		fk_msg.roll = orient.at<float>(1,0);

		pub.publish(fk_msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

}

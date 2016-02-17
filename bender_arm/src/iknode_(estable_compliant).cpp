#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "bender_planning_old/States.h"
#include "bender_planning_old/Command.h"
#include "bender_planning_old/Dummy.h"
#include "kinematics/PlanIKGoal.h"
#include "BenderModel/ArmState.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "arm_controller/Speed.h"
#include "arm_controller/TorqueLimit.h"
#include <stdio.h>
#include <iostream>

#include "math.h"

using namespace std;
using namespace cv;

double arm_state[6] = {0,0,0,0,0,-0.01};
double goal[5];
bool HOLD = false;
bool CHANGE_GOAL = false;
int m_sign[6] = {1, -1, 1, -1, -1, 1};

ros::Publisher state_pub;
ros::ServiceClient state_client;

void left_forward_kinematics(double a0,double a1,double a2,double a3,double a4,double a5,Mat *fk){
	Mat H1 = Mat::eye(4,4,CV_64F);
	Mat H2 = Mat::eye(4,4,CV_64F);
	Mat H3 = Mat::eye(4,4,CV_64F);
	Mat H4 = Mat::eye(4,4,CV_64F);
	Mat H5 = Mat::eye(4,4,CV_64F);
	Mat H6 = Mat::eye(4,4,CV_64F);
	Mat H7 = Mat::eye(4,4,CV_64F);
	Mat effector_local = Mat::zeros(4,1,CV_64F);
	Mat H;

	H1.at<double>(0,0) = cos(a0);
	H1.at<double>(0,2) = sin(a0);
	H1.at<double>(2,0) = -sin(a0);
	H1.at<double>(2,2) = cos(a0);
	H1.at<double>(0,3) = 3;
	H1.at<double>(1,3) = 23.5;
	H1.at<double>(2,3) = 124;

	H2.at<double>(1,1) = cos(a1);
	H2.at<double>(1,2) = -sin(a1);
	H2.at<double>(2,1) = sin(a1);
	H2.at<double>(2,2) = cos(a1);
	H2.at<double>(1,3) = 3.8;
	H2.at<double>(2,3) = -3;

	H3.at<double>(0,0) = cos(a2);
	H3.at<double>(0,1) = -sin(a2);
	H3.at<double>(1,0) = sin(a2);
	H3.at<double>(1,1) = cos(a2);
	H3.at<double>(0,3) = 1.5;
	H3.at<double>(2,3) = -16;

	H4.at<double>(0,0) = cos(a3);
	H4.at<double>(0,2) = sin(a3);
	H4.at<double>(2,0) = -sin(a3);
	H4.at<double>(2,2) = cos(a3);
	H4.at<double>(0,3) = -1.8;
	H4.at<double>(2,3) = -18.2;

	H5.at<double>(0,0) = cos(a4);
	H5.at<double>(0,1) = -sin(a4);
	H5.at<double>(1,0) = sin(a4);
	H5.at<double>(1,1) = cos(a4);
	H5.at<double>(0,3) = 0.4;
	H5.at<double>(2,3) = -7.8;

	H6.at<double>(0,0) = cos(a5);
	H6.at<double>(0,2) = sin(a5);
	H6.at<double>(2,0) = -sin(a5);
	H6.at<double>(2,2) = cos(a5);
	H6.at<double>(0,3) = -1.6;
	H6.at<double>(2,3) = -27.9;

	H7.at<double>(2,3) = -16;

	effector_local.at<double>(3,0) = 1;

	H = H1*H2*H3*H4*H5*H6*H7;

	*fk = H*effector_local;

	fk->resize(5);
	fk->at<double>(3,0) = asin(H.at<double>(2,2));
	fk->at<double>(4,0) = -asin(H.at<double>(2,1));

}

void jacobian(double *alpha, Mat *J){
	*J = Mat(5,6,CV_64F);

	(*J).at<double>(0,0) = -0.16e2 * (((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) - 0.16e2 * ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[5]) - 0.16e1 * ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) - 0.16e1 * (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4]) - 0.357e2 * (-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) - 0.357e2 * cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3]) + 0.4e0 * (-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - 0.4e0 * cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3]) + 0.18e1 * sin(alpha[0]) * cos(alpha[2]) - 0.18e1 * cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2]) - 0.342e2 * cos(alpha[0]) * cos(alpha[1]) - 0.15e1 * sin(alpha[0]) - 0.3e1 * cos(alpha[0]);
	(*J).at<double>(0,1) = -0.16e2 * ((sin(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + sin(alpha[0]) * sin(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + sin(alpha[0]) * cos(alpha[1]) * cos(alpha[2]) * sin(alpha[4])) * sin(alpha[5]) - 0.16e2 * (sin(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * sin(alpha[3]) - sin(alpha[0]) * sin(alpha[1]) * cos(alpha[3])) * cos(alpha[5]) - 0.16e1 * (sin(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + sin(alpha[0]) * sin(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) - 0.16e1 * sin(alpha[0]) * cos(alpha[1]) * cos(alpha[2]) * sin(alpha[4]) - 0.357e2 * sin(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * sin(alpha[3]) + 0.357e2 * sin(alpha[0]) * sin(alpha[1]) * cos(alpha[3]) + 0.4e0 * sin(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + 0.4e0 * sin(alpha[0]) * sin(alpha[1]) * sin(alpha[3]) - 0.18e1 * sin(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) + 0.342e2 * sin(alpha[0]) * sin(alpha[1]);
	(*J).at<double>(0,2) = -0.16e2 * ((-cos(alpha[0]) * sin(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[3]) * cos(alpha[4]) + (-cos(alpha[0]) * cos(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) - 0.16e2 * (-cos(alpha[0]) * sin(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[3]) * cos(alpha[5]) - 0.16e1 * (-cos(alpha[0]) * sin(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[3]) * cos(alpha[4]) - 0.16e1 * (-cos(alpha[0]) * cos(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[4]) - 0.357e2 * (-cos(alpha[0]) * sin(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[3]) + 0.4e0 * (-cos(alpha[0]) * sin(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[3]) + 0.18e1 * cos(alpha[0]) * sin(alpha[2]) - 0.18e1 * sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2]);
	(*J).at<double>(0,3) = -0.16e2 * (-(cos(alpha[0]) * cos(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) - sin(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[4]) * sin(alpha[5]) - 0.16e2 * ((cos(alpha[0]) * cos(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - sin(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[5]) - 0.16e1 * (-(cos(alpha[0]) * cos(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) - sin(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[4]) - 0.357e2 * (cos(alpha[0]) * cos(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) + 0.357e2 * sin(alpha[0]) * cos(alpha[1]) * sin(alpha[3]) - 0.4e0 * (cos(alpha[0]) * cos(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) - 0.4e0 * sin(alpha[0]) * cos(alpha[1]) * cos(alpha[3]);
	(*J).at<double>(0,4) = -0.16e2 * (-((cos(alpha[0]) * cos(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - sin(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) + (-cos(alpha[0]) * sin(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[4])) * sin(alpha[5]) + 0.16e1 * ((cos(alpha[0]) * cos(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - sin(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) - 0.16e1 * (-cos(alpha[0]) * sin(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[4]);
	(*J).at<double>(0,5) = -0.16e2 * (((cos(alpha[0]) * cos(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - sin(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (-cos(alpha[0]) * sin(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * cos(alpha[5]) + 0.16e2 * ((cos(alpha[0]) * cos(alpha[2]) + sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) + sin(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * sin(alpha[5]);
	(*J).at<double>(1,0) = 0;
	(*J).at<double>(1,1) = -0.16e2 * ((-sin(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) - sin(alpha[1]) * cos(alpha[2]) * sin(alpha[4])) * sin(alpha[5]) - 0.16e2 * (-sin(alpha[1]) * sin(alpha[2]) * sin(alpha[3]) - cos(alpha[1]) * cos(alpha[3])) * cos(alpha[5]) - 0.16e1 * (-sin(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + 0.16e1 * sin(alpha[1]) * cos(alpha[2]) * sin(alpha[4]) + 0.357e2 * sin(alpha[1]) * sin(alpha[2]) * sin(alpha[3]) + 0.357e2 * cos(alpha[1]) * cos(alpha[3]) - 0.4e0 * sin(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + 0.4e0 * cos(alpha[1]) * sin(alpha[3]) + 0.18e1 * sin(alpha[1]) * sin(alpha[2]) + 0.342e2 * cos(alpha[1]);
	(*J).at<double>(1,2) = -0.16e2 * (cos(alpha[1]) * cos(alpha[2]) * cos(alpha[3]) * cos(alpha[4]) - cos(alpha[1]) * sin(alpha[2]) * sin(alpha[4])) * sin(alpha[5]) - 0.16e2 * cos(alpha[1]) * cos(alpha[2]) * sin(alpha[3]) * cos(alpha[5]) - 0.16e1 * cos(alpha[1]) * cos(alpha[2]) * cos(alpha[3]) * cos(alpha[4]) + 0.16e1 * cos(alpha[1]) * sin(alpha[2]) * sin(alpha[4]) - 0.357e2 * cos(alpha[1]) * cos(alpha[2]) * sin(alpha[3]) + 0.4e0 * cos(alpha[1]) * cos(alpha[2]) * cos(alpha[3]) - 0.18e1 * cos(alpha[1]) * cos(alpha[2]);
	(*J).at<double>(1,3) = -0.16e2 * (-cos(alpha[1]) * sin(alpha[2]) * sin(alpha[3]) + sin(alpha[1]) * cos(alpha[3])) * cos(alpha[4]) * sin(alpha[5]) - 0.16e2 * (cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + sin(alpha[1]) * sin(alpha[3])) * cos(alpha[5]) - 0.16e1 * (-cos(alpha[1]) * sin(alpha[2]) * sin(alpha[3]) + sin(alpha[1]) * cos(alpha[3])) * cos(alpha[4]) - 0.357e2 * cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) - 0.357e2 * sin(alpha[1]) * sin(alpha[3]) - 0.4e0 * cos(alpha[1]) * sin(alpha[2]) * sin(alpha[3]) + 0.4e0 * sin(alpha[1]) * cos(alpha[3]);
	(*J).at<double>(1,4) = -0.16e2 * (-(cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + sin(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) + cos(alpha[1]) * cos(alpha[2]) * cos(alpha[4])) * sin(alpha[5]) + 0.16e1 * (cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + sin(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) - 0.16e1 * cos(alpha[1]) * cos(alpha[2]) * cos(alpha[4]);
	(*J).at<double>(1,5) = -0.16e2 * ((cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + sin(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + cos(alpha[1]) * cos(alpha[2]) * sin(alpha[4])) * cos(alpha[5]) + 0.16e2 * (cos(alpha[1]) * sin(alpha[2]) * sin(alpha[3]) - sin(alpha[1]) * cos(alpha[3])) * sin(alpha[5]);
	(*J).at<double>(2,0) = -0.16e2 * (((-cos(alpha[0]) * cos(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) + sin(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (cos(alpha[0]) * sin(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) - 0.16e2 * ((-cos(alpha[0]) * cos(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) - sin(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[5]) - 0.16e1 * ((-cos(alpha[0]) * cos(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) + sin(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) - 0.16e1 * (cos(alpha[0]) * sin(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4]) - 0.357e2 * (-cos(alpha[0]) * cos(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) + 0.357e2 * sin(alpha[0]) * cos(alpha[1]) * cos(alpha[3]) + 0.4e0 * (-cos(alpha[0]) * cos(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) + 0.4e0 * sin(alpha[0]) * cos(alpha[1]) * sin(alpha[3]) + 0.18e1 * cos(alpha[0]) * cos(alpha[2]) + 0.18e1 * sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2]) + 0.342e2 * sin(alpha[0]) * cos(alpha[1]) - 0.15e1 * cos(alpha[0]) + 0.3e1 * sin(alpha[0]);
	(*J).at<double>(2,1) = -0.16e2 * ((cos(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[2]) * sin(alpha[4])) * sin(alpha[5]) - 0.16e2 * (cos(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * sin(alpha[3]) - cos(alpha[0]) * sin(alpha[1]) * cos(alpha[3])) * cos(alpha[5]) - 0.16e1 * (cos(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) - 0.16e1 * cos(alpha[0]) * cos(alpha[1]) * cos(alpha[2]) * sin(alpha[4]) - 0.357e2 * cos(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * sin(alpha[3]) + 0.357e2 * cos(alpha[0]) * sin(alpha[1]) * cos(alpha[3]) + 0.4e0 * cos(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + 0.4e0 * cos(alpha[0]) * sin(alpha[1]) * sin(alpha[3]) - 0.18e1 * cos(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) + 0.342e2 * cos(alpha[0]) * sin(alpha[1]);
	(*J).at<double>(2,2) = -0.16e2 * ((sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[3]) * cos(alpha[4]) + (sin(alpha[0]) * cos(alpha[2]) - cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) - 0.16e2 * (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[3]) * cos(alpha[5]) - 0.16e1 * (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[3]) * cos(alpha[4]) - 0.16e1 * (sin(alpha[0]) * cos(alpha[2]) - cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[4]) - 0.357e2 * (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[3]) + 0.4e0 * (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[3]) - 0.18e1 * sin(alpha[0]) * sin(alpha[2]) - 0.18e1 * cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2]);
	(*J).at<double>(2,3) = -0.16e2 * (-(-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[4]) * sin(alpha[5]) - 0.16e2 * ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[5]) - 0.16e1 * (-(-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[4]) - 0.357e2 * (-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) + 0.357e2 * cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3]) - 0.4e0 * (-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) - 0.4e0 * cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3]);
	(*J).at<double>(2,4) = -0.16e2 * (-((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[4])) * sin(alpha[5]) + 0.16e1 * ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) - 0.16e1 * (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[4]);
	(*J).at<double>(2,5) = -0.16e2 * (((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * cos(alpha[5]) + 0.16e2 * ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * sin(alpha[5]);
	(*J).at<double>(3,0) = ((((-cos(alpha[0]) * cos(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) + sin(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (cos(alpha[0]) * sin(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) + ((-cos(alpha[0]) * cos(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) - sin(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[5])) * pow(0.1e1 - pow((((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) + ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[5]), 2), -0.5);
	(*J).at<double>(3,1) = (((cos(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[2]) * sin(alpha[4])) * sin(alpha[5]) + (cos(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * sin(alpha[3]) - cos(alpha[0]) * sin(alpha[1]) * cos(alpha[3])) * cos(alpha[5])) * pow(0.1e1 - pow((((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) + ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[5]), 2), -0.5);
	(*J).at<double>(3,2) = (((sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[3]) * cos(alpha[4]) + (sin(alpha[0]) * cos(alpha[2]) - cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[3]) * cos(alpha[5])) * pow(0.1e1 - pow((((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) + ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[5]), 2), -0.5);
	(*J).at<double>(3,3) = ((-(-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[4]) * sin(alpha[5]) + ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[5])) * pow(0.1e1 - pow((((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) + ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[5]), 2), -0.5);
	(*J).at<double>(3,4) = (-((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[4])) * sin(alpha[5]) * pow(0.1e1 - pow((((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) + ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[5]), 2), -0.5);
	(*J).at<double>(3,5) = ((((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * cos(alpha[5]) - ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * sin(alpha[5])) * pow(0.1e1 - pow((((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * sin(alpha[5]) + ((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * cos(alpha[5]), 2), -0.5);
	(*J).at<double>(4,0) = -(-((-cos(alpha[0]) * cos(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) + sin(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) + (cos(alpha[0]) * sin(alpha[2]) - sin(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[4])) * pow(0.1e1 - pow(-((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[4]), 2), -0.5);
	(*J).at<double>(4,1) = -(-(cos(alpha[0]) * cos(alpha[1]) * sin(alpha[2]) * cos(alpha[3]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) + cos(alpha[0]) * cos(alpha[1]) * cos(alpha[2]) * cos(alpha[4])) * pow(0.1e1 - pow(-((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[4]), 2), -0.5);
	(*J).at<double>(4,2) = -(-(sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[3]) * sin(alpha[4]) + (sin(alpha[0]) * cos(alpha[2]) - cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[4])) * pow(0.1e1 - pow(-((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[4]), 2), -0.5);
	(*J).at<double>(4,3) = (-(-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * sin(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * cos(alpha[3])) * sin(alpha[4]) * pow(0.1e1 - pow(-((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[4]), 2), -0.5);
	(*J).at<double>(4,4) = -(-((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * cos(alpha[4]) - (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * sin(alpha[4])) * pow(0.1e1 - pow(-((-sin(alpha[0]) * cos(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * sin(alpha[2])) * cos(alpha[3]) - cos(alpha[0]) * cos(alpha[1]) * sin(alpha[3])) * sin(alpha[4]) + (sin(alpha[0]) * sin(alpha[2]) + cos(alpha[0]) * sin(alpha[1]) * cos(alpha[2])) * cos(alpha[4]), 2), -0.5);
	(*J).at<double>(4,5) = 0;
}
/*
void true_state(sensor_msgs::JointState msg){

	arm_state[0] = -msg.position[0];
	arm_state[1] = msg.position[1];
	arm_state[2] = msg.position[2];
	arm_state[3] = -msg.position[3];
	arm_state[4] = msg.position[4];
	arm_state[5] = msg.position[5];

	HOLD = false;

	return;
}
*/

int leerPoseServo(double *alpha)
{
	//FUNCION DE ROS PARA LEER LOS ANGULOS DE LOS DISTINTOS MOTORES DEL BRAZO INVOLUCRADOS EN PLANIFICACION

	bender_planning_old::States srv;

	srv.request.select.resize(8);
	for(int i=0; i<6;i++)
		srv.request.select[i]=true;
	for(int i=6; i<8;i++)
		srv.request.select[i]=false;
	//ROS_INFO("Request para lectura de poses rellenado");

	if (state_client.call(srv))
	{
		//ROS_INFO("Llamada a servicio de lectura de poses exitosa");
		alpha[0]=srv.response.state[0];
		alpha[1]=-srv.response.state[1];
		alpha[2]=srv.response.state[2];
		alpha[3]=-srv.response.state[3];
		alpha[4]=-srv.response.state[4];
		alpha[5]=srv.response.state[5];
		ROS_INFO("Pose motores: %f %f %f %f %f %f",alpha[0],alpha[1],alpha[2],alpha[3],alpha[4],alpha[5]);
	}
	else
	{
		ROS_ERROR("Failed to call service right_arm_joints/states in Uarm::leerPoseServo");
	return 1;
	}
	return 0;
}

bool leerMoving(string name, ros::NodeHandle nh)
{
	//Si el motor está moviendose retorna un boleano verdadero(1)
	//La entrada es el nombre del motor

	//FUNCION DE ROS PARA OBTENER SI EL MOTOR SE ESTÁ MOVIENDO
	while(!ros::service::waitForService("left_arm_joints/states", ros::Duration(1.0)))
	{
		ROS_INFO_STREAM("Waiting for planner service left_arm_joints/states");
	}
	ros::ServiceClient client = nh.serviceClient<bender_planning_old::States>("left_arm_joints/states");
	bender_planning_old::States srv;
	srv.request.select.resize(8);
	string names[8]={"hombro_1","hombro_2","hombro_3","codo_1","codo_2","muneca","dedo_2","dedo_1"};
	int motor_pos = -1;
	for(int i=0; i<8;i++)
	{
		if (names[i].compare(name)==0)
		{
			motor_pos = i;
			srv.request.select[i] = true;
		}
		else
			srv.request.select[i]=false;
	}
	if (client.call(srv))
	{
		return srv.response.is_moving[motor_pos];
	}
	else
	{
		ROS_ERROR("Failed to call service left_arm_joints/states in Uarm::leerMoving");
	}

	return 0;
}
bool BrazoMov(ros::NodeHandle nh)
{
	return (leerMoving("hombro_1",nh) ||
			 leerMoving("hombro_2",nh) ||
			 leerMoving("muneca",nh) ||
			 leerMoving("hombro_3",nh) ||leerMoving("codo_1",nh) ||
			 leerMoving("dedo_2",nh) || leerMoving("dedo_1",nh));

}

bool set_goal(kinematics::PlanIKGoal::Request &req, kinematics::PlanIKGoal::Response &res)
{
	goal[0] = req.x;
	goal[1] = req.y;
	goal[2] = req.z;
	goal[3] = req.pitch;
	goal[4] = req.roll;

	CHANGE_GOAL = true;
	HOLD = false;

	return true;
}

bool cancel_goal(bender_planning_old::Dummy::Request &req, bender_planning_old::Dummy::Response &res)
{
	HOLD = true;

	return true;
}

int main(int argc, char **argv){

	//inicializacion del nodo, publicaciones y suscripciones
	ros::init(argc, argv, "iknode");

	ros::NodeHandle nh;

	while(!ros::service::waitForService("left_arm_joints/states", ros::Duration(1.0)))
	{
		ROS_INFO_STREAM("Waiting for planner service left_arm_joints/states");
	}

	//ros::Subscriber state_sub = nh.subscribe("/joint_states",1000,true_state);
	state_client = nh.serviceClient<bender_planning_old::States>("left_arm_joints/states");
	while(leerPoseServo(arm_state))
		;
	state_pub = nh.advertise<bender_planning_old::Command>("left_arm_joints/command", 1000);
	ros::ServiceClient limit_torque = nh.serviceClient<arm_controller::TorqueLimit>("left_arm_joints/torque_limit",1000);

	ros::ServiceServer plan_service = nh.advertiseService("left_arm/set_planning_goal",set_goal);
	ros::ServiceServer plan_cancel = nh.advertiseService("left_arm/cancel_planning_goal",cancel_goal);

	ros::Publisher hombro_1_pub = nh.advertise<std_msgs::Float64>("hombro_1_controller/command",1000);
	ros::Publisher codo_1_pub = nh.advertise<std_msgs::Float64>("codo_1_controller/command",1000);
	ros::ServiceClient motor_speed = nh.serviceClient<arm_controller::Speed>("left_arm_joints/set_speed",1000);

	ros::Rate loop_rate(30);

	//parametro de suavidad de la respuesta a perturbacion
	double eta = 0.5;
	//definicion de desviaciones y torques limite para el compliance del manipulador
	double dl[5];
	double dh[5];
	double tl[5];
	double th[5];

	dl[0] = 1;
	dh[0] = 10;
	dl[1] = 1;
	dh[1] = 10;
	dl[2] = 0;
	dh[2] = 5;
	dl[3] = 0;
	dh[3] = 0.1;
	dl[4] = 0.02;
	dh[4] = 0.2;

	tl[0] = 0.2;
	th[0] = 0.7;
	tl[1] = 0.2;
	th[1] = 0.6;
	tl[2] = 0.6;
	th[2] = 1;
	tl[3] = 0.5;
	th[3] = 1;
	tl[4] = 0.2;
	th[4] = 0.6;
	double tlimnull = 0.5;

	string names[6]={"hombro_1","hombro_2","hombro_3","codo_1","codo_2","muneca"};

	//ciclo principal del programa
	//variable de término del algoritmo de movimiento
	bool FINISHED = false;
	while (ros::ok())
	{
		//espera a leer el estado del brazo y recibir una peticion de planificacion
		while ((HOLD || !CHANGE_GOAL))
		{
			if(!ros::ok())
				return 1;
			ros::spinOnce();

			loop_rate.sleep();
		}
		//--->comienza la planificacion<---
		CHANGE_GOAL = false;
		//definicion de variables
		double xf[5];
		xf[0] = goal[0];
		xf[1] = goal[1];
		xf[2] = goal[2];
		xf[3] = goal[3];
		xf[4] = goal[4];

		double kx = 0.9;
		double kq = 2;

		Mat J;
		Mat Jpinv;
		Mat opt2;
		Mat xi;
		Mat x_act;
		Mat dx;
		Mat dq;
		Mat q_act = Mat::zeros(6,1,CV_64F);
		Mat xd = Mat::zeros(5,1,CV_64F);
		Mat xd_;
		Mat dg = Mat::zeros(6,1,CV_64F);

		Mat Rtask;
		Mat R0;
		Mat R;

		while(leerPoseServo(arm_state))
			;
		left_forward_kinematics(arm_state[0],arm_state[1],arm_state[2],arm_state[3],arm_state[4],arm_state[5],&xi);
		//leerPoseServo(alpha);
		jacobian(arm_state,&J);

		int count = 0;
		int n = (int)sqrt(pow(xf[0]-xi.at<double>(0,0),2)+pow(xf[1]-xi.at<double>(1,0),2)+pow(xf[2]-xi.at<double>(2,0),2))*2;

		//ciclo de poses intermedias a alcanzar
		while (ros::ok())
		{
			//---------->CINEMATICA INVERSA<----------

			if(CHANGE_GOAL || HOLD)
				break;
			if (count < n)
			{
				count++;
				ROS_INFO("count = %d",count);
			}
			double p =double(count)/n;
			//pose actual
			q_act.at<double>(0,0) = arm_state[0];
			q_act.at<double>(1,0) = arm_state[1];
			q_act.at<double>(2,0) = arm_state[2];
			q_act.at<double>(3,0) = arm_state[3];
			q_act.at<double>(4,0) = arm_state[4];
			q_act.at<double>(5,0) = arm_state[5];

			ROS_INFO("Pose actual: %f, %f, %f, %f, %f, %f",q_act.at<double>(0,0),q_act.at<double>(1,0),q_act.at<double>(2,0),q_act.at<double>(3,0),q_act.at<double>(4,0),q_act.at<double>(5,0));

			while(leerPoseServo(arm_state) && ros::ok())
				;
			left_forward_kinematics(arm_state[0],arm_state[1],arm_state[2],arm_state[3],arm_state[4],arm_state[5],&x_act);
			ROS_INFO("Cinematica Directa: %f, %f, %f, %f, %f",x_act.at<double>(0,0),x_act.at<double>(1,0),x_act.at<double>(2,0),x_act.at<double>(3,0),x_act.at<double>(4,0));

			//jacobiano
			jacobian(arm_state,&J);

			//gradiente del criterio de optimizacion secundario
			dg.at<double>(0,0) = (q_act.at<double>(0,0))/(M_PI);
			dg.at<double>(1,0) = (q_act.at<double>(1,0))/(M_PI/2);
			dg.at<double>(2,0) = (q_act.at<double>(2,0))/(M_PI);
			dg.at<double>(3,0) = (q_act.at<double>(3,0))/(M_PI);
			dg.at<double>(4,0) = (q_act.at<double>(4,0))/(M_PI);
			dg.at<double>(5,0) = (q_act.at<double>(5,0))/(M_PI);

			//ROS_INFO("Gradiente de g: %f, %f, %f, %f, %f, %f",dg.at<double>(0,0),dg.at<double>(1,0),dg.at<double>(2,0),dg.at<double>(3,0),dg.at<double>(4,0),dg.at<double>(5,0));

			//pose intermedia
			xd.at<double>(0,0) = p*xf[0] + (1-p)*xi.at<double>(0,0);
			xd.at<double>(1,0) = p*xf[1] + (1-p)*xi.at<double>(1,0);
			xd.at<double>(2,0) = p*xf[2] + (1-p)*xi.at<double>(2,0);
			xd.at<double>(3,0) = p*xf[3] + (1-p)*xi.at<double>(3,0);
			xd.at<double>(4,0) = p*xf[4] + (1-p)*xi.at<double>(4,0);
			//ROS_INFO("Pose Objetivo: %f, %f, %f, %f, %f",xd.at<double>(0,0),xd.at<double>(1,0),xd.at<double>(2,0),xd.at<double>(3,0),xd.at<double>(4,0));

			//pose objetivo ponderada
			xd_ = xd + eta*(x_act-xd);

			//diferencia entre pose actual e intermedia (velocidad)
			dx = kx*(xd - x_act);
			ROS_INFO("delta x: %f, %f, %f, %f, %f",dx.at<double>(0,0),dx.at<double>(1,0),dx.at<double>(2,0),dx.at<double>(3,0),dx.at<double>(4,0));

			//pseudoinverso del jacobiano
			Jpinv = J.inv(DECOMP_SVD);

			//termino para considerar optimizacion del criterio secundario
			opt2 = -1.0/40*(Mat::eye(6,6,CV_64F) - Jpinv*J)*dg;

			//variacion requerida en los angulos de los motores
			dq = kq*(Jpinv*dx + opt2);
			//dq = kq*Jpinv*dx;
			ROS_INFO("delta q: %f, %f, %f, %f, %f, %f",dq.at<double>(0,0),dq.at<double>(1,0),dq.at<double>(2,0),dq.at<double>(3,0),dq.at<double>(4,0),dq.at<double>(5,0));


			//---------->COMPLIANCE<----------

			//calculo del compliance en cada eje dependiendo de la desviacion
			//ROS_INFO("calculando compliance");
			dx = 1/kx*dx;
			double c[5];
			for(int i = 0; i<5; i++)
				c[i] = min(1.0,max(0.0,1.0-(dx.at<double>(i,0)-dl[i])/(dh[i]-dl[i])));
			//calculo del torque en cada eje dependiendo del compliance
			//ROS_INFO("calculando torque en cada eje");
			Mat tlimtask = Mat::zeros(6,1,CV_64F);
			for(int i = 0; i<5; i++)
				tlimtask.at<double>(i,0) = c[i]*tl[i]+(1.0-c[i])*th[i];
			tlimtask.at<double>(5,0) = tlimnull;
			//calculo de las matrices de importancia para limitacion del torque
			//ROS_INFO("calculando matrices de importancia");
			Mat st_vel = Mat::eye(5,5,CV_64F);
			for(int i = 0; i<st_vel.rows; i++)
				st_vel.at<double>(i,i) = dx.at<double>(i,0);
			Rtask = abs(Jpinv*st_vel);
			R0 = abs(opt2);
			hconcat(Rtask,R0,R);
			//normalizacion de la matriz de importancia
			//ROS_INFO("normalizacion de la matriz de importancia");
			for(int j = 0; j<R.cols; j++)
			{
				double sum = 0;
				for(int i = 0; i<R.rows; i++)
				{
					sum = sum + R.at<double>(i,j);
				}
				for(int i = 0; i<R.rows; i++)
				{
					R.at<double>(i,j) = 1.0/sum * R.at<double>(i,j)	;
				}
			}
			//calculo del torque limite necesario en cada motor dependiendo del escogido para cada eje
			//ROS_INFO("calculo del torque limite en cada motor");
			Mat tlim;
			tlim = R*tlimtask;


			//publicacion del nuevo estado de los motores
			bender_planning_old::Command com;

			com.positions.resize(8);
			com.select.resize(8);
			com.speed.resize(8);

			for(int i=0;i<8;i++)
			{
				if(i<6)
				{
					com.positions[i]=m_sign[i]*(q_act.at<double>(i,0) + dq.at<double>(i,0));
					com.speed[i]=0.1;
					com.select[i]=true;
				}
				else
				{
					com.positions[i]=0;
					com.speed[i]=0;
					com.select[i]=false;
				}
			}

			state_pub.publish(com);

			//asignacion de los limites de torque
			/*
			arm_controller::TorqueLimit tlim_msg;
			for(int i=0;i<6;i++)
			{
				tlim_msg.request.motor_name = names[i];
				tlim_msg.request.torque_limit = tlim.at<double>(i,0);
				ROS_INFO("limites de torque: %f, %f, %f, %f, %f, %f",tlim.at<double>(0,0),tlim.at<double>(1,0),tlim.at<double>(2,0),tlim.at<double>(3,0),tlim.at<double>(4,0),tlim.at<double>(5,0));
				if(!limit_torque.call(tlim_msg))
					ROS_ERROR("Unable to call service left_arm_joints/torque_limit");
			}
			*/


			//ros::spinOnce();
			if(count == n && !FINISHED && false)
			{
				FINISHED = true;
				double qf[6];
				while(leerPoseServo(qf) && ros::ok())
					;
				//se aumenta y luego disminuye el valor de hombro_1 para dismimuir el esfuerzo y corregir la caída del brazo por gravedad
				arm_controller::Speed sp;
				sp.request.motor_name = "hombro_1";
				sp.request.speed = 0.05;
				motor_speed.call(sp);
				std_msgs::Float64 q_plus;
				q_plus.data = m_sign[0]*(qf[0]-0.04);
				hombro_1_pub.publish(q_plus);
				while(leerMoving("hombro_1",nh) && ros::ok())
					;
				/*
				q_plus.data = m_sign[0]*(qf[0]-0.05);
				hombro_1_pub.publish(q_plus);
				while(leerMoving("hombro_1",nh) && ros::ok())
					;
				*/
			}

			loop_rate.sleep();

			ros::spinOnce();
		}
		ros::spinOnce();
	}

	return 0;
}

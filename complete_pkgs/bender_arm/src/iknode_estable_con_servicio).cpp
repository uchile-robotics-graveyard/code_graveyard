#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "bender_planning_old/States.h"
#include "kinematics/PlanIKGoal.h"
#include "BenderModel/ArmState.h"
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <iostream>

#include "math.h"

using namespace std;
using namespace cv;

double arm_state[6] = {0,0,0,0,0,-0.01};
bool HOLD = true;

ros::Publisher state_pub;

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
	H1.at<double>(2,3) = 123;

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

/*
int leerPoseServo(double *alpha)
{
	alpha[0]=0.0;
	alpha[1]=0.0;
	alpha[2]=0.0;
	alpha[3]=0.0;
	//FUNCION DE ROS PARA LEER LOS ANGULOS DE LOS DISTINTOS MOTORES DEL BRAZO INVOLUCRADOS EN PLANIFICACION
	while(!ros::service::waitForService("right_arm_joints/states", ros::Duration(1.0)))
	{
		ROS_INFO_STREAM("Waiting for planner service right_arm_joints/states");
	}
	//ROS_INFO("Servicio right_arm_joints/states encontrado");
	ros::ServiceClient client = nh.serviceClient<bender_planning_old::States>("right_arm_joints/states");
	//ROS_INFO("Cliente para lectura de poses iniciado");
	bender_planning_old::States srv;
	srv.request.select.resize(8);
	for(int i=0; i<6;i++)
		srv.request.select[i]=true;
	for(int i=6; i<8;i++)
		srv.request.select[i]=false;
	//ROS_INFO("Request para lectura de poses rellenado");

	if (client.call(srv))
	{
		//ROS_INFO("Llamada a servicio de lectura de poses exitosa");
		alpha[0]=srv.response.state[0];
		alpha[1]=srv.response.state[1];
		alpha[2]=srv.response.state[2];
		alpha[3]=srv.response.state[3];
		alpha[4]=srv.response.state[4];
		alpha[5]=srv.response.state[5];
	}
	else
	{
		ROS_ERROR("Failed to call service right_arm_joints/states in Uarm::leerPoseServo");
	return 1;
	}
	return 0;
}
*/

bool plan_ik(kinematics::PlanIKGoal::Request &req, kinematics::PlanIKGoal::Response &res)
{
	double xf[5];
	xf[0] = req.x;
	xf[1] = req.y;
	xf[2] = req.z;
	xf[3] = req.pitch;
	xf[4] = req.roll;

	ros::Rate loop_rate(30);

	double kx = 0.5;
	double kq = 2;

	Mat J;
	Mat Jpinv;
	Mat xi;
	Mat x_act;
	Mat q_act = Mat::zeros(6,1,CV_64F);
	Mat xd = Mat::zeros(5,1,CV_64F);
	Mat dg = Mat::zeros(6,1,CV_64F);

	left_forward_kinematics(arm_state[0],arm_state[1],arm_state[2],arm_state[3],arm_state[4],arm_state[5],&xi);
	//leerPoseServo(alpha);
	jacobian(arm_state,&J);

	int count = 0;
	int n = 200;

	while (HOLD)
	{
		ros::spinOnce();

		loop_rate.sleep();
	}

	while (ros::ok() && count < n)
	{
		count++;
		double p =double(count)/n;
		q_act.at<double>(0,0) = arm_state[0];
		q_act.at<double>(1,0) = arm_state[1];
		q_act.at<double>(2,0) = arm_state[2];
		q_act.at<double>(3,0) = arm_state[3];
		q_act.at<double>(4,0) = arm_state[4];
		q_act.at<double>(5,0) = arm_state[5];

		ROS_INFO("Pose actual: %f, %f, %f, %f, %f, %f",q_act.at<double>(0,0),q_act.at<double>(1,0),q_act.at<double>(2,0),q_act.at<double>(3,0),q_act.at<double>(4,0),q_act.at<double>(5,0));

		left_forward_kinematics(arm_state[0],arm_state[1],arm_state[2],arm_state[3],arm_state[4],arm_state[5],&x_act);
		//ROS_INFO("Cinematica Directa: %f, %f, %f, %f, %f",x_act.at<double>(0,0),x_act.at<double>(1,0),x_act.at<double>(2,0),x_act.at<double>(3,0),x_act.at<double>(4,0));
		jacobian(arm_state,&J);

		dg.at<double>(0,0) = (q_act.at<double>(0,0))/(M_PI);
		dg.at<double>(1,0) = (q_act.at<double>(1,0) - M_PI/4)/(M_PI/2);
		dg.at<double>(2,0) = (q_act.at<double>(2,0) + M_PI/4)/(M_PI);
		dg.at<double>(3,0) = (q_act.at<double>(3,0))/(M_PI);
		dg.at<double>(4,0) = (q_act.at<double>(4,0))/(M_PI);
		dg.at<double>(5,0) = (q_act.at<double>(5,0))/(M_PI);

		//ROS_INFO("Gradiente de g: %f, %f, %f, %f, %f, %f",dg.at<double>(0,0),dg.at<double>(1,0),dg.at<double>(2,0),dg.at<double>(3,0),dg.at<double>(4,0),dg.at<double>(5,0));

		xd.at<double>(0,0) = p*xf[0] + (1-p)*xi.at<double>(0,0);
		xd.at<double>(1,0) = p*xf[1] + (1-p)*xi.at<double>(1,0);
		xd.at<double>(2,0) = p*xf[2] + (1-p)*xi.at<double>(2,0);
		xd.at<double>(3,0) = p*xf[3] + (1-p)*xi.at<double>(3,0);
		xd.at<double>(4,0) = p*xf[4] + (1-p)*xi.at<double>(4,0);
		//ROS_INFO("Pose Objetivo: %f, %f, %f, %f, %f",xd.at<double>(0,0),xd.at<double>(1,0),xd.at<double>(2,0),xd.at<double>(3,0),xd.at<double>(4,0));

		Mat dx;
		Mat dq;
		dx = kx*(xd - x_act);
		//ROS_INFO("delta x: %f, %f, %f, %f, %f",dx.at<double>(0,0),dx.at<double>(1,0),dx.at<double>(2,0),dx.at<double>(3,0),dx.at<double>(4,0));

		//cout << "J = "<< endl << " "  << J << endl << endl;
		Jpinv = J.inv(DECOMP_SVD);
		//cout << "Jpinv = "<< endl << " "  << Jpinv << endl << endl;
		dq = kq*(Jpinv*dx - 1.0/40*(Mat::eye(6,6,CV_64F) - Jpinv*J)*dg);
		//dq = kq*Jpinv*dx;
		//ROS_INFO("delta q: %f, %f, %f, %f, %f, %f",dq.at<double>(0,0),dq.at<double>(1,0),dq.at<double>(2,0),dq.at<double>(3,0),dq.at<double>(4,0),dq.at<double>(5,0));


		BenderModel::ArmState pub_msg;
		pub_msg.Joint1 = q_act.at<double>(0,0) + dq.at<double>(0,0);
		pub_msg.Joint2 = q_act.at<double>(1,0) + dq.at<double>(1,0);
		pub_msg.Joint3 = q_act.at<double>(2,0) + dq.at<double>(2,0);
		pub_msg.Joint4 = q_act.at<double>(3,0) + dq.at<double>(3,0);
		pub_msg.Joint5 = q_act.at<double>(4,0) + dq.at<double>(4,0);
		pub_msg.Joint6 = q_act.at<double>(5,0) + dq.at<double>(5,0);


		//ROS_INFO("Orden Actuadores: %f, %f, %f, %f, %f, %f",pub_msg.Joint1,pub_msg.Joint2,pub_msg.Joint3,pub_msg.Joint4,pub_msg.Joint5,pub_msg.Joint6);

		state_pub.publish(pub_msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "iknode");

	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe("/joint_states",1000,true_state);
	state_pub = nh.advertise<BenderModel::ArmState>("/PosicionActuadoresLeftArm", 1000);
	ros::ServiceServer plan_service = nh.advertiseService("left_arm/plan_ik",plan_ik);

	ros::spin();

	return 0;
}

#ifndef LEFT_CINEMATICA_H_
#define LEFT_CINEMATICA_H_

// Includes

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <bender_msgs/Command.h>
#include <bender_srvs/States.h>
#include <bender_srvs/Onoff.h>
#include <bender_srvs/TorqueEnable.h>
#include <bender_srvs/TorqueLimit.h>

using namespace std;

// Defines

#define g 9.8
#define pi 3.14159

//Functions

void LeerAngulo(double th[]);
void LeerCarga(double carga[]);
void GetGravity(double gg[], int joint,double angulos[]);
void DK_gravity(double xyz[], int link);
void DK_joint(double xyz[], double angulos[], int joint);
void DK_matrix(double angulos[], int joint);
void CenterGravity(double xyz[], double angulos[], double M[], int link);
void GetPlano();
double MAcumulada(double M[], int link);
void CargaMotoresCalc(double torque[],double th[], double M[]);
void SoltarJoint(double torque[], double carga[]);
bool LimitarTorqueMax(double limiteTorque[], int joint);
bool SetPosicion(double th[], bool select[]);

//Variables

cv::Mat H[7];
cv::Mat CM[7];
cv::Mat a,x0,homogenea;

ros::NodeHandle * n;
ros::ServiceClient call_torque;
ros::ServiceClient torque_limit;
ros::ServiceClient torque_on;
ros::Publisher call_pose;

bender_srvs::TorqueLimit tl;
bender_msgs::Command pose;
bender_srvs::TorqueEnable tor;
bender_srvs::Onoff onoff;

double left_pose[8], left_limite[8], right_pose[8], right_limite[8];
double th[8],carga[8];
bool joint_select[8];




#endif /* LEFT_CINEMATICA_H_ */

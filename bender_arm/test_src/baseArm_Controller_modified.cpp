#include "ros/ros.h"
#include "bender_msgs/baseArmState.h"
#include "bender_msgs/baseEfectorPosition.h"
#include "bender_srvs/setAnglesBaseArm.h"
#include "bender_srvs/setGoalBaseArm.h"
#include "bender_srvs/manipBaseArm.h"
#include "bender_srvs/orientBaseArm.h"
#include "bender_msgs/grip.h"
#include "bender_srvs/LoadMode.h"
#include <opencv/cv.hpp>
#include <math.h>
#include "bender_msgs/Command.h"
#include "bender_srvs/Dummy.h"
#include "bender_srvs/States.h"
#include "bender_srvs/AngVel.h"

//copiar el siguiente código en el .launch para que ejecute este nodo también
//<node name="baseArm_Controller"    pkg="bottomArmController" type="baseArm_Controller" />

/****** Variables Globales ******/
float armState[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float simArmState[6] = { 0.0, -1.57, 1.6, 0.0, -0.09, 0.09 };
float efector[3] = { 0.0, 0.0, 0.0 };
float goalAngles[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float initpose[6] = { 0.0, -1.57, 1.6, -0.0, -0.09, 0.09 };
float goal[3] = { 0.0, 0.0, 0.0 };
float Goal[3] = {0.0,0.0,0.0};
float angleMaxMin[2][6] = { { 0.63, 1.68, 2.29, 1.61, -0.08, 1.1 }, { -0.82,
		-1.62, -2.23, -3.35, -1.1, 0.08 } };
cv::Mat jacobian = cv::Mat::eye(3, 3, CV_32F);
cv::Mat pInvJacobian = cv::Mat::eye(3, 3, CV_32F);
cv::Mat simulDAngles = cv::Mat::zeros(3, 1, CV_32F);

/************ Declaración de Funciones ************/

/*** servicios ***/
int setGoal(double g1, double g2, double g3);
bool setAngles(bender_srvs::setAnglesBaseArm::Request &req,	bender_srvs::setAnglesBaseArm::Response &res);
bool orientate(bender_srvs::orientBaseArm::Request &req,	bender_srvs::orientBaseArm::Response &res);
bool grasp(bender_srvs::manipBaseArm::Request &req,	bender_srvs::manipBaseArm::Response &res);
bool CerrarGripBrazoService(bender_srvs::manipBaseArm::Request &req,bender_srvs::manipBaseArm::Response &res);
bool GirarGripService(bender_srvs::AngVel::Request &req, bender_srvs::AngVel::Response &res);
bool MoverBrazoService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
bool PoseInicialService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
bool PlanificarEfectorService(bender_srvs::setGoalBaseArm::Request &req, bender_srvs::setGoalBaseArm::Response &res);
bool setGoalService(bender_srvs::setGoalBaseArm::Request &req,	bender_srvs::setGoalBaseArm::Response &res);

/*** funciones ***/
void CerrarGripBrazo(int loadMode);
void GirarGrip(double angulo);
void MoverBrazo();
void PoseInicial();
void MoverGripAng(double angulo, double velocidad);

/*** cinemática directa ***/
float getX(float O1, float O2, float O3);
float getY(float O1, float O2, float O3);
float getZ(float O1, float O2, float O3);

/*** cinemática Inversa ***/
void evaluateInverseKinematic(float O1, float O2, float O3);
void getControlAction(float dx, float dy, float dz, float O1, float O2,	float O3);

ros::ServiceClient agarre;
ros::ServiceClient plan;
ros::ServiceClient giro;
ros::ServiceClient ssa;
ros::ServiceClient stateClient;
ros::ServiceClient cliente;
ros::ServiceClient client;
ros::ServiceClient getStateClient;
ros::Publisher command_pub;
ros::Publisher baseArmState_pub;
ros::Publisher baseArmEfect_pub;
ros::Publisher bottomMotors_pub;
ros::Publisher grip_pub;

bender_srvs::States stateUpdate;
bender_msgs::baseArmState jointStates;
bender_msgs::baseEfectorPosition efectorMsg;
bender_msgs::Command cmdMsg;
bender_msgs::grip gripMsg;

/*** variables para actualizar los estados ***/
int laps = 0;
float maxDesp = 0.2;
float dAngles[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
bool selection[6] = { true, true, true, true, false, false };
float speeds[6] = { 0.3, 0.3, 0.3, 0.3, 0.3, 0.3 };
double var = 2;

/********** MAIN *********/
int main(int argc, char **argv) {

	/**** Inicializacion ****/
	ros::init(argc, argv, "baseArm_ControllerNode");
	ros::NodeHandle n;
	baseArmState_pub = n.advertise<bender_msgs::baseArmState>("PosicionActuadoresBaseArm", 2);
	baseArmEfect_pub = n.advertise<bender_msgs::baseEfectorPosition>("PosicionEfectorBaseArm", 2);
	bottomMotors_pub = n.advertise<bender_msgs::Command>("bottom_arm_joints/command", 2);
	grip_pub = n.advertise<bender_msgs::grip>("gripState", 1);

	command_pub = n.advertise<bender_msgs::Command>("bottom_arm_joints/command", 1000);
	client = n.serviceClient<bender_srvs::States>("bottom_arm_joints/states");
	cliente = n.serviceClient<bender_srvs::Dummy>("bottom_arm/actualizarPose");
	stateClient = n.serviceClient<bender_srvs::States>("bottom_arm_joints/states");
	ssa = n.serviceClient<bender_srvs::setGoalBaseArm>("setGoalBaseArm");
	plan = n.serviceClient<bender_srvs::setGoalBaseArm>("bottom_arm/planificarEfector");
	giro = n.serviceClient<bender_srvs::AngVel>("bottom_arm/girarGrip");
	agarre = n.serviceClient<bender_srvs::manipBaseArm>("manipBaseArm");

	ros::ServiceServer efect_ser = n.advertiseService("setGoalBaseArm",	setGoalService);
	ros::ServiceServer angle_ser = n.advertiseService("setAnglesBaseArm", setAngles);
	ros::ServiceServer orient_ser = n.advertiseService("orientBaseArm",	orientate);
	ros::ServiceServer manip_ser = n.advertiseService("manipBaseArm", CerrarGripBrazoService);
	ros::ServiceServer init_pose = n.advertiseService("bottom_arm/poseInicial",PoseInicialService);
	ros::ServiceServer act_pose = n.advertiseService("bottom_arm/actualizarPose",MoverBrazoService);
	ros::ServiceServer planificar = n.advertiseService("bottom_arm/planificarEfector",PlanificarEfectorService);
	ros::ServiceServer girargrip = n.advertiseService("bottom_arm/girarGrip", GirarGripService);


	getStateClient = n.serviceClient<bender_srvs::States>("bottom_arm_joints/states"); //comentar si solo se desea usar el simulador

	ros::Rate loop_rate(30);


	stateUpdate.request.select.resize(6);
	for (int i = 0; i < 6; i++) {
		stateUpdate.request.select[i] = selection[i];
	}

	cmdMsg.positions.resize(6);
	cmdMsg.select.resize(6);
	cmdMsg.speed.resize(6);

	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}

/******* Implementación de las Funciones *******/
void evaluateInverseKinematic(float O1, float O2, float O3) {
	//esta función cambia los valores de la matriz del jacobiano, para actualizarla a los estados O1, O2,O3

	jacobian.at<float>(0, 0) = 29.0 * sin(O1) * sin(O2) * sin(O3)
					- 16.0 * cos(O2) * sin(O1) - 29.0 * cos(O2) * cos(O3) * sin(O1)
					- 5.3 * sin(O1);
	jacobian.at<float>(0, 1) = -16.0 * cos(O1) * sin(O2)
					- 29.0 * cos(O1) * cos(O2) * sin(O3)
					- 29.0 * cos(O1) * cos(O3) * sin(O2);
	jacobian.at<float>(0, 2) = -29 * cos(O1) * cos(O2) * sin(O3)
					- 29 * cos(O1) * cos(O3) * sin(O2);
	jacobian.at<float>(1, 0) = 5.3 * cos(O1) + 16.0 * cos(O1) * cos(O2)
					+ 29.0 * cos(O1) * cos(O2) * cos(O3)
					- 29.0 * cos(O1) * sin(O2) * sin(O3);
	jacobian.at<float>(1, 1) = -16.0 * sin(O1) * sin(O2)
					- 29.0 * cos(O2) * sin(O1) * sin(O3)
					- 29.0 * cos(O3) * sin(O1) * sin(O2);
	jacobian.at<float>(1, 2) = -29.0 * cos(O2) * sin(O1) * sin(O3)
					- 29.0 * cos(O3) * sin(O1) * sin(O2);

	jacobian.at<float>(2, 0) = 0;
	jacobian.at<float>(2, 1) = 16.0 * cos(O2) + 29.0 * cos(O2) * cos(O3)
					- 29.0 * sin(O2) * sin(O3);
	jacobian.at<float>(2, 2) = 29.0 * cos(O2) * cos(O3)
					- 29.0 * sin(O2) * sin(O3);

	cv::Mat auxMatrix = jacobian.t() * jacobian;
	pInvJacobian = (auxMatrix.inv()) * jacobian.t();

}

/*** Servicios ***/
bool setGoalService(bender_srvs::setGoalBaseArm::Request &req,bender_srvs::setGoalBaseArm::Response &res) {

	res.received = setGoal(req.x,req.y,req.z);

	return true;
}

int setGoal(double g1, double g2, double g3){

	bender_srvs::Dummy service;

	double res = 0;
	float maxDesp = 0.5;
	int n = 6;
	float simulState[3][n];
	float simulEfect[3][n];
	int broken[n];

	Goal[0] = g1;
	Goal[1] = g2;
	Goal[2] = g3;

	goal[0] = g1;
	goal[1] = g2;
	goal[2] = g3;

	//**los n=6 puntos iniciales desde los cuales se intenta llegar al objetivo**//
	simulState[0][0] = armState[0];
	simulState[1][0] = armState[1];
	simulState[2][0] = armState[2];

	simulState[0][1] = 0.0;
	simulState[1][1] = 1.57;
	simulState[2][1] = -1.57;

	simulState[0][2] = 0.0;
	simulState[1][2] = -1.57;
	simulState[2][2] = 1.57;

	simulState[0][3] = 0.0;
	simulState[1][3] = 0.5;
	simulState[2][3] = 0.0;

	simulState[0][4] = 0.6;
	simulState[1][4] = 1.57;
	simulState[2][4] = 1.57;

	simulState[0][5] = -0.6;
	simulState[1][5] = 1.57;
	simulState[2][5] = 1.57;

	//** Movimiento simulado hasta la posicion pedida **//
	for (int j = 0; j < n; j++) {
		float simulDEfect[3] = { 0.0, 0.0, 0.0 };
		int count = 0;
		broken[j] = 0;
		simulEfect[0][j] = getX(simulState[0][j], simulState[1][j],
				simulState[2][j]);
		simulEfect[1][j] = getY(simulState[0][j], simulState[1][j],
				simulState[2][j]);
		simulEfect[2][j] = getZ(simulState[0][j], simulState[1][j],
				simulState[2][j]);
		while (((fabs(goal[0] - simulEfect[0][j])
				+ fabs(goal[1] - simulEfect[1][j])
				+ fabs(goal[2] - simulEfect[2][j])) / 3.0) >= 0.3) { //dentro de este while se hace el movimiento iterativo hasta la posición deseada

			for (int i = 0; i < 3; i++) { //calculo los deltas para cada angulo
				simulDEfect[i] = (goal[i] - simulEfect[i][j]);
			}

			getControlAction(simulDEfect[0], simulDEfect[1], simulDEfect[2],
					simulState[0][j], simulState[1][j], simulState[2][j]); //llama a la cinematica inversa, el resultado esta en simulDAngles

			//ROS_INFO("Condition Value %f",((fabs(goal[0]-simulEfect[0][j])+fabs(goal[1]-simulEfect[1][j])+fabs(goal[2]-simulEfect[2][j]))/3.0));
			//ROS_INFO("Deltas %f %f %f",simulDAngles.at<float>(0,0),simulDAngles.at<float>(1,0),simulDAngles.at<float>(2,0));
			//ROS_INFO("Deltas %f %f %f",simulDEfect[0],simulDEfect[1],simulDEfect[2]);
			//ROS_INFO("Estados %f %f %f",simulState[0],simulState[1],simulState[2]);

			float norma = sqrt(
					simulDAngles.at<float>(0, 0) * simulDAngles.at<float>(0, 0)
					+ simulDAngles.at<float>(1, 0)
					* simulDAngles.at<float>(1, 0)
					+ simulDAngles.at<float>(2, 0)
					* simulDAngles.at<float>(2, 0)); //calcula el largo del movimiento a realizar

			//simulo mover el brazo
			if (norma > maxDesp) { //si el movimiento es muy largo, se limita
				simulState[0][j] = simulState[0][j]
				                                 + maxDesp * simulDAngles.at<float>(0, 0) / norma;
				simulState[1][j] = simulState[1][j]
				                                 + maxDesp * simulDAngles.at<float>(1, 0) / norma;
				simulState[2][j] = simulState[2][j]
				                                 + maxDesp * simulDAngles.at<float>(2, 0) / norma;
			} else {
				simulState[0][j] = simulState[0][j]
				                                 + simulDAngles.at<float>(0, 0);
				simulState[1][j] = simulState[1][j]
				                                 + simulDAngles.at<float>(1, 0);
				simulState[2][j] = simulState[2][j]
				                                 + simulDAngles.at<float>(2, 0);
			}

			//actualizo la posición del efector
			simulEfect[0][j] = getX(simulState[0][j], simulState[1][j],
					simulState[2][j]);
			simulEfect[1][j] = getY(simulState[0][j], simulState[1][j],
					simulState[2][j]);
			simulEfect[2][j] = getZ(simulState[0][j], simulState[1][j],
					simulState[2][j]);

			count++;
			if (count > 1000) { //si no converge en 1000 pasos estoy abender_srvs::Dummy service;sumiendo que no puede llegar, ajustar dado el valor de maxDesp
				broken[j] = 1;
				break;
			}

		}

	}

	//evaluación de la mejor posición
	float lastfitness = 10000000000;
	int theOne = -1;
	float alpha = 0.2; //que tan importante es llegar más cerca de la posición con respecto a la minimización de movimiento
	for (int j = 0; j < n; j++) {
		if (broken[j] == 0) {
			if (simulState[0][j] <= angleMaxMin[0][0]
			                                       && simulState[0][j] >= angleMaxMin[1][0]
			                                                                             && simulState[1][j] <= angleMaxMin[0][1]
			                                                                                                                   && simulState[1][j] >= angleMaxMin[1][1]
			                                                                                                                                                         && simulState[2][j] <= angleMaxMin[0][2]
			                                                                                                                                                                                               && simulState[2][j] >= angleMaxMin[1][2]
			                                                                                                                                                                                                                                     && (simulEfect[2][j] > 0.0 || simulEfect[0][j] > 0.0)) { //se revisa que las soluciones cumplan las restricciones de angulos y sobre X y Z para no poder golpear el pioneer
				float fitness = (armState[0] - simulState[0][j])
								* (armState[0] - simulState[0][j])
								+ (armState[1] - simulState[1][j])
								* (armState[1] - simulState[1][j])
								+ (armState[2] - simulState[2][j])
								* (armState[2] - simulState[2][j])
								+ alpha
								* ((goal[0] - simulEfect[0][j])
										* (goal[0] - simulEfect[0][j])
										+ (goal[1] - simulEfect[1][j])
										* (goal[1] - simulEfect[1][j])
										+ (goal[2] - simulEfect[2][j])
										* (goal[2] - simulEfect[2][j]));
				if (fitness < lastfitness) {
					lastfitness = fitness;
					theOne = j;
				}
			} else {
				broken[j] = 1;
			}
		}
	}

	//ROS_INFO("broken:%d %d %d %d %d %d \n theOne:%d",broken[0],broken[1],broken[2],broken[3],broken[4],broken[5],theOne);

	//se revisa si existe una solución seleccionada
	int brokenCond = 0;
	for (int j = 0; j < n; j++) {
		brokenCond += broken[j];
	}
	if (brokenCond >= n || goal[1] < -20.0 || (goal[0] < 30 && goal[2] < 10)) {
		ROS_INFO("Imposible alcanzar la posición");
		goalAngles[0] = armState[0];
		goalAngles[1] = armState[1];
		goalAngles[2] = armState[2];
		res= 0;
	} else {
		if (theOne >= 0) {
			goalAngles[0] = simulState[0][theOne];
			goalAngles[1] = simulState[1][theOne];
			goalAngles[2] = simulState[2][theOne];
			res= 1;
		} else {
			res= -1;
		}
	}

	//Se ajustan los angulos de las articulaciones a [-pi,pi]
	while (goalAngles[0] >= 3.14) {
		goalAngles[0] = goalAngles[0] - 2 * 3.14;
	}
	ROS_INFO("1");
	while (goalAngles[1] >= 3.14) {
		goalAngles[1] = goalAngles[1] - 2 * 3.14;
	}
	ROS_INFO("2");
	while (goalAngles[2] >= 3.14) {
		goalAngles[2] = goalAngles[2] - 2 * 3.14;
	}
	ROS_INFO("3");
	while (goalAngles[0] <= -3.14) {
		goalAngles[0] = goalAngles[0] + 2 * 3.14;
	}
	ROS_INFO("4");
	while (goalAngles[1] <= -3.14) {
		goalAngles[1] = goalAngles[1] + 2 * 3.14;
	}
	ROS_INFO("5");
	while (goalAngles[2] <= -3.14) {
		goalAngles[2] = goalAngles[2] + 2 * 3.14;
	}

	//Retorna el servicio
	//ROS_INFO("Retorna");

//	cliente.call(service);

	return res;
}

bool setAngles(bender_srvs::setAnglesBaseArm::Request &req,	bender_srvs::setAnglesBaseArm::Response &res) {

	float wannabeGoalAng[3] = { 0.0, 0.0, 0.0 };

	wannabeGoalAng[0] = (req).alpha;
	wannabeGoalAng[1] = (req).beta;
	wannabeGoalAng[2] = (req).gamma;

	int condition = 0;
	for (int i = 0; i < 3; i++) { //se revisa que cumplan las restricciones
		if (wannabeGoalAng[i] < angleMaxMin[0][i]
		                                       && wannabeGoalAng[i] > angleMaxMin[1][i]) {
			condition++;
		}
	}

	//los angulos requeridos se transforman en meta si cumplen las restricciones de angulos
	if (condition == 3) {
		for (int i = 0; i < 3; i++) {
			goalAngles[i] = wannabeGoalAng[i];
		}
		res.received = 1;
	} else {
		res.received = 0;
	}
	return true;
}
bool orientate(bender_srvs::orientBaseArm::Request &req,
		bender_srvs::orientBaseArm::Response &res) {
	float orient = 0.0;
	if ((req).orientation) {
		orient = -1.57;
	} else {
		orient = -3.16;
	}
	if (armState[3] == orient) {
		(res).received = 0;
	} else {
		(res).received = 1;
		goalAngles[3] = orient;
	}

	return true;
}
bool grasp(bender_srvs::manipBaseArm::Request &req,
		bender_srvs::manipBaseArm::Response &res) {
	float fingerPos = 0.0;
	if ((req).action) {
		fingerPos = 1.0;

	} else {
		fingerPos = 0.09;
	}
	if (fabs(armState[4] + fingerPos) < 0.02
			|| fabs(armState[5] - fingerPos) < 0.02) {
		(res).received = 0;
	} else {
		(res).received = 1;
		goalAngles[4] = -fingerPos;
		goalAngles[5] = fingerPos;
	}

	return true;
}

/****** Cinemática Directa ******/
float getX(float O1, float O2, float O3) {

	return 5.3 * cos(O1) + 16.0 * cos(O1) * cos(O2)
			+ 29.0 * cos(O1) * cos(O2) * cos(O3)
			- 29.0 * cos(O1) * sin(O2) * sin(O3) + 25.0;
}

float getY(float O1, float O2, float O3) {

	return 5.3 * sin(O1) + 16.0 * cos(O2) * sin(O1)
			+ 29.0 * cos(O2) * cos(O3) * sin(O1)
			- 29.0 * sin(O1) * sin(O2) * sin(O3);
}

float getZ(float O1, float O2, float O3) {

	return 16.0 * sin(O2) + 29.0 * cos(O2) * sin(O3) + 29.0 * cos(O3) * sin(O2)
			- 2.5;
}

void getControlAction(float dx, float dy, float dz, float O1, float O2,
		float O3) {
	//esta función toma un dx, dy, dz, más los estados actuales y en el vector simulDAngles termina en cuanto deben modificarse O1, O2 y O3
	cv::Mat auxVector = cv::Mat::zeros(3, 1, CV_32F);
	cv::Mat result = cv::Mat::zeros(3, 1, CV_32F);
	auxVector.at<float>(0, 0) = dx;
	auxVector.at<float>(1, 0) = dy;
	auxVector.at<float>(2, 0) = dz;

	evaluateInverseKinematic(O1, O2, O3);

	result = pInvJacobian * auxVector;

	simulDAngles.at<float>(0, 0) = result.at<float>(0, 0);
	simulDAngles.at<float>(1, 0) = result.at<float>(1, 0);
	simulDAngles.at<float>(2, 0) = result.at<float>(2, 0);
}

void MoverGripAng(double angulo, double velocidad){

	//FUNCION DE ROS PARA MOTORES IZQ Y DER DE LA MANO
	//ros::Publisher command_pub = nh.advertise<bender_planning_old::Command>("left_arm_joints/command", 1000);

	bender_msgs::Command com;

	com.positions.resize(6);
	com.select.resize(6);
	com.speed.resize(6);

	for(int i=0;i<6;i++)
	{
		com.positions[i] = 0;
		com.speed[i] = 0;
		com.select[i]=false;
		if(i==4)
		{
			com.positions[i] = angulo;
			com.speed[i] = velocidad;
			com.select[i]=true;
		}
		if (i==5)
		{
			com.positions[i] = -1*angulo;
			com.speed[i] = velocidad;
			com.select[i]=true;
		}
	}

	command_pub.publish(com);
	usleep(100000);


	ros::spinOnce();
}

bool CerrarGripBrazoService(bender_srvs::manipBaseArm::Request &req,bender_srvs::manipBaseArm::Response &res){
	CerrarGripBrazo(req.action);
	return true;
}

void CerrarGripBrazo(int loadMode)
{
	double anguloR = 0, anguloL = 0;
	double ANG = 0;
	double load = 0.0; //variables para almacenar lectura de torque
	double loadR = 0.0;
	double loadL = 0.0;
	double loadMax;
	double k0 = 0.3; //Constante de offset para función exponencial 0.1
	double k1 = 12; //Constante de ganancia para función exponencial 12
	double k2 = 3000; //Modificar enter 1000(pasos grandes) y 5000(pasos chicos) Constante para función exponencial
	double ang = 90; //angulo de apertura inicial
	double velocidad = 0.3; //Velocidad en rad/seg (de 0 a 6.67)

//	if (loadMode==0) loadMax=0.4; //Si modo liviano para apretar
//	else loadMax=0.7; //Si modo fuerte

	bender_srvs::States srv;
	srv.request.select.resize(6);
	for(int i=0; i<4;i++)
		srv.request.select[i]=false;
	for(int i=4; i<6;i++)
		srv.request.select[i]=true;

	loadMax = 0.4;

	if (loadMode == 1){ // Abrir el grip
		ANG = -ang*M_PI/180.0;
		MoverGripAng(ANG,0.3);
	}
	else{
		while (fabs(loadR)<loadMax && fabs(loadL)<loadMax){
		 //Función sigmoidal para suavizar el cerrado del gripper cuando esté cerca al objeto
			ang = ang - (1-exp(-1/k2*ang*ang))*k1 + k0;
//			printf("ANGULO_SIGMOIDE: %f\n",ang);
			if (ang<=10)  break;
			MoverGripAng(-ang*M_PI/180.0, velocidad);
			usleep(50000);
			while(!ros::service::waitForService("bottom_arm_joints/states", ros::Duration(1.0)))
			{
				ROS_INFO_STREAM("Waiting for planner service bottom_arm_joints/states");
			}
			if (stateClient.call(srv))
			{
//				ROS_INFO("leyendo motores...");
				loadR = srv.response.load[4];
				loadL = srv.response.load[5];
				anguloR = srv.response.state[4];
				anguloL = srv.response.state[5];
			}
			else
			{
				ROS_ERROR("Failed to call service bottom_arm_joints/states in CerrarGripBrazo");
			}
			load = 0.5*(fabs(loadR)+fabs(loadL));
			ROS_INFO("LOADR: %f",loadR);
			ROS_INFO("LOADL: %f",loadL);
			ROS_INFO("ang_R: %f", anguloR*180/M_PI);
			ROS_INFO("ang_L: %f", anguloL*180/M_PI);
//			if(fabs(loadR)>loadMax) ROS_INFO("R SUPERADO!");
//			if(fabs(loadL)>loadMax) ROS_INFO("L SUPERADO!");
		}
		if (ang<=10) {
			printf("Objeto no tomado... Angulo final=%f\n",ang);
			}
		else {printf("Objeto tomado. Angulo final=%f - Carga=%f\n",ang,load);
			}
	}

}

bool PoseInicialService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	PoseInicial();
	ROS_INFO("Listo para grippear!");
	return true;
}

void PoseInicial(){

	cmdMsg.positions.resize(6);
	cmdMsg.select.resize(6);
	cmdMsg.speed.resize(6);

	if (laps == 0){
		for (int i = 0 ; i < 6 ; i++){
			cmdMsg.positions[i] = initpose[i];
			cmdMsg.select[i] = true;
			cmdMsg.speed[i] = 0.3;
		}
		laps++;
	} else {
		for (int i = 0 ; i < 4 ; i++){
			cmdMsg.positions[i] = initpose[i];
			cmdMsg.select[i] = true;
			cmdMsg.speed[i] = 0.3;
		}
		for (int i = 4 ; i < 6 ; i++){
			cmdMsg.select[i] = false;
		}
	}


	command_pub.publish(cmdMsg);

}

bool MoverBrazoService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	ROS_INFO("Moviendo el brazo de la base");
	MoverBrazo();
	return true;
}

void MoverBrazo(){
	/*Actualización de Estados*/

//		for (int j = 0; j < 6; j++) {
//			armState[j] = simArmState[j]; //descomentar si se desea solo simular
//			dAngles[j] = goalAngles[j] - armState[j];
//		}

	/**/
	ROS_INFO("#1 var: %f \n",var);
	ROS_INFO("   goal: %f | %f | %f \n",Goal[0],Goal[1],Goal[2]);
	ROS_INFO("efector: %f | %f | %f \n",efector[0],efector[1],efector[2]);
	ros::Rate while_rate(30);
	while (var >= 1.9){
		var = ((fabs(Goal[0] - efector[0]) + fabs(Goal[1] - efector[1]) + fabs(Goal[2] - efector[2])) / 3.0);
		if (getStateClient.call(stateUpdate)) { //comentar el if y el else, si se desea solo simular
			for (int i = 0; i < 6; i++) {
				armState[i] = stateUpdate.response.state[i];
				simArmState[i] = stateUpdate.response.state[i];
				dAngles[i] = goalAngles[i] - armState[i];
			}
		} else {
			ROS_ERROR(
					"Service bottom_arm_joints/states failed while being requested by baseArm_Controller");
		}
		/**/
		efector[0] = getX(armState[0], armState[1], armState[2]);
		efector[1] = getY(armState[0], armState[1], armState[2]);
		efector[2] = getZ(armState[0], armState[1], armState[2]);

		//    if (laps%10==0){//mensajes de debugeo
		//ROS_INFO(" X:%f Y:%f Z:%f \n O1:%f O2:%f O3:%f",efector[0],efector[1],efector[2],armState[0],armState[1],armState[2]);
		//ROS_INFO("l%f r%f ",stateUpdate.response.load[4],stateUpdate.response.load[5]);
		//	if(laps>=100){laps=laps-100;}
		//    }
		/** Condición para saber si el efector está en posición**/
		if (sqrt(pow((Goal[0] - efector[0]),2) + pow((Goal[1] - efector[1]),2)
				+ pow((Goal[2] - efector[2]),2)) >= 1) {
			float norma = sqrt(
					dAngles[0] * dAngles[0] + dAngles[1] * dAngles[1]
																   + dAngles[2] * dAngles[2]); //medida de distancia del desplazamiento a realizar

			if (norma > maxDesp) { //si el desplazamiento es muy largo se limita
				jointStates.Joint1 = armState[0] + maxDesp * dAngles[0] / norma;
				jointStates.Joint2 = armState[1] + maxDesp * dAngles[1] / norma;
				jointStates.Joint3 = armState[2] + maxDesp * dAngles[2] / norma;
				simArmState[0] = armState[0] + maxDesp * dAngles[0] / norma;
				simArmState[1] = armState[1] + maxDesp * dAngles[1] / norma;
				simArmState[2] = armState[2] + maxDesp * dAngles[2] / norma;
				cmdMsg.positions[0] = armState[0]
											   + maxDesp * dAngles[0] / norma;
				cmdMsg.positions[1] = armState[1]
											   + maxDesp * dAngles[1] / norma;
				cmdMsg.positions[2] = armState[2]
											   + maxDesp * dAngles[2] / norma;
			} else { // se desplazan las articulaciones
				jointStates.Joint1 = armState[0] + dAngles[0];
				jointStates.Joint2 = armState[1] + dAngles[1];
				jointStates.Joint3 = armState[2] + dAngles[2];
				simArmState[0] = armState[0] + dAngles[0];
				simArmState[1] = armState[1] + dAngles[1];
				simArmState[2] = armState[2] + dAngles[2];
				cmdMsg.positions[0] = armState[0] + dAngles[0];
				cmdMsg.positions[1] = armState[1] + dAngles[1];
				cmdMsg.positions[2] = armState[2] + dAngles[2];
			}

		}
		//movimiento de la articulación de la muñeca
		jointStates.Joint4 = armState[3] + dAngles[3];
		simArmState[3] = armState[3] + dAngles[3];
		cmdMsg.positions[3] = armState[3];// + dAngles[3];

		//**Publicacion de Mensajes**//
		efectorMsg.x = efector[0];
		efectorMsg.y = efector[1];
		efectorMsg.z = efector[2];

		for (int i = 0; i < 6; i++) {
			cmdMsg.select[i] = selection[i];
			cmdMsg.speed[i] = speeds[i];
		}

		bottomMotors_pub.publish(cmdMsg);
	//		grip_pub.publish(gripMsg);
	//		baseArmState_pub.publish(jointStates);
		baseArmEfect_pub.publish(efectorMsg);

		//*Fin del Ciclo*//
		ROS_INFO("Actualizando Posicion");
		ROS_INFO("middle var: %f \n",var);
		while_rate.sleep();
	}
	ROS_INFO("#2 var: %f \n",var);
	var = 2;
	efector[0] = 0;
	efector[1] = 0;
	efector[2] = 0;
}

void GirarGrip(double angulo)
{
	bender_msgs::Command com;
	com.select.resize(6);
	com.positions.resize(6);
	com.speed.resize(6);
	com.select[3] = true;
	com.speed[3]= 0.5;
	com.positions[3] = angulo*M_PI/180;

	command_pub.publish(com);

}

bool GirarGripService(bender_srvs::AngVel::Request &req, bender_srvs::AngVel::Response &res){
	GirarGrip(req.angle);
	ROS_INFO("Girando el grip %f grados", req.angle);
	return true;
}

bool PlanificarEfectorService(bender_srvs::setGoalBaseArm::Request &req, bender_srvs::setGoalBaseArm::Response &res){
	bender_srvs::setGoalBaseArm srv;
	srv.request.x = req.x;
	srv.request.y = req.y;
	srv.request.z = req.z;
//	ROS_INFO("1");
	if (setGoal(req.x,req.y,req.z) == 1){
//		ROS_INFO("2");
		MoverBrazo();
	}
	else
		ROS_INFO("Posicion invalida");

	return true;
}


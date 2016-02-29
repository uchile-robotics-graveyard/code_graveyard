#include "ros/ros.h"
#include "bender_msgs/baseArmState.h"
#include "bender_msgs/baseEfectorPosition.h"
#include "bender_srvs/setAnglesBaseArm.h"
#include "bender_srvs/setGoalBaseArm.h"
#include "bender_srvs/manipBaseArm.h"
#include "bender_srvs/orientBaseArm.h"
#include "bender_msgs/grip.h"
#include <opencv/cv.hpp>
#include <math.h>
#include "bender_msgs/Command.h"
#include "bender_srvs/States.h"

//copiar el siguiente código en el .launch para que ejecute este nodo también
//<node name="baseArm_Controller"    pkg="bottomArmController" type="baseArm_Controller" />

/****** Variables Globales ******/
float armState[6] = { 0.0, -1.57, 1.6, -3.16, -0.09, 0.09 };
float simArmState[6] = { 0.0, -1.57, 1.6, -3.16, -0.09, 0.09 };
float efector[3] = { 0.0, 0.0, 0.0 };
float goalAngles[6] = { 0.0, -1.57, 1.6, -3.16, -0.09, 0.09 };
float goal[3] = { 0.0, 0.0, 0.0 };
float angleMaxMin[2][6] = { { 0.63, 1.68, 2.29, 1.61, -0.08, 1.1 }, { -0.82,
		-1.62, -2.23, -3.35, -1.1, 0.08 } };
cv::Mat jacobian = cv::Mat::eye(3, 3, CV_32F);
cv::Mat pInvJacobian = cv::Mat::eye(3, 3, CV_32F);
cv::Mat simulDAngles = cv::Mat::zeros(3, 1, CV_32F);

/************ Declaración de Funciones ************/

/*** servicios ***/
bool setGoal(bender_srvs::setGoalBaseArm::Request &req,
		bender_srvs::setGoalBaseArm::Response &res);
bool setAngles(bender_srvs::setAnglesBaseArm::Request &req,
		bender_srvs::setAnglesBaseArm::Response &res);
bool orientate(bender_srvs::orientBaseArm::Request &req,
		bender_srvs::orientBaseArm::Response &res);
bool grasp(bender_srvs::manipBaseArm::Request &req,
		bender_srvs::manipBaseArm::Response &res);

/*** cinemática directa ***/
float getX(float O1, float O2, float O3);
float getY(float O1, float O2, float O3);
float getZ(float O1, float O2, float O3);

/*** cinemática Inversa ***/
void evaluateInverseKinematic(float O1, float O2, float O3);
//void linearMovement(float goal[3]);
void getControlAction(float dx, float dy, float dz, float O1, float O2,
		float O3);

/********** MAIN *********/
int main(int argc, char **argv) {

	/**** Inicializacion ****/
	ros::init(argc, argv, "baseArm_ControllerNode");
	ros::NodeHandle n;
	ros::Publisher baseArmState_pub = n.advertise<bender_msgs::baseArmState>(
			"PosicionActuadoresBaseArm", 2);
	ros::Publisher baseArmEfect_pub = n.advertise<
			bender_msgs::baseEfectorPosition>("PosicionEfectorBaseArm", 2);
	ros::Publisher bottomMotors_pub = n.advertise<bender_msgs::Command>(
			"bottom_arm_joints/command", 2);
	ros::Publisher grip_pub = n.advertise<bender_msgs::grip>("gripState", 1);

	ros::ServiceServer efect_ser = n.advertiseService("setGoalBaseArm",
			setGoal);
	ros::ServiceServer angle_ser = n.advertiseService("setAnglesBaseArm",
			setAngles);
	ros::ServiceServer orient_ser = n.advertiseService("orientBaseArm",
			orientate);
	ros::ServiceServer manip_ser = n.advertiseService("manipBaseArm", grasp);

	ros::ServiceClient getStateClient = n.serviceClient<bender_srvs::States>(
			"bottom_arm_joints/states"); //comentar si solo se desea usar el simulador

	ros::Rate loop_rate(30);

	bender_msgs::baseArmState jointStates;
	bender_msgs::baseEfectorPosition efectorMsg;
	bender_msgs::Command cmdMsg;
	bender_msgs::grip gripMsg;

	bender_srvs::States stateUpdate;

	int laps = 0;
	float maxDesp = 0.2;
	float dAngles[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	bool selection[6] = { true, true, true, true, true, true };
	float speeds[6] = { 0.3, 0.3, 0.3, 0.3, 0.3, 0.3 };

	stateUpdate.request.select.resize(6);
	for (int i = 0; i < 6; i++) {
		stateUpdate.request.select[i] = selection[i];
	}

	cmdMsg.positions.resize(6);
	cmdMsg.select.resize(6);
	cmdMsg.speed.resize(6);

	float gripCount = 0.0;
	/***** Ciclo Principal *****/
	while (ros::ok()) {

		/**Actualización de Estados**/

		for (int j = 0; j < 6; j++) {
			armState[j] = simArmState[j]; //descomentar si se desea solo simular
			dAngles[j] = goalAngles[j] - armState[j];
		}

		/**/
		if (getStateClient.call(stateUpdate)) { //comentar todo el if y el else, si se desea solo simular
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
		if (((fabs(goal[0] - efector[0]) + fabs(goal[1] - efector[1])
				+ fabs(goal[2] - efector[2])) / 3.0) >= .5) {
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
		cmdMsg.positions[3] = armState[3] + dAngles[3];

		//movimiento de la pinza
		//como aquí se hace lectura de la corriente de los motores, todo este IF se debe comentar para solo simular, junto con el ELSE, pero no su contenido
		/**/
		if ((fabs(stateUpdate.response.load[4]) > 0.1
				|| fabs(stateUpdate.response.load[5]) > 0.1)
				&& (fabs(goalAngles[4]) <= .1 || fabs(goalAngles[5]) <= .1)
				&& gripCount <= 0.0) { //condición de agarre
			float agarre = 0.01;
			float position = (armState[5] - armState[4]) / 2.0;
			if (fabs(stateUpdate.response.load[4]) > 0.6
					|| fabs(stateUpdate.response.load[5]) > 0.6) { //condición agarre fuerte
				jointStates.Joint5 = -position - agarre;
				cmdMsg.positions[4] = -position - agarre;
				jointStates.Joint6 = position + agarre;
				cmdMsg.positions[5] = position + agarre;
				//goalAngles[4]=armState[4];
				//goalAngles[5]=armState[5];
				gripCount = gripCount + 5.0;
				ROS_INFO("if 2");
			} else if (fabs(stateUpdate.response.load[4]) > 0.25
					|| fabs(stateUpdate.response.load[5]) > 0.25) { //condición agarre medio
				jointStates.Joint5 = -position;
				//	   cmdMsg.positions[4]=-position;
				jointStates.Joint6 = position;
				// 	   cmdMsg.positions[5]=position;
				//goalAngles[4]=armState[4];
				//goalAngles[5]=armState[5];
				ROS_INFO("if 1");
			} else { //agarre muy suave
				jointStates.Joint5 = -position + agarre;
				cmdMsg.positions[4] = -position + agarre;
				jointStates.Joint6 = position - agarre;
				cmdMsg.positions[5] = position - agarre;
				//goalAngles[4]=armState[4];
				//goalAngles[5]=armState[5];
				ROS_INFO("else");
				gripCount = gripCount + 20.0;
			}

			gripMsg.somethingGripped = true;
		}/**/
		else { //no hay nada en la pinza
			jointStates.Joint5 = armState[4] + dAngles[4];
			cmdMsg.positions[4] = armState[4] + dAngles[4];
			jointStates.Joint6 = armState[5] + dAngles[5];
			cmdMsg.positions[5] = armState[5] + dAngles[5];
			simArmState[4] = armState[4] + dAngles[4];
			simArmState[5] = armState[5] + dAngles[5];
			if (fabs(stateUpdate.response.load[4])
					+ fabs(stateUpdate.response.load[5]) > 0.2) { //comentar para solo simular
				gripMsg.somethingGripped = true;
			} else if (gripCount == 0.0) { // comentar para solo simular
				gripMsg.somethingGripped = false;
			}
			gripCount = gripCount - 1.0; // comentar para solo simular
		}

		//**Publicacion de Mensajes**//
		efectorMsg.x = efector[0];
		efectorMsg.y = efector[1];
		efectorMsg.z = efector[2];

		for (int i = 0; i < 6; i++) {
			cmdMsg.select[i] = selection[i];
			cmdMsg.speed[i] = speeds[i];
		}

		bottomMotors_pub.publish(cmdMsg);
		baseArmState_pub.publish(jointStates);
		baseArmEfect_pub.publish(efectorMsg);
		grip_pub.publish(gripMsg);

		//*Fin del Ciclo*//
		ros::spinOnce();
		loop_rate.sleep();
		laps++;

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
bool setGoal(bender_srvs::setGoalBaseArm::Request &req,
		bender_srvs::setGoalBaseArm::Response &res) {

	float maxDesp = 0.5;
	int n = 6;
	float simulState[3][n];
	float simulEfect[3][n];
	int broken[n];

	goal[0] = (req).x;
	goal[1] = (req).y;
	goal[2] = (req).z;

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
			if (count > 1000) { //si no converge en 1000 pasos estoy asumiendo que no puede llegar, ajustar dado el valor de maxDesp
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
	if (brokenCond >= n || goal[2] < -20.0 || (goal[0] < 30 && goal[3] < 10)) {
		ROS_INFO("Imposible alcanzar la posición");
		goalAngles[0] = armState[0];
		goalAngles[1] = armState[1];
		goalAngles[2] = armState[2];
		res.received = 0;
	} else {
		if (theOne >= 0) {
			goalAngles[0] = simulState[0][theOne];
			goalAngles[1] = simulState[1][theOne];
			goalAngles[2] = simulState[2][theOne];
			res.received = 1;
		} else {
			res.received = -1;
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
	return true;
}

bool setAngles(bender_srvs::setAnglesBaseArm::Request &req,
		bender_srvs::setAnglesBaseArm::Response &res) {

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


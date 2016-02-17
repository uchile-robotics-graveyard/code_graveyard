#ifndef _PLANIFICADOR_H_

#define _PLANIFICADOR_H_

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <iostream> 
#include <vector>
#include <setjmp.h>
#include <signal.h>

#include "bender_planning_old/OMNode.h"
#include "bender_planning_old/IKNode.h"
#include "bender_planning_old/spline.h"
#include "bender_planning_old/punto.h"

#include "bender_msgs/Command.h"

#include "bender_srvs/PlanningGoalState.h"
#include "bender_srvs/PlanningGoalCartesian.h"
#include "bender_srvs/Margin.h"
#include "bender_srvs/Punch.h"
#include "bender_srvs/Slope.h"
#include "bender_srvs/States.h"
#include "bender_srvs/TorqueEnable.h"
#include "bender_srvs/AngVel.h"
#include "bender_srvs/Dummy.h"
#include "bender_srvs/Feedback.h"
#include "bender_srvs/LoadMode.h"

#include <ros/ros.h>

#define BUFSIZE 50
#define PI 3.14159265
#define TOL 0.0005

#define OKvar (1)
#define ERRORvar (0)

#define MAX_TIEMPO_CINEMATICA_INVERSA (2)

using namespace std;

int generarCoef(double qi[],double qo[],double tiempo,double coef[][4]);
double Angulo(int Joint, double tiempo, double coef[][4]);
double Velocidad(int Joint, double tiempo, double coef[][4]);

void matrizRotacion(double matrizRot[][3], double angulos[]);


class Uarm{
private:
	double lastx;
	double lasty;
	double lastz;
	double lastxg;
	double lastyg;
	double lastzg;
	ros::Publisher command_pub;
	ros::Subscriber sub;
	
public:	
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner;
	ros::ServiceServer cartesian_service;
	ros::ServiceServer grasp_service;
	ros::ServiceServer state_service;
	ros::ServiceServer gripang_service;
	ros::ServiceServer munecaang_service;
	ros::ServiceServer abrirgrip_service;
	ros::ServiceServer cerrargrip_service;
	ros::ServiceServer orientargrip_service;
	ros::ServiceServer posicion_reposo_service;
	ros::ServiceServer salir_posicion_reposo_service;
	ros::ServiceServer posicion_inicial_service;
	ros::ServiceServer posicion_pre1_service;
	ros::ServiceServer posicion_pre2_service;
	ros::ServiceServer posicion_post1_service;
	ros::ServiceServer entregar_papel_service;
	ros::ServiceServer torque_enable_service;
	ros::ServiceServer feedback_service;
	ros::ServiceServer grasp_feedback_service;
	bool apretar;


	Uarm();
	void planificadorDirecto(double qi[],double qo[]);
	bool MoverBrazoB(double x, double y, double z, int Brazo, bool PlanGrip);
	void MoverBrazoAng(double q0, double q1, double q2,double q3,int Brazo);
	void AccionarBrazo(double *tr,int Brazo);
	void AccionarBrazoSpline(double *tr,int Brazo);

	void PosInicial(int Brazo);
	void torqueOnOff(int OnOff);
	void torqueMotorOnOff(string name, int OnOff); //30MAY2012
	void AbrirCerrarGrip(int Abrir);
	int leerPoseServo(double qi[]);
	bool leerMoving(string name); // DLF 8JUN2011
	bool BrazoMov(int Brazo); // 24JUN2011
	void MoverGripAng(double angulo, double velocidad); // 24JUN2011
	void AbrirGripBrazo();// 24JUN2011
	int CerrarGripBrazo(int loadMode);// 24JUN2011
	void PosicionReposo();//30MAY2012
	void SalirPosicionReposo();//30MAY2012
	void MoverMuneca(double angulo, double velocidad);//30MAY2012
	void MoverMotor(double angulo, double velocidad, int motor);

	/*void SetComplianceSlope(string name,int joint); //23MAY2012
	void SetAllComplianceSlope(int slope); //23MAY2012
	int SelectSlopeLevel(double total, double actual); //23MAY2012
	*/
	void SetPunch(string name,int value); //23MAY2012
	void SetAllPunch(int value); //23MAY2012

	void SetPIDGain(int P, int ID,string name);
	int SelectIDLevel(double total, double actual);

	void OrientarGrip(void);
	void CinematicaDirecta(double state[], double pos[]);
	void CinematicaDirectaGrasp(double state[], double pos[]);

	bool GoalCartesianService(bender_srvs::PlanningGoalCartesian::Request &req,bender_srvs::PlanningGoalCartesian::Response &res);
	bool GraspService(bender_srvs::PlanningGoalCartesian::Request &req,bender_srvs::PlanningGoalCartesian::Response &res);
	bool GoalStateService(bender_srvs::PlanningGoalState::Request &req,bender_srvs::PlanningGoalState::Response &res);
	bool MoverGripAngService(bender_srvs::AngVel::Request &req,bender_srvs::AngVel::Response &res);
	bool MoverMunecaService(bender_srvs::AngVel::Request &req,bender_srvs::AngVel::Response &res);
	bool OrientarGripService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
	bool AbrirGripBrazoService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
	bool CerrarGripBrazoService(bender_srvs::LoadMode::Request &req,bender_srvs::LoadMode::Response &res);
	bool PosicionReposoService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
	bool SalirPosicionReposoService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
	bool PosicionInicialService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
	bool PosicionPre1Service(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
	bool PosicionPre2Service(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
	bool PosicionPost1Service(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
	bool TorqueEnableService(bender_srvs::TorqueEnable::Request &req, bender_srvs::TorqueEnable::Response &res);

	bool EntregarPapelService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);

	bool FeedbackService(bender_srvs::Feedback::Request &req, bender_srvs::Feedback::Response &res);
	bool GraspFeedbackService(bender_srvs::Feedback::Request &req,bender_srvs::Feedback::Response &res);

	~Uarm();
};

#endif

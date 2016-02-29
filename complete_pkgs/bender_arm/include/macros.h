/*
 * macros.h
 *
 *  Created on: Jan 15, 2014
 *      Author: bendervision
 */

#ifndef MACROS_H_
#define MACROS_H_

#include "ros/ros.h"
#include "bender_msgs/grip.h"
#include "bender_msgs/baseArmState.h"
#include "bender_msgs/baseEfectorPosition.h"
#include "bender_srvs/PlanningGoalCartesian.h"
#include "bender_srvs/setAnglesBaseArm.h"
#include "bender_srvs/setGoalBaseArm.h"
#include "bender_srvs/manipBaseArm.h"
#include "bender_srvs/orientBaseArm.h"
#include "bender_srvs/LoadMode.h"
#include <opencv/cv.hpp>
#include <math.h>
#include "bender_msgs/Command.h"
#include "bender_srvs/Dummy.h"
#include "bender_srvs/States.h"
#include "bender_srvs/AngVel.h"
#include "bender_srvs/planeXY.h"
#include "bender_srvs/ArmState.h"
#include "bender_srvs/TorqueLimit.h"
#include "bender_srvs/Onoff.h"

bool EntregarBrazoDerechoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res);
bool EntregarBrazoIzquierdoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res);
bool AgarrarObjetoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res);
bool SoltarObjetoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res);
bool SoltarSuaveService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res);
bool RecibirObjetoManoDerechaService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
bool RecibirObjetoManoIzquierdaService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res);
bool PosicionReposoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res);
bool SalirPosicionReposoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res);

void SoltarSuave(double soltar);
void MoverMotor(double angulo, double velocidad,int brazo, int motor);
void MoverMuneca(double angulo, double velocidad);
void MoverGripBaseAng(double angulo, double velocidad);
void RecibirObjeto(int brazo);
void AgarrarObjeto();
void SoltarObjeto();
void EntregarBrazoDerecho();
void EntregarBrazoIzquierdo();
void PosicionReposo();
void SalirPosicionReposo();
void LeerCarga(double carga[],int brazo);
bool agarroLaLata(int brazo);

ros::ServiceClient inicialbase,torqueoff;
ros::ServiceClient agarre;
ros::ServiceClient plan;
ros::ServiceClient giro;
ros::ServiceClient abrir,cerrar,grasp,orientar,inicial,post1,pre1,pre2;

ros::Publisher command__pub;
ros::Publisher command__pub__base;
ros::Publisher  vel_pub;
ros::NodeHandle * n;


#endif /* MACROS_H_ */

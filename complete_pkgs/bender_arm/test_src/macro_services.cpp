/*
 * macro_services.cpp
 *
 *  Created on: Jan 7, 2014
 *      Author: bendervision
 */

#include "macros.h"

int main(int argc, char **argv) {

	/**** Inicializacion ****/
	ros::init(argc, argv, "manipArmServices");
//	ros::AsyncSpinner spinner(4);
	ros::NodeHandle nh;
	n = &nh;

	plan = n->serviceClient<bender_srvs::setGoalBaseArm>("bottom_arm/planificarEfector");
	giro = n->serviceClient<bender_srvs::AngVel>("bottom_arm/girarGrip");
	agarre = n->serviceClient<bender_srvs::manipBaseArm>("manipBaseArm");
	inicialbase = n->serviceClient<bender_srvs::Dummy>("bottom_arm/poseInicial");
	torqueoff = n->serviceClient<bender_srvs::Onoff>("bottom_arm_joints/onoff");

	/* lanzar servicios de los tres brazos */
	ros::ServiceServer entregarI = n->advertiseService("bottom_arm/entregarIzq",EntregarBrazoIzquierdoService);
	ros::ServiceServer entregarD = n->advertiseService("bottom_arm/entregarDer",EntregarBrazoDerechoService);
	ros::ServiceServer agarra = n->advertiseService("bottom_arm/agarrarObjeto", AgarrarObjetoService);
	ros::ServiceServer suelta = n->advertiseService("bottom_arm/soltarObjeto", SoltarObjetoService);

	ros::ServiceServer receiverright = n->advertiseService("right_arm/recibir_objeto_der", RecibirObjetoManoDerechaService);
	ros::ServiceServer receiverleft = n->advertiseService("left_arm/recibir_objeto_izq", RecibirObjetoManoIzquierdaService);

	ros::ServiceServer posreposo = n->advertiseService("bottom_arm/posicion_reposo", PosicionReposoService);
	ros::ServiceServer sposreposo = n->advertiseService("bottom_arm/salir_posicion_reposo", SalirPosicionReposoService);

	command__pub = n->advertise<bender_msgs::Command>("right_arm_joints/command", 1000);
	command__pub__base = n->advertise<bender_msgs::Command>("bottom_arm_joints/command", 1000);

	ros::Rate loop_rate(30);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

//	spinner.start();
//	ros::waitForShutdown();
	return 0;
}

bool PosicionReposoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res){
	PosicionReposo();
	return true;
}

bool SalirPosicionReposoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res){
	SalirPosicionReposo();
	return true;
}

bool EntregarBrazoIzquierdoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res){
	EntregarBrazoIzquierdo();
	return true;
}

bool EntregarBrazoDerechoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res){
	EntregarBrazoDerecho();
	return true;
}

bool AgarrarObjetoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res){
	AgarrarObjeto();
	return true;
}

bool SoltarObjetoService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res){
	SoltarObjeto();
	return true;
}

bool SoltarSuaveService(bender_srvs::Dummy::Request &req, bender_srvs::Dummy::Response &res){
//	SoltarSuave(0.1);
	MoverGripBaseAng(M_PI/2,0.3);
	return true;
}

bool RecibirObjetoManoDerechaService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	/* para llamar servicios del brazo derecho */
	while(!ros::service::waitForService("right_arm/abrir_grip", ros::Duration(0.0)))
	{
		ROS_INFO_STREAM("Waiting for right_arm services to be ready");
	}
	RecibirObjeto(2);

	return true;
}

bool RecibirObjetoManoIzquierdaService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	/* para llamar servicios del brazo izquierdo */
	while(!ros::service::waitForService("left_arm/abrir_grip", ros::Duration(0.0)))
	{
		ROS_INFO_STREAM("Waiting for left_arm services to be ready");
	}
	RecibirObjeto(0);

	return true;
}

void MoverGripBaseAng(double angulo, double velocidad){

	//FUNCION DE ROS PARA MOTORES IZQ Y DER DE LA MANO
	//ros::Publisher command__pub = nh.advertise<bender_msgs::Command>("right_arm_joints/command", 1000);

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
			com.positions[i] = -1*angulo;
			com.speed[i] = velocidad;
			com.select[i]=true;
		}
		if (i==5)
		{
			com.positions[i] = angulo;
			com.speed[i] = velocidad;
			com.select[i]=true;
		}
	}

	command__pub__base.publish(com);

}

void MoverMuneca(double angulo, double velocidad){

	bender_msgs::Command com;

	com.positions.resize(8);
	com.select.resize(8);
	com.speed.resize(8);

	for(int i=0;i<8;i++)
	{
		if(i==5)
		{
			com.positions[i] = angulo;
			com.speed[i] = velocidad;
			com.select[i]=true;
		}
		else
		{
			com.positions[i] = 0;
			com.speed[i] = 0;
			com.select[i]=false;
		}
	}

	command__pub.publish(com);

}
void MoverMotor(double angulo, double velocidad, int brazo, int motor){

	//FUNCION DE ROS PARA MOVER servo dado por el indice motor.
	bender_msgs::Command com;
	int motores = 0;

	if (brazo == 1){
		command__pub = n->advertise<bender_msgs::Command>("left_arm_joints/command", 1000);
		motores = 8;
	}
	else if (brazo == 2){
		command__pub = n->advertise<bender_msgs::Command>("right_arm_joints/command", 1000);
		motores = 8;
	}
	else if (brazo == 3){
		command__pub = n->advertise<bender_msgs::Command>("bottom_arm_joints/command", 1000);
		motores = 6;
	}

	com.positions.resize(motores);
	com.select.resize(motores);
	com.speed.resize(motores);

	for(int i=0;i<motores;i++)
	{
		if(i==motor)
		{
			com.positions[i] = angulo;
			com.speed[i] = velocidad;
			com.select[i]=true;
		}
		else
		{
			com.positions[i] = 0;
			com.speed[i] = 0;
			com.select[i]=false;
		}
	}

	command__pub.publish(com);
}

void RecibirObjeto(int brazo){

	int contador = 0, intentos = 5;

	inicialbase = n->serviceClient<bender_srvs::Dummy>("bottom_arm/poseInicial");

	if(brazo == 1){// brazo izquierdo
		abrir = n->serviceClient<bender_srvs::Dummy>("left_arm/abrir_grip");
		cerrar = n->serviceClient<bender_srvs::LoadMode>("left_arm/cerrar_grip");
		grasp = n->serviceClient<bender_srvs::PlanningGoalCartesian>("left_arm/grasp");
		orientar = n->serviceClient<bender_srvs::Dummy>("left_arm/orientar_grip");
		inicial = n->serviceClient<bender_srvs::Dummy>("left_arm/posicion_inicial");
		pre1 = n->serviceClient<bender_srvs::Dummy>("left_arm/posicion_premanipulacion1");
		pre2 = n->serviceClient<bender_srvs::Dummy>("left_arm/posicion_premanipulacion2");
		post1 = n->serviceClient<bender_srvs::Dummy>("left_arm/posicion_postmanipulacion1");
	}
	else if(brazo == 2){// brazo derecho
		abrir = n->serviceClient<bender_srvs::Dummy>("right_arm/abrir_grip");
		cerrar = n->serviceClient<bender_srvs::LoadMode>("right_arm/cerrar_grip");
		grasp = n->serviceClient<bender_srvs::PlanningGoalCartesian>("right_arm/grasp");
		orientar = n->serviceClient<bender_srvs::Dummy>("right_arm/orientar_grip");
		inicial = n->serviceClient<bender_srvs::Dummy>("right_arm/posicion_inicial");
		pre1 = n->serviceClient<bender_srvs::Dummy>("right_arm/posicion_premanipulacion1");
		pre2 = n->serviceClient<bender_srvs::Dummy>("right_arm/posicion_premanipulacion2");
		post1 = n->serviceClient<bender_srvs::Dummy>("right_arm/posicion_postmanipulacion1");
	}
	bender_srvs::Dummy dum;
	bender_srvs::LoadMode loadm;
	bender_srvs::PlanningGoalCartesian xyz;

	inicial.call(dum);

	while(contador < intentos){

		pre1.call(dum);

		if(brazo == 2){// brazo derecho
			xyz.request.x = 40;
			xyz.request.y = -30;
			xyz.request.z = 70;
			grasp.call(xyz);
//			ROS_INFO("paso1:  \n");

			MoverMotor(-1.2,0.5,brazo,4);
//			ROS_INFO("paso2:  \n");

			MoverMotor(-1.1,0.5,brazo,5);
//			ROS_INFO("paso3:  \n");

			abrir.call(dum);
//			ROS_INFO("paso4:  \n");

			xyz.request.x = 40;
			xyz.request.y = -0;
			xyz.request.z = 72;
			grasp.call(xyz);
//			ROS_INFO("paso5:  \n");

			loadm.request.loadmode = 0;
			cerrar.call(loadm);

			if(agarroLaLata(brazo)){

//				brazo de la base
				SoltarSuave(0.1);
				inicialbase.call(dum);
				sleep(2);

//				brazo derecho o izquierdo
				orientar.call(dum);
				pre1.call(dum);
				break;
			}
			else{
				contador++;
				ROS_INFO("Intento fallido N%d", contador);
			}
		}
	}

}

bool agarroLaLata(int brazo){

	// si el brazo derecho agarro la lata, retorna true, si no false
	if (brazo == 1 || brazo == 2){
		double carga[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
		LeerCarga(carga,brazo);
		ROS_INFO("LoadL: %f -- LoadR: %f", carga[6],carga[7]);
		if (fabs(carga[6]) > 0.3 || fabs(carga[7]) > 0.3){
			ROS_INFO("AGARRO!");
			return true;
		}
		else{
			ROS_INFO("NO AGARRO!");
			return false;
		}
	}
	else if (brazo == 3){
		double carga[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
		LeerCarga(carga,brazo);
		ROS_INFO("LoadL: %f -- LoadR: %f", carga[4],carga[5]);
		if (fabs(carga[4]) > 0.3 || fabs(carga[5]) > 0.3){
			ROS_INFO("AGARRO!");
			return true;
		}
		else{
			ROS_INFO("NO AGARRO!");
			return false;
		}
	}
	return false;

}

void LeerCarga(double carga[],int brazo){
	// inicializa el vector
	ros::ServiceClient client;
	int motores = 0;

	if(brazo == 1){ // si 1 brazo izquierdo
		client = n->serviceClient<bender_srvs::States>("left_arm_joints/states");
		motores = 8;
	}
	else if (brazo == 2){// si 2 brazo derecho
		client = n->serviceClient<bender_srvs::States>("right_arm_joints/states");
		motores = 8;
	}
	else if (brazo == 3){
		client = n->serviceClient<bender_srvs::States>("bottom_arm_joints/states");
		motores = 6;
	}

	for (int i=0; i<motores; i++)
	{
		//ROS_INFO("iniciando: %d",i);
		carga[i]=0;
	}

	bender_srvs::States srv;
	srv.request.select.resize(motores);
	for (int i=0; i<motores; i++)
	{
		//ROS_INFO("seteando: %d",i);
		srv.request.select[i] = true;
	}

	if (client.call(srv))
	{
		for (int i=0; i<motores ; i++)
		{
			//ROS_INFO("copiando: %d",i);
			carga[i]=srv.response.load[i];
		}
	}
	else
	{
		ROS_INFO("Failed to call service: left_arm_joints/states");
	}
}

bool LevantarObjeto(){
	//Cierra el grip y cuando lo agarra (torque mayor que UMBRAL) lo levanta del suelo 2 centimetros para realizar la proxima accion
	AgarrarObjeto();
	if (agarroLaLata(3)){
		MoverMotor(0.3,0.3,3,3);
		return true;
	}
	else {
		SoltarObjeto();
		return false;
	}

}

void PosicionReposo(){
	bender_srvs::Dummy dum;
	bender_srvs::Onoff ono;

	ono.request.select = false;

	inicialbase.call(dum);
	ROS_INFO("0");
	sleep(1);
	MoverGripBaseAng(2.4,0.4);
	ROS_INFO("1");
	sleep(3);
	MoverMotor(1.0,0.4,3,1);
	ROS_INFO("2");
	sleep(3);
	MoverMotor(-2.1,0.4,3,2);
	ROS_INFO("3");
	sleep(5);
	MoverMotor(-1.1,0.4,3,1);
	ROS_INFO("4");
	sleep(5);
	torqueoff.call(ono);
	ROS_INFO("Posicion de reposo");
}

void SalirPosicionReposo(){
	bender_srvs::Dummy dum;
	bender_srvs::Onoff ono;

	ROS_INFO("Saliendo de la posicion de reposo");
	ono.request.select = true;
	torqueoff.call(ono);

	MoverMotor(-2.1,0.4,3,2);
	MoverMotor(-1.1,0.4,3,1);
	sleep(5);
	MoverMotor(1.1,0.4,3,1);
	sleep(5);
	MoverMotor(1.6,0.4,3,2);
	sleep(5);
	inicialbase.call(dum);
}

void EntregarBrazoIzquierdo(){
		bender_srvs::setGoalBaseArm srv;
	bender_srvs::AngVel ang;
	srv.request.x = 35;
	srv.request.y = -10;
	srv.request.z = 40;
	ang.request.angle = -120;

	plan.call(srv);
	giro.call(ang);
}

void EntregarBrazoDerecho(){
	bender_srvs::setGoalBaseArm srv;
	bender_srvs::AngVel ang;
	srv.request.x = 35;
	srv.request.y = 7;
	srv.request.z = 40;
	ang.request.angle = 120;

	plan.call(srv);
	giro.call(ang);
}

void AgarrarObjeto(){
	bender_srvs::manipBaseArm action;
	action.request.action = 0;
	agarre.call(action);
}

void SoltarObjeto(){
	bender_srvs::manipBaseArm action;
	action.request.action = 1;
	agarre.call(action);

}

void SoltarSuave(double soltar){
	ros::ServiceClient load = n->serviceClient<bender_srvs::TorqueLimit>("bottom_arm_joints/set_torque_limit");

	bender_srvs::TorqueLimit srv;

	srv.request.torque_limit = soltar;
	srv.request.motor_name = "m_5";
	load.call(srv);
	srv.request.motor_name = "m_6";
	load.call(srv);
}

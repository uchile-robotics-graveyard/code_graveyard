#include "bender_planning_old/planner.h"

double M=0;
double N=0;
double a=0;
double b=0;

static jmp_buf ring;

void timeout(int sig)
{
  longjmp(ring, 1);
}

Uarm::Uarm()
:spinner(1),
 nh()
{

	ros::AsyncSpinner spinner(1);

	cartesian_service = nh.advertiseService("/right_arm/plan_cartesian", &Uarm::GoalCartesianService,this);
	grasp_service = nh.advertiseService("/right_arm/grasp", &Uarm::GraspService,this);
	state_service = nh.advertiseService("/right_arm/plan_state", &Uarm::GoalStateService,this);
	gripang_service = nh.advertiseService("/right_arm/mover_grip_ang", &Uarm::MoverGripAngService,this);
	munecaang_service = nh.advertiseService("/right_arm/mover_right_muneca_ang", &Uarm::MoverMunecaService,this);
	abrirgrip_service = nh.advertiseService("/right_arm/abrir_grip", &Uarm::AbrirGripBrazoService,this);
	cerrargrip_service = nh.advertiseService("/right_arm/cerrar_grip", &Uarm::CerrarGripBrazoService,this);
	orientargrip_service = nh.advertiseService("/right_arm/orientar_grip", &Uarm::OrientarGripService,this);
	posicion_reposo_service = nh.advertiseService("/right_arm/posicion_reposo", &Uarm::PosicionReposoService,this);
	salir_posicion_reposo_service = nh.advertiseService("/right_arm/salir_posicion_reposo", &Uarm::SalirPosicionReposoService,this);
	posicion_inicial_service = nh.advertiseService("/right_arm/posicion_inicial", &Uarm::PosicionInicialService,this);
	posicion_pre1_service = nh.advertiseService("/right_arm/posicion_premanipulacion1", &Uarm::PosicionPre1Service,this);
	posicion_pre2_service = nh.advertiseService("/right_arm/posicion_premanipulacion2", &Uarm::PosicionPre2Service,this);
	posicion_post1_service = nh.advertiseService("/right_arm/posicion_postmanipulacion1", &Uarm::PosicionPost1Service,this);
	torque_enable_service = nh.advertiseService("/right_arm/torque_enable", &Uarm::TorqueEnableService,this);
	feedback_service = nh.advertiseService("/right_arm/feedback", &Uarm::FeedbackService, this);
	grasp_feedback_service = nh.advertiseService("/right_arm/feedback_grasp", &Uarm::GraspFeedbackService, this);

	entregar_papel_service = nh.advertiseService("/right_arm/entregar_papel", &Uarm::EntregarPapelService,this);

	command_pub = nh.advertise<bender_msgs::Command>("right_arm_joints/command", 1000);

	lastx=0;
	lasty=0;
	lastz=0;
	lastxg=0;
	lastyg=0;
	lastzg=0;

	apretar = false;

  	ROS_INFO("Ready to Plan");

}

bool Uarm::GoalCartesianService(bender_srvs::PlanningGoalCartesian::Request &req,bender_srvs::PlanningGoalCartesian::Response &res){
	MoverBrazoB(req.x,req.y,req.z,2,false);
	double qi[4];
	while(leerPoseServo(qi))
		;
	MoverMotor(qi[0]+0.04,0.05,0);
	return true;
}
bool Uarm::GraspService(bender_srvs::PlanningGoalCartesian::Request &req,bender_srvs::PlanningGoalCartesian::Response &res){
	MoverBrazoB(req.x,req.y,req.z,2,true);
	double qi[4];
	while(leerPoseServo(qi))
		;
	MoverMotor(qi[0]+0.04,0.05,0);
	return true;
}
bool Uarm::GoalStateService(bender_srvs::PlanningGoalState::Request &req,bender_srvs::PlanningGoalState::Response &res){
	MoverBrazoAng(req.s0,req.s1,req.s2,req.s3,1);
	return true;
}
bool Uarm::MoverGripAngService(bender_srvs::AngVel::Request &req,bender_srvs::AngVel::Response &res){
	apretar = false;
	sleep(0.5);
	MoverGripAng(req.angle,req.velocity);
	return true;
}
bool Uarm::MoverMunecaService(bender_srvs::AngVel::Request &req,bender_srvs::AngVel::Response &res){
	MoverMuneca(req.angle,req.velocity);
	return true;
}
bool Uarm::OrientarGripService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	OrientarGrip();
	ROS_INFO("Gripper orientado");
	return true;
}
bool Uarm::AbrirGripBrazoService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	AbrirGripBrazo();
	return true;
}
bool Uarm::CerrarGripBrazoService(bender_srvs::LoadMode::Request &req,bender_srvs::LoadMode::Response &res){
	res.taken = CerrarGripBrazo(req.loadmode);
	return true;
}
bool Uarm::PosicionReposoService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	PosicionReposo();
	return true;
}
bool Uarm::SalirPosicionReposoService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	SalirPosicionReposo();
	return true;
}
bool Uarm::PosicionInicialService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	//MoverGripAng(0.3,0.3);
	MoverMotor(0.0,0.3,4);
	double qi[4];
	double pos[3];
	while(leerPoseServo(qi))
		;
	CinematicaDirecta(qi,pos);

	if(pos[0]>10)
		MoverMuneca(1.4,0.6);
	if(-pos[1]<30){
		MoverMotor(0.0,0.4,1);
		MoverMotor(0.0,0.4,2);
	}
	if(pos[0]>15)
		MoverBrazoAng(-0.6,0.0,0.0,1.9,1);
	MoverBrazoAng(0.0,0.1,0.0,0.0,1);
	MoverMuneca(0.0,0.6);
	MoverBrazoAng(0.0,0.0,0.0,0.0,1);
	return true;
}
bool Uarm::PosicionPre1Service(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	MoverMotor(0.1,0.1,1);
	MoverMuneca(1.4,0.6);
	MoverBrazoAng(-0.6,0.0,0.0,1.9,1);
	return true;
}
bool Uarm::PosicionPre2Service(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	MoverBrazoAng(0.2,0.0,0.0,1.9,1);
	//MoverMuneca(0.0,0.6);
	return true;
}
bool Uarm::PosicionPost1Service(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	MoverMuneca(1.4,0.6);
	double qi[4];
	while(leerPoseServo(qi))
		;
	MoverBrazoAng(qi[0],qi[1],qi[2],1.9,1);
	return true;
}
bool Uarm::TorqueEnableService(bender_srvs::TorqueEnable::Request &req, bender_srvs::TorqueEnable::Response &res){
	torqueOnOff(req.torque_enable);
	return true;
}

bool Uarm::EntregarPapelService(bender_srvs::Dummy::Request &req,bender_srvs::Dummy::Response &res){
	MoverMuneca(1.4,0.6);
	MoverBrazoAng(-0.6,0.0,0.0,1.9,1);
	MoverMuneca(0.0,0.6);
	MoverBrazoAng(0.4,0.0,0.0,1.9,1);
	OrientarGrip();
	sleep(2);
	MoverGripAng(0.5,0.3);
	sleep(2);
	MoverGripAng(0.0,0.3);
	usleep(500000);
	MoverMuneca(0.0,0.3);
	usleep(500000);
	MoverBrazoAng(-0.6,0.0,0.0,1.9,1);
	MoverBrazoAng(0.0,0.0,0.0,0.0,1);
	torqueOnOff(0);
	return true;
}

bool Uarm::FeedbackService(bender_srvs::Feedback::Request &req, bender_srvs::Feedback::Response &res){
	double qi[4];
	double pos[]={lastx, lasty, lastz};
	leerPoseServo(qi);
	if(lastx==0 || lasty==0 || lastz==0)
		CinematicaDirecta(qi,pos);
	MoverBrazoB(pos[0]+req.xerr,pos[1]+req.yerr,pos[2]+req.zerr,2,false);
	return true;
}
bool Uarm::GraspFeedbackService(bender_srvs::Feedback::Request &req,bender_srvs::Feedback::Response &res){
	double qi[4];
	double pos[]={lastxg, lastyg, lastzg};
	leerPoseServo(qi);
	if(lastxg==0 || lastyg==0 || lastzg==0)
		CinematicaDirectaGrasp(qi,pos);
	MoverBrazoB(pos[0]+req.xerr,pos[1]+req.yerr,pos[2]+req.zerr,2,true);
	return true;
}


void Uarm::PosicionReposo(){
	MoverMuneca(1.4,0.7);
	MoverBrazoAng(-0.47, 0.0, 0.0, 0.95,1);
	MoverBrazoAng(-0.47, -0.29, 0.0, 0.95,1);
	//MoverMotor(0.15,0.3,4);
}
void Uarm::SalirPosicionReposo(){
	//MoverMotor(0.0,0.3,4);
	MoverBrazoAng(-0.47, 0.0, 0.0, 0.95,1);
	MoverBrazoAng(0.0, 0.0, 0.0, 0.0,1);
	MoverMuneca(0.0,0.7);
	sleep(1);
	torqueOnOff(0);
}
bool Uarm::MoverBrazoB(double x, double y, double z, int Brazo, bool PlanGrip){
	//printf("planificando hasta el punto: %f %f %f \n",x,y,z);
	if(PlanGrip){
		lastxg=x;
		lastyg=y;
		lastzg=z;
	}
	else{
		lastx=x;
		lasty=y;
		lastz=z;
	}
	double qi[5];
	double po[]={x,y,z};
	while(leerPoseServo(qi))
		;
	qi[4]=0;
	//printf("las poses iniciales son: %f %f %f %f \n",qi[0]*180/PI,qi[1]*180/PI,qi[2]*180/PI,qi[3]*180/PI);
	OMNode * Map=new OMNode(0,0,0,0);
	double *tr = NULL;
	bool success = false;
	struct sigaction action;
	action.sa_handler = timeout;
	action.sa_flags = SA_RESTART | SA_NODEFER;

	sigaction(SIGALRM,&action,NULL);

	if(setjmp(ring) != 0) {
		// sigaction(SIGALRM, &action, NULL);
		return false;
	}

	alarm(10);
	tr = AppStar(qi,po,Map,2,&success,PlanGrip);
	alarm(0);

	//printf("primera instrucción: %f %f %f %f %f \n",tr[1]*180/PI,tr[2]*180/PI,tr[3]*180/PI,tr[4]*180/PI,tr[5]*180/PI);
	//printf("ultima instrucción: %f %f %f %f %f \n",tr[1+5*((int)tr[0]-1)]*180/PI,tr[2+5*((int)tr[0]-1)]*180/PI,tr[3+5*((int)tr[0]-1)]*180/PI,tr[4+5*((int)tr[0]-1)]*180/PI,tr[5+5*((int)tr[0]-1)]*180/PI);
	/*
	for(int i=1; i<=(int)tr[0]; i++){
		printf("instruccion appstar: %f %f %f %f %f \n",tr[1+5*(i-1)]*180/PI,tr[2+5*(i-1)]*180/PI,tr[3+5*(i-1)]*180/PI,tr[4+5*(i-1)]*180/PI,tr[5+5*(i-1)]*180/PI);
		getchar();
	}
	*/
	if(tr[0]<15){
		AccionarBrazo(tr,Brazo);
	}
	else{
		ROS_INFO("Utilizando spline para suavizar la trayectoria");
		torqueOnOff(1);
		AccionarBrazoSpline(tr,Brazo);
	}
	while(1){
		//printf("brazo moviendose");
		if(!BrazoMov(1)){
			//printf("brazo dejo de moverse");
			break;
		}
	}

	//printf("las poses finales son: %f %f %f %f \n",qi[0]*180/PI,qi[1]*180/PI,qi[2]*180/PI,qi[3]*180/PI); 
	return success;
}
void Uarm::AccionarBrazoSpline(double *tr,int Brazo){
	Punto4D *poses=new Punto4D[(int)tr[0]];
	double *distancias=new double[(int)tr[0]];
	distancias[0]=0;
	for(int i=0; i<tr[0]; i++){
		poses[i].x=tr[5*i+1];

		poses[i].y=tr[5*i+2];
		poses[i].z=tr[5*i+3];
		poses[i].w=tr[5*i+4];
		if(i>0)
			distancias[i]=sqrt(pow(poses[i-1].x-poses[i].x,2)+pow(poses[i-1].y-poses[i].y,2)+pow(poses[i-1].z-poses[i].z,2)+pow(poses[i-1].w-poses[i].w,2))+distancias[i-1];
	}
	

	L_Spline1DGen<Punto4D> spline;
	for(int i=0;i<tr[0];i++){
		if (!distancias[i]<0.000001)
			spline.push_back(distancias[i],poses[i]);
	}
	spline.compute();
	
	//QueryTimer Reloj = QueryTimer();

	// EL tiempo de traslacion debe darse o calcularse a partir de la distancia al objetivo !!!!
	double tiempo; // tiempo de traslacion

	tiempo=distancias[int(tr[0])-1];
	printf("Tiempo: %lf \n",tiempo);

	double t=0;
	int n=0;
	double rate=50;
	//El loop se ejecuta a "rate" Hz <-> 1/rate ms
	ros::Rate loop_rate(rate);
	//SetPIDGain(32,0,"right_hombro_2");
	//SetPIDGain(32,0,"right_codo_1");
	//SetPunch("right_hombro_2",0);
	//SetPunch("right_codo_1",0);

	while(n<1 && ros::ok()){
		Punto4D res={0,0,0,0};
		Punto4D resd1={0,0,0,0};
		try {
			spline.evaluate(t,res,resd1);
		} catch(...) {
			printf("spline evaluada fuera del rango permitido\n");
		}
		//printf("pto evaluacion spline: %f, %f, %f, %f \n",res.x,res.y,res.z,res.w);
		//printf("derivada pto evaluacion spline: %f, %f, %f, %f \n\n",resd1.x,resd1.y,resd1.z,resd1.w);
		double angulo[4]={res.x,res.y,res.z,res.w};
		double velocidad[4]={resd1.x,resd1.y,resd1.z,resd1.w};

		//int slope = SelectSlopeLevel(tiempo,t);
		//SetComplianceSlope("right_hombro_1",slope);
		//SetComplianceSlope("right_codo_1",slope);
		//int ID = SelectIDLevel(tiempo,t);
		//SetPIDGain(32,ID,"right_hombro_2");
		//SetPIDGain(32,ID,"right_codo_1");
		//Reloj.Start();
		ROS_INFO("Pose brazo: %f, %f, %f, %f\n",angulo[0],angulo[1],angulo[2],angulo[3]);

		bender_msgs::Command com;

		com.positions.resize(8);
		com.select.resize(8);
		com.speed.resize(8);

		bool pub=true;
		for(int i=0;i<8;i++)
		{
			if(i<4)
			{
				if(!isnan(angulo[i]) && !isnan(velocidad[i])){
					com.positions[i]=angulo[i];
					com.speed[i]=0.5*abs(velocidad[i]);
					com.select[i]=true;
				}
				else
					pub=false;
			}
			else
			{
				com.positions[i]=0;
				com.speed[i]=0;
				com.select[i]=false;
			}
		}
		if(pub){
			command_pub.publish(com);
			ros::spinOnce();
		}
		
		//Siguiente punto en la planificacion
		//La frecuencia de actualizacion es 80 Hz -> T=20ms
		/*
		while(1){
			Reloj.Stop();
			if(Reloj.Duracion()>=0.02)
				break;
		}
		*/
		loop_rate.sleep();
		//Se aumenta el tiempo en 1/rate [s]
		t=t+1/rate;
		if(t>tiempo)
			n=n+1; //Termine
	}
	//SetPIDGain(32,0,"right_hombro_2");
	//SetPIDGain(32,0,"right_codo_1");
	//SetPunch("right_hombro_2",800);
	//SetPunch("right_codo_1",800);
	delete(poses);
	delete(distancias);
}
void Uarm::AccionarBrazo(double *tr,int Brazo){
	double angulo[4];
	int n = (int)tr[0]; // cantidad de poses
	double rate=50;
	//El loop se ejecuta a "rate" Hz <-> 1/rate ms
	ros::Rate loop_rate(rate);
	//SetPIDGain(32,0,"right_hombro_2");
	//SetPIDGain(32,0,"right_codo_1");
	//SetPunch("right_hombro_2",800);
	//SetPunch("right_codo_1",800);
	int it=0;
	while(it<n && ros::ok()){;
		angulo[0]=tr[5*it+1];
		angulo[1]=tr[5*it+2];
		angulo[2]=tr[5*it+3];
		angulo[3]=tr[5*it+4];

		//SetPIDGain(32,32,"right_hombro_2");
		//SetPIDGain(32,32,"right_codo_1");
		ROS_INFO("Pose brazo: %f, %f, %f, %f\n",angulo[0],angulo[1],angulo[2],angulo[3]);

		bender_msgs::Command com;

		com.positions.resize(8);
		com.select.resize(8);
		com.speed.resize(8);

		bool pub=true;
		for(int i=0;i<8;i++)
		{
			if(i<4)
			{
				if(angulo[i]!=NAN){
					com.positions[i]=angulo[i];
					com.speed[i]=0.1;
					com.select[i]=true;
				}
				else
					pub=false;
			}
			else
			{
				com.positions[i]=0;
				com.speed[i]=0;
				com.select[i]=false;
			}
		}
		if(pub){
			command_pub.publish(com);
			ros::spinOnce();
		}

		loop_rate.sleep();

		it=it+1; //Siguiente pose
	}
	//SetPIDGain(32,0,"right_hombro_2");
	//SetPIDGain(32,0,"right_codo_1");
}
void Uarm::MoverBrazoAng(double q0, double q1, double q2,double q3,int Brazo){
	double qi[4];
	double qo[4];
	qo[0]=q0;
	qo[1]=q1;
	qo[2]=q2;
	qo[3]=q3;
	//ROS_INFO("Pose objetivo asignada");
	if(Brazo==1){
		while(leerPoseServo(qi))
			ROS_INFO("Intentando obtener pose actual");
		ROS_INFO("Entrando a planificar");
		torqueOnOff(1);
		planificadorDirecto(qi,qo);
	}

	while(1){
		//printf("brazo moviendose");
		if(!BrazoMov(1)){
			//printf("brazo dejo de moverse");
			break;
		}
	}
}
// Mover Gripper Brazo derecho(26Junio2011)
void Uarm::MoverGripAng(double angulo, double velocidad){

	//FUNCION DE ROS PARA MOTORES IZQ Y DER DE LA MANO
	//ros::Publisher command_pub = nh.advertise<bender_msgs::Command>("right_arm_joints/command", 1000);

	bender_msgs::Command com;

	com.positions.resize(8);
	com.select.resize(8);
	com.speed.resize(8);

	for(int i=0;i<8;i++)
	{
		com.positions[i] = 0;
		com.speed[i] = 0;
		com.select[i]=false;
		if(i==6)
		{
			com.positions[i] = -1*angulo;
			com.speed[i] = velocidad;
			com.select[i]=true;
		}
		if (i==7)
		{
			com.positions[i] = angulo;
			com.speed[i] = velocidad;
			com.select[i]=true;
		}
	}

	command_pub.publish(com);
	usleep(100000);

	while(1)
		if(!BrazoMov(1)) break;

	ros::spinOnce();
}
//Mover muñeca del brazo derecho angulo en radianes, velocidad en rad/sec (recomendado 0.66)
void Uarm::MoverMuneca(double angulo, double velocidad){

	//FUNCION DE ROS PARA MOVER right_muneca.
	//ros::Publisher command_pub = nh.advertise<bender_msgs::Command>("right_arm_joints/command", 1000);

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

	command_pub.publish(com);

	usleep(100000);

	while(1)
		if(!BrazoMov(1)) break;
	ros::spinOnce();
}
void Uarm::MoverMotor(double angulo, double velocidad, int motor){

	//FUNCION DE ROS PARA MOVER servo dado por el indice motor.
	bender_msgs::Command com;

	com.positions.resize(8);
	com.select.resize(8);
	com.speed.resize(8);

	for(int i=0;i<8;i++)
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

	command_pub.publish(com);

	usleep(100000);

	while(1)
		if(!BrazoMov(1)) break;
	ros::spinOnce();
}
void Uarm::PosInicial(int Brazo){
	printf("Volviendo a pos Inicial \n");
	double qi[4];
	double qo[4];
	qo[0]=qo[1]=qo[2]=qo[3]=0;
	if(Brazo==1){
		while(leerPoseServo(qi))
			;
		planificadorDirecto(qi,qo);
	}
}
void Uarm::torqueOnOff(int OnOff){
	//FUNCION DE ROS PARA ACTIVAR TORQUE DE TODOS LOS MOTORES
	while(!ros::service::waitForService("right_arm_joints/torque_enable", ros::Duration(1.0)) && ros::ok())
	{
		ROS_INFO_STREAM("Waiting for planner service right_arm_joints/torque_enable");
	}

	ros::ServiceClient client = nh.serviceClient<bender_srvs::TorqueEnable>("right_arm_joints/torque_enable");
	bender_srvs::TorqueEnable srv;
	string names[8]={"right_hombro_1","right_hombro_2","right_hombro_3","right_codo_1","right_codo_2","right_muneca","right_dedo_2","right_dedo_1"};
	for(int i=0; i<6;i++)
	{
		srv.request.motor_name=names[i];
		srv.request.torque_enable=OnOff;
		if (!client.call(srv))
			ROS_ERROR("Failed to call service right_arm_joints/torque_enable in Uarm::torqueOnOff");
	}
}
void Uarm::torqueMotorOnOff(string name,int OnOff){
	//FUNCION DE ROS PARA ACTIVAR TORQUE DE UN MOTOR
	while(!ros::service::waitForService("right_arm_joints/torque_enable", ros::Duration(1.0)) && ros::ok())
	{
		ROS_INFO_STREAM("Waiting for planner service right_arm_joints/torque_enable");
	}

	ros::ServiceClient client = nh.serviceClient<bender_srvs::TorqueEnable>("right_arm_joints/torque_enable");
	bender_srvs::TorqueEnable srv;

	srv.request.motor_name=name;
	srv.request.torque_enable=OnOff;
	if (!client.call(srv))
		ROS_ERROR("Failed to call service right_arm_joints/torque_enable in Uarm::torqueMotorOnOff");

}
void Uarm::SetPIDGain(int P, int ID, string name){
	while(!ros::service::waitForService("right_arm_joints/set_compliance_slope", ros::Duration(1.0)) && ros::ok())
	{
		ROS_INFO_STREAM("Waiting for planner service right_arm_joints/set_compliance_slope");
		ROS_INFO_STREAM("Waiting for planner service right_arm_joints/set_compliance_margin");
	}
	ros::ServiceClient client_slope = nh.serviceClient<bender_srvs::Slope>("right_arm_joints/set_compliance_slope");
	ros::ServiceClient client_margin = nh.serviceClient<bender_srvs::Margin>("right_arm_joints/set_compliance_margin");
	bender_srvs::Slope srv;
	bender_srvs::Margin srv2;

	srv.request.motor_name = name;
	srv.request.slope = P;
	srv2.request.motor_name = name;
	srv2.request.margin = ID;
	if (!client_slope.call(srv))
		ROS_ERROR("Failed to call service right_arm_joints/set_compliance_slope in Uarm::SetPIDGain");
	if (!client_margin.call(srv2))
		ROS_ERROR("Failed to call service right_arm_joints/set_compliance_margin in Uarm::SetPIDGain");
}
/*void Uarm::SetComplianceSlope(string name,int slope){
	//FUNCION DE ROS PARA SETEAR LOS SLOPES DE UN MOTOR
	while(!ros::service::waitForService("right_arm_joints/set_compliance_slope", ros::Duration(1.0)))
	{
		ROS_INFO_STREAM("Waiting for planner service right_arm_joints/set_compliance_slope");
	}
	ros::ServiceClient client = nh.serviceClient<bender_srvs::Slope>("right_arm_joints/set_compliance_slope");
	bender_srvs::Slope srv;

	srv.request.motor_name = name;
	srv.request.slope = slope;
	if (!client.call(srv))
		ROS_ERROR("Failed to call service right_arm_joints/set_compliance_slope in Uarm::SetComplianceSlope");
}
void Uarm::SetAllComplianceSlope(int slope){
	//FUNCION DE ROS PARA SETEAR LOS SLOPES DE TODOS LOS MOTORES
	while(!ros::service::waitForService("right_arm_joints/set_compliance_slope", ros::Duration(1.0)))
	{
		ROS_INFO_STREAM("Waiting for planner service right_arm_joints/set_compliance_slope");
	}

	ros::ServiceClient client = nh.serviceClient<bender_srvs::Slope>("right_arm_joints/set_compliance_slope");
	bender_srvs::Slope srv;

	string names[8]={"right_hombro_1","right_hombro_2","right_hombro_3","right_codo_1","right_codo_2","right_muneca","right_dedo_2","right_dedo_1"};
	for(int i=0; i<8;i++)
	{
		srv.request.motor_name = names[i];
		srv.request.slope = slope;
		if (!client.call(srv))
			ROS_ERROR("Failed to call service right_arm_joints/set_compliance_slope in Uarm::SetAllComplianceSlope");
	}
}
int Uarm::SelectSlopeLevel(double total, double actual){

	double x=100.0*actual/total;

	//int y=(int)(atan(-x/20+2.9)*24+34);
	int y=(int)(64-(1-exp(-1.0/4500.0*x*x))*64);
	printf("x: %f | slope: %d\n",x,y);
	return y;
}
*/
void Uarm::SetPunch(string name,int value){
	//FUNCION DE ROS PARA SETEAR EL PUNCH DE UN MOTOR
	while(!ros::service::waitForService("right_arm_joints/set_compliance_punch", ros::Duration(1.0)) && ros::ok())
	{
		ROS_INFO_STREAM("Waiting for planner service right_arm_joints/set_compliance_punch");
	}
	ros::ServiceClient client = nh.serviceClient<bender_srvs::Punch>("right_arm_joints/set_compliance_punch");
	bender_srvs::Punch srv;
	srv.request.motor_name = name;
	srv.request.punch = value;
	if (!client.call(srv))
		ROS_ERROR("Failed to call service right_arm_joints/set_compliance_punch in Uarm::SetPunch");
}
void Uarm::SetAllPunch(int value)
{
	//FUNCION DE ROS PARA SETEAR EL PUNCH DE TODOS LOS MOTORES
	while(!ros::service::waitForService("right_arm_joints/set_compliance_slope", ros::Duration(1.0)) && ros::ok())
	{
		ROS_INFO_STREAM("Waiting for planner service right_arm_joints/set_compliance_punch");
	}
	ros::ServiceClient client = nh.serviceClient<bender_srvs::Punch>("right_arm_joints/set_compliance_punch");
	bender_srvs::Punch srv;
	string names[8]={"right_hombro_1","right_hombro_2","right_hombro_3","right_codo_1","right_codo_2","right_muneca","right_dedo_2","right_dedo_1"};
	for(int i=0; i<8;i++)
	{
		srv.request.motor_name = names[i];
		srv.request.punch = value;
		if (!client.call(srv))
			ROS_ERROR("Failed to call service right_arm_joints/set_compliance_punch in Uarm::SetAllPunch");
	}
}

int Uarm::SelectIDLevel(double total, double actual){

	double x=100.0*actual/total;

	//int y=(int)(atan(-x/20+2.9)*24+34);
	int y=(int)((1-exp(-1.0/4500.0*x*x))*32);
	//printf("x: %f | slope: %d\n",x,y);
	return y;
}
void Uarm::AbrirCerrarGrip(int Abrir)
{
	apretar = false;
	if (Abrir)
		MoverGripAng(90*M_PI/180.0,0.66);
	else
		MoverGripAng(0.0,0.66);
}
void Uarm::AbrirGripBrazo()
{
	apretar = false;
	MoverGripAng(60*M_PI/180.0,0.66);
}
void Uarm::OrientarGrip(void)
{
	ROS_INFO("Orientando Grip");
	double matrizRot[3][3];
	double q[4];
	while(leerPoseServo(q))
		;
	matrizRotacion(matrizRot,q); // Esto calcula la matriz homogenea de rotacion 

	//ANGULO EN EL QUE SE DEBE ROTAR right_codo_2 (ROLL)
	double anguloZ = -1*asin(matrizRot[2][1]); // Esto es la componente Z(XYZ) de V(UVW)
	//ANGULO EN EL QUE SE DEBE ROTAR right_muneca (PITCH)
	double anguloY = asin(matrizRot[2][2]); //Esto es la componente Z(XYZ) de W(UVW)
	// Enviar el comando al servo del right_codo_2 y right_muneca
	//FUNCION DE ROS PARA PUBLICAR ANGULO DE LA right_muneca Y EL CODO
	bender_msgs::Command com;
	com.positions.resize(8);
	com.select.resize(8);
	com.speed.resize(8);
	for(int i=0;i<8;i++)
	{
		com.positions[i] = 0;
		com.speed[i] = 0;
		com.select[i]=false;
		if(i==4) //right_codo_2
		{
			//ROS_INFO("Llenando mensaje para right_codo_2: %f",anguloZ);
			com.positions[i] = anguloZ;
			com.speed[i] = 0.6;
			com.select[i]=true;
		}
		if (i==5) //right_muneca
		{
			//ROS_INFO("Llenando mensaje para right_muneca: %f",anguloY);
			com.positions[i] = anguloY-CorrAngMuneca*M_PI/180.0;
			com.speed[i] = 0.6;
			com.select[i]=true;
		}
	}
	command_pub.publish(com);

	usleep(200000);

	while(1)
		if(!BrazoMov(1)) break;
	ros::spinOnce();
}
int Uarm::CerrarGripBrazo(int loadMode)
{
//======== control de torque para gripper brazo derecho =======
	double load = 0.0; //variables para almacenar lectura de torque
	double loadR = 0.0;
	double loadL = 0.0;
	double loadMax;
	double k0 = 0.1; //Constante de offset para función exponencial
	double k1 = 20; //Constante de ganancia para función exponencial
	double k2 = 3000; //Modificar enter 1000(pasos grandes) y 5000(pasos chicos) Constante para función exponencial
	double ang = 60; //angulo de apertura inicial
	double velocidad = 0.3; //Velocidad en rad/seg (de 0 a 6.67)
	
	AbrirGripBrazo();
	if (loadMode==0) loadMax=0.4; //Si modo liviano para apretar
	else loadMax=0.5; //Si modo fuerte

	ros::ServiceClient client = nh.serviceClient<bender_srvs::States>("right_arm_joints/states");
	bender_srvs::States srv;
	srv.request.select.resize(8);
	for(int i=0; i<6;i++)
		srv.request.select[i]=false;
	for(int i=6; i<8;i++)
		srv.request.select[i]=true;

	while (abs(loadR)<loadMax && abs(loadL)<loadMax){
	 //Función sigmoidal para suavizar el cerrado del gripper cuando esté cerca al objeto
		ang = ang - (1-exp(-1/k2*ang*ang))*k1 + k0;
		if (ang<=5) break;
		MoverGripAng(ang*M_PI/180.0, velocidad);
		usleep(50000);
		while(!ros::service::waitForService("right_arm_joints/states", ros::Duration(1.0)) && ros::ok())
		{
			ROS_INFO_STREAM("Waiting for planner service right_arm_joints/states");
		}
		if (client.call(srv))
		{
			loadR = srv.response.load[6];
			loadL = srv.response.load[7];
		}
		else
		{
			ROS_ERROR("Failed to call service right_arm_joints/states in Uarm::CerrarGripBrazo");
		}
		load = 0.5*(abs(loadR)+abs(loadL));
		ROS_INFO("LOADR: %f",loadR);
		ROS_INFO("LOADL: %f",loadL);
	}
	if (ang<=5) {
		printf("Objeto no tomado... Angulo final=%f\n",ang); 
		apretar = false;
		return ERRORvar;}
	else {printf("Objeto tomado. Angulo final=%f - Carga=%f\n",ang,load);
		apretar = true;
		return OKvar;}
	//======== control de torque para gripper brazo derecho=======
}
bool Uarm::leerMoving(string name)
{
	//Si el motor está moviendose retorna un boleano verdadero(1)
	//La entrada es el nombre del motor
	
	//FUNCION DE ROS PARA OBTENER SI EL MOTOR SE ESTÁ MOVIENDO
	while(!ros::service::waitForService("right_arm_joints/states", ros::Duration(1.0)) && ros::ok())
	{
		ROS_INFO_STREAM("Waiting for planner service right_arm_joints/states");
	}
	ros::ServiceClient client = nh.serviceClient<bender_srvs::States>("right_arm_joints/states");
	bender_srvs::States srv;
	srv.request.select.resize(8);
	string names[8]={"right_hombro_1","right_hombro_2","right_hombro_3","right_codo_1","right_codo_2","right_muneca","right_dedo_2","right_dedo_1"};
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
		ROS_ERROR("Failed to call service right_arm_joints/states in Uarm::leerMoving");
	}

	return 0;
}
bool Uarm::BrazoMov(int Brazo)
{
	//return 0;

	return (leerMoving("right_hombro_1") ||
			 leerMoving("right_hombro_2") ||
			 leerMoving("right_muneca") ||
			 leerMoving("right_hombro_3") ||leerMoving("right_codo_1") ||
			 leerMoving("right_dedo_2") || leerMoving("right_dedo_1"));

}
int Uarm::leerPoseServo(double qi[])
{
	qi[0]=0.0;
	qi[1]=0.0;
	qi[2]=0.0;
	qi[3]=0.0;
	//FUNCION DE ROS PARA LEER LOS ANGULOS DE LOS DISTINTOS MOTORES DEL BRAZO INVOLUCRADOS EN PLANIFICACION
	while(!ros::service::waitForService("right_arm_joints/states", ros::Duration(1.0)) && ros::ok())
	{
		ROS_INFO_STREAM("Waiting for planner service right_arm_joints/states");
	}
	//ROS_INFO("Servicio right_arm_joints/states encontrado");
	ros::ServiceClient client = nh.serviceClient<bender_srvs::States>("right_arm_joints/states");
	//ROS_INFO("Cliente para lectura de poses iniciado");
	bender_srvs::States srv;
	srv.request.select.resize(8);
	for(int i=0; i<4;i++)
		srv.request.select[i]=true;
	for(int i=4; i<8;i++)
		srv.request.select[i]=false;
	//ROS_INFO("Request para lectura de poses rellenado");

	if (client.call(srv))
	{
		//ROS_INFO("Llamada a servicio de lectura de poses exitosa");
		qi[0]=srv.response.state[0];
		qi[1]=srv.response.state[1];
		qi[2]=srv.response.state[2];
		qi[3]=srv.response.state[3];
		//ROS_INFO("pose motores %f %f %f %f",qi[0],qi[1],qi[2],qi[3]);
	}
	else
	{
		ROS_ERROR("Failed to call service right_arm_joints/states in Uarm::leerPoseServo");
	return 1;
	}
	return 0;
}
void Uarm::planificadorDirecto(double qi[],double qo[])
{
	/* Esta funcion realiza una planificacion de trayectoria sin usar cinematica inversa*/
	/* Su entrada es qi: angulos actuales y qo: angulos finales*/
	printf("Entre a planificar!!\n");
	// tiempo de traslacion (fijo ?)
	int i;
	double tiempo=0;
	for (i=0;i<4;i++)
		tiempo+=fabs(qi[i]-qo[i]);
	printf("%lf \n",tiempo);
	if(tiempo<1)
		tiempo+=0.5;
	// coeficientes de los polinomios
	double coef[6][4]; 
	generarCoef(qi,qo,tiempo,coef);
	QueryTimer Reloj = QueryTimer();

	double t=0;
	int n=0;
	double rate=50;

	//ros::Publisher command_pub = nh.advertise<bender_msgs::Command>("right_arm_joints/command", 1000);

	//El loop se ejecuta a "rate" Hz <-> 1/rate ms
	ros::Rate loop_rate(rate);
	//SetPIDGain(32,0,"right_hombro_2");
	//SetPIDGain(32,0,"right_codo_1");
	//SetPunch("right_hombro_2",0);
	//SetPunch("right_codo_1",0);
	while(n<1 && ros::ok()){

		//int slope = 32;
		//SetComplianceSlope("right_hombro_1",slope);
		//SetComplianceSlope("right_codo_1",slope);
		
		bender_msgs::Command com;

		com.positions.resize(8);
		com.select.resize(8);
		com.speed.resize(8);

		for(int i=0;i<8;i++)
		{
			if(i<4)
			{
				com.positions[i] = Angulo(i,t,coef);
				com.speed[i] = 1000*abs(Velocidad(i,t,coef));
				com.select[i]=true;
			}
			else
			{
				com.positions[i] = 0;
				com.speed[i] = 0;
				com.select[i]=false;
			}
		}

		ROS_INFO("Pose brazo: %f, %f, %f, %f\n",com.positions[0],com.positions[1],com.positions[2],com.positions[3]);

		command_pub.publish(com);
		ros::spinOnce();
		loop_rate.sleep();
		//Siguiente punto en la planificacion
		//Se aumenta el tiempo en 1/rate [s]
		t=t+1/rate;
		if(t>tiempo)
			n=n+1; //Termine
	}

}
void Uarm::CinematicaDirecta(double state[], double pos[]){
	double dist[4];

	double ca1=cos(state[0]);
	double ca2=cos(state[1]);
	double ca3=cos(state[2]);
	double ca4=cos(-state[3]);
	double sa1=sin(state[0]);
	double sa2=sin(state[1]);
	double sa3=sin(state[2]);
	double sa4=sin(-state[3]);
	double TF[16];

	TF[0]=ca1*ca3*ca4+sa1*(ca2*sa4-sa2*sa3*ca4);
	TF[1]=ca1*sa3+sa1*sa2*ca3;
	TF[2]=ca1*ca3*sa4+sa1*(-ca2*ca4-sa2*sa3*sa4);
	TF[3]=ca1*(ca3*(-1.5*ca4-35.65*(sa4+0.050491))+1.5)+sa1*(ca2*(35.65*ca4-1.5*sa4+34.2)+sa2*sa3*(1.5*ca4+35.65*(sa4+0.050491))+3);

	TF[4]=-ca2*sa3*ca4-sa2*sa4;
	TF[5]=ca2*ca3;
	TF[6]=sa2*ca4-ca2*sa3*sa4;
	TF[7]=ca2*sa3*(1.5*ca4+35.65*(sa4+0.050491))+sa2*(-35.65*ca4+1.5*sa4-34.2)-3.8;

	TF[8]=ca1*(sa2*sa3*ca4-ca2*sa4)+sa1*ca3*ca4;
	TF[9]=sa1*sa3-ca1*sa2*ca3;
	TF[10]=ca1*(ca2*ca4+sa2*sa3*sa4)+sa1*ca3*sa4;
	TF[11]=ca1*(ca2*(-35.65*ca4+1.5*sa4-34.2)+sa2*sa3*(-1.5*ca4-35.65*(sa4+0.050491))-3)+sa1*(ca3*(-1.5*ca4-35.65*(sa4+0.050491))+1.5);

	TF[12]=0;
	TF[13]=0;
	TF[14]=0;
	TF[15]=1;

	hTranslation(-DistHombroBaseX,DistHombroBaseY,-DistHombroBaseZ,TF);

	hTransform(TF,0,0,0,dist);
	pos[0]=dist[0];
	pos[1]=dist[1];
	pos[2]=dist[2];
}
void Uarm::CinematicaDirectaGrasp(double state[], double pos[]){
	double dist[4];

	double ca1=cos(state[0]);
	double ca2=cos(state[1]);
	double ca3=cos(state[2]);
	double ca4=cos(-state[3]);
	double sa1=sin(state[0]);
	double sa2=sin(state[1]);
	double sa3=sin(state[2]);
	double sa4=sin(-state[3]);
	double TF[16];

	TF[0]=ca1*ca3*ca4+sa1*(ca2*sa4-sa2*sa3*ca4);
	TF[1]=ca1*sa3+sa1*sa2*ca3;
	TF[2]=ca1*ca3*sa4+sa1*(-ca2*ca4-sa2*sa3*sa4);
	TF[3]=ca1*(ca3*(-1.5*ca4-35.65*(sa4+0.050491))+1.5)+sa1*(ca2*(35.65*ca4-1.5*sa4+34.2)+sa2*sa3*(1.5*ca4+35.65*(sa4+0.050491))+3);

	TF[4]=-ca2*sa3*ca4-sa2*sa4;
	TF[5]=ca2*ca3;
	TF[6]=sa2*ca4-ca2*sa3*sa4;
	TF[7]=ca2*sa3*(1.5*ca4+35.65*(sa4+0.050491))+sa2*(-35.65*ca4+1.5*sa4-34.2)-3.8;

	TF[8]=ca1*(sa2*sa3*ca4-ca2*sa4)+sa1*ca3*ca4;
	TF[9]=sa1*sa3-ca1*sa2*ca3;
	TF[10]=ca1*(ca2*ca4+sa2*sa3*sa4)+sa1*ca3*sa4;
	TF[11]=ca1*(ca2*(-35.65*ca4+1.5*sa4-34.2)+sa2*sa3*(-1.5*ca4-35.65*(sa4+0.050491))-3)+sa1*(ca3*(-1.5*ca4-35.65*(sa4+0.050491))+1.5);

	TF[12]=0;
	TF[13]=0;
	TF[14]=0;
	TF[15]=1;

	double anguloYright_muneca=asin(ca1*(ca2*ca4+sa2*sa3*sa4)+sa1*ca3*sa4);
	double anguloZright_muneca=asin(sa1*sa3-ca1*sa2*ca3);
	double rotYright_muneca[16]={	cos(anguloYright_muneca)	,0			,-sin(anguloYright_muneca)	,0,
							0			,1			,0			,0,
							sin(anguloYright_muneca)	,0			,cos(anguloYright_muneca)	,0,
							0			,0			,0			,1};
	double rotZright_muneca[16]={	cos(anguloZright_muneca)	,-sin(anguloZright_muneca),0			,0,
							sin(anguloZright_muneca)	,cos(anguloZright_muneca)	,0			,0,
							0			,0			,1			,0,
							0			,0			,0			,1};
	hMatrixMultip(TF,rotZright_muneca);
	hMatrixMultip(rotZright_muneca,rotYright_muneca);
	hIdentity(TF);
	hMatrixMultip(rotYright_muneca,TF);

	hTranslation(-DistHombroBaseX,DistHombroBaseY,-DistHombroBaseZ,TF);

	//Punta del grip en coord (0,0,-16) respecto de la right_muneca
	hTransform(TF,0,0,-16,dist);
	pos[0]=dist[0];
	pos[1]=dist[1];
	pos[2]=dist[2];
}
Uarm::~Uarm(){}

void matrizRotacion(double matrizRot[][3], double angulos[]){
	double ca1=cos(angulos[0]);
	double ca2=cos(angulos[1]);
	double ca3=cos(angulos[2]);
	double ca4=cos(-angulos[3]);
	double sa1=sin(angulos[0]);
	double sa2=sin(angulos[1]);
	double sa3=sin(angulos[2]);
	double sa4=sin(-angulos[3]);

	matrizRot[0][0]= ca1*ca3*ca4+sa1*(ca2*sa4-sa2*sa3*ca4);
	matrizRot[0][1]= ca1*sa3+sa1*sa2*ca3;
	matrizRot[0][2]= ca1*ca3*sa4+sa1*(-ca2*ca4-sa2*sa3*sa4);
	matrizRot[1][0]= -ca2*sa3*ca4-sa2*sa4;
	matrizRot[1][1]= ca2*ca3;
	matrizRot[1][2]= sa2*ca4-ca2*sa3*sa4;
	matrizRot[2][0]= ca1*(sa2*sa3*ca4-ca2*sa4)+sa1*ca3*ca4;
	matrizRot[2][1]= sa1*sa3-ca1*sa2*ca3;
	matrizRot[2][2]= ca1*(ca2*ca4+sa2*sa3*sa4)+sa1*ca3*sa4;
}
int generarCoef(double qi[],double qo[],double tiempo,double coef[][4]){
	// coef es una matriz de 6 (coeficientes)* 4 (joints)
	int k;
	double vi=0,vo=0,ai=0,ao=0;
	for(k=0;k<4;k++){
	    	coef[0][k]=qi[k];
	    	coef[1][k]=vi;
	    	coef[2][k]=ai;
	    	coef[3][k] = (20*qo[k] - 20*qi[k] -(8*vo+12*vi)*tiempo - (3*ai-ao)*pow(tiempo,2) )/(2*pow(tiempo,3));
	    	coef[4][k] = (30*qi[k] - 30*qo[k] +(14*vo+16*vi)*tiempo + (3*ai-2*ao)*pow(tiempo,2) )/(2*pow(tiempo,4));
	    	coef[5][k] = (12*qo[k] - 12*qi[k] -(6*vo+6*vi)*tiempo - (ai-ao)*pow(tiempo,2) )/(2*pow(tiempo,5));
	}
	/*
	for(k=0;k<6;k++)
		printf("%lf\n",coef[k][4]);
	*/
	return 1;
}
double Angulo(int Joint, double tiempo, double coef[][4]){
	double ang;
	ang = coef[0][Joint]+coef[1][Joint]*tiempo + coef[2][Joint]*pow(tiempo,2)+coef[3][Joint]*pow(tiempo,3)+coef[4][Joint]*pow(tiempo,4)+coef[5][Joint]*pow(tiempo,5);

	return ang;
}
double Velocidad(int Joint, double tiempo, double coef[][4]){
	double vel;
	vel = coef[1][Joint] + 2*coef[2][Joint]*tiempo + 3*coef[3][Joint]*pow(tiempo,2)+4*coef[4][Joint]*pow(tiempo,3)+5*coef[5][Joint]*pow(tiempo,4);

	return vel;
}

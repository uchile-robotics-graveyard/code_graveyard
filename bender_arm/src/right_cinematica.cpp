/*
 * right_cinematica.cpp
 *
 *  Created on: Nov 21, 2013
 *      Author: gonzalo
 */

#include <torque_brazo/torque.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "r_torque_node");
	ros::NodeHandle nh;
	n = &nh;

	ros::Rate loop_rate(30);

	call_torque = n->serviceClient<bender_srvs::TorqueEnable>("right_arm_joints/torque_enable");
	torque_limit = n->serviceClient<bender_srvs::TorqueLimit>("right_arm_joints/torque_limit");
	torque_on = n -> serviceClient<bender_srvs::Onoff>("right_arm_joints/onoff");
	call_pose = n->advertise<bender_msgs::Command>("right_arm_joints/command",1000);

    // VARIABLES

    a = cv::Mat(4,4,CV_64F, cv::Scalar::all(0));
    x0 = cv::Mat(4,1,CV_64F,cv::Scalar::all(0));
    homogenea = cv::Mat(4,4,CV_64F, cv::Scalar::all(0));
	x0.at<double>(0)=0;
	x0.at<double>(1)=0;
	x0.at<double>(2)=0;
	x0.at<double>(3)=1;

	// MASAS
	double M[7];
	M[0] = 0.316; // este es el link del hombro de Bender
	M[1] = 0.216;
	M[2] = 0.282;
	M[3] = 0.256;
	M[4] = 0.243;
	M[5] = 0.222;
	M[6] = 0.430;

//	double efector[3],muneca[3];

//	double cm1[3],cm2[3],cm3[3],cm4[3],cm5[3],cm6[3];
//	double test[3];
	double torque[6];
//	double GG[3];

    // Se inicializan las matrices homogeneas como indentidad
    for(int i=0;i<7;i++){
        	H[i]=cv::Mat::eye(4,4,CV_64F);
        	CM[i]=cv::Mat::eye(4,4,CV_64F);
    }

    onoff.request.select = true;
    torque_on.call(onoff);

    while(ros::ok()){

    	LeerAngulo(th);
    	LeerCarga(carga);
//    	ROS_INFO("ANGULOS LEIDOS");

    	printf("%d--------------------------------------------------------------%d\n",0,0);

//    	DK_joint(efector,th,7);

//    	DK_joint(muneca,th,6);


    	CargaMotoresCalc(torque,th, M);

    	printf("hombroY = %f | load_hombroY = %f | ERROR = %f\n", torque[0],carga[0],abs(torque[0]-carga[0]));
    	printf("hombroX = %f | load_hombroX = %f | ERROR = %f\n", torque[1],carga[1],abs(torque[1]-carga[1]));
    	printf("hombroZ = %f | load_hombroZ = %f | ERROR = %f\n", torque[2],carga[2],abs(torque[2]-carga[2]));
    	printf("codoY   = %f | load_codoY = %f   | ERROR = %f\n", torque[3],carga[3],abs(torque[3]-carga[3]));
    	printf("codoZ   = %f | load_codoZ = %f   | ERROR = %f\n", torque[4],carga[4],abs(torque[4]-carga[4]));
    	printf("munecaY = %f | load_munecaY = %f | ERROR = %f\n", torque[5],carga[5],abs(torque[5]-carga[5]));

    	SoltarJoint(torque,carga);

//    	printf("\n\n");
//    	    for(int i=0;i<4;i++){
//    	        printf("|");
//    	        for(int j=0;j<4;j++){
//    	            printf(" %4f ",a.at<double>(i,j));
//    	        }
//    	        printf("|\n");
//    	    }

//    	GetGravity(GG,6,th);
//    	printf("G: X = %f | Y = %f | Z = %f\n",GG[0],GG[1],GG[2]);

//    	ROS_INFO("DK_gravity");
//    	printf("CM referencial: X = %f | Y = %f | Z = %f\n",cm6[0],cm6[1],cm6[2]);
//    	printf("Centro de Gravedad: X = %f | Y = %f | Z = %f\n",test[0],test[1],test[2]);


		ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}

void GetGravity(double gg[], int joint, double angulos[])
{
	cv::Mat rotx1,roty1, roty2, roty3 ,rotz1, rotz2, G;
	double c0,c1,c2,c3,c4,c5;
	double s0,s1,s2,s3,s4,s5;
	cv::Mat temp;

	rotx1 = cv::Mat::eye(4,4,CV_64F);
	roty1 = cv::Mat::eye(4,4,CV_64F);
	roty2 = cv::Mat::eye(4,4,CV_64F);
	roty3 = cv::Mat::eye(4,4,CV_64F);
	rotz1 = cv::Mat::eye(4,4,CV_64F);
	rotz2 = cv::Mat::eye(4,4,CV_64F);
	G = cv::Mat(4,1,CV_64F, cv::Scalar::all(0));

	G.at<double>(2) = -g;
	G.at<double>(3) = 1;

	double right_pose[8];

	right_pose [0] = angulos[0];
	right_pose [1] = angulos[1];
	right_pose [2] = angulos[2];
	right_pose [3] = angulos[3];
	right_pose [4] = angulos[4];
	right_pose [5] = angulos[5];

	c0 = cos(right_pose[0]);
	s0 = sin(right_pose[0]);
	roty1.at<double>(0,0) = c0;
	roty1.at<double>(0,2) = s0;
	roty1.at<double>(2,0) = -s0;
	roty1.at<double>(2,2) = c0;

	c1 = cos(right_pose[1]);
	s1 = sin(right_pose[1]);
	rotx1.at<double>(1,1) = c1;
	rotx1.at<double>(1,2) = -s1;
	rotx1.at<double>(2,1) = s1;
	rotx1.at<double>(2,2) = c1;

	c2 = cos(right_pose[2]);
	s2 = sin(right_pose[2]);
	rotz1.at<double>(0,0) = c2;
	rotz1.at<double>(0,1) = -s2;
	rotz1.at<double>(1,0) = s2;
	rotz1.at<double>(1,1) = c2;

	c3 = cos(right_pose[3]);
	s3 = sin(right_pose[3]);
	roty2.at<double>(0,0) = c3;
	roty2.at<double>(0,2) = s3;
	roty2.at<double>(2,0) = -s3;
	roty2.at<double>(2,2) = c3;

	c4 = cos(right_pose[4]);
	s4 = sin(right_pose[4]);
	rotz2.at<double>(0,0) = c4;
	rotz2.at<double>(0,1) = -s4;
	rotz2.at<double>(1,0) = s4;
	rotz2.at<double>(1,1) = c4;

	c5 = cos(right_pose[5]);
	s5 = sin(right_pose[5]);
	roty3.at<double>(0,0) = c5;
	roty3.at<double>(0,2) = s5;
	roty3.at<double>(2,0) = -s5;
	roty3.at<double>(2,2) = c5;

	switch(joint)
	{
	case 1: temp = G; break;
	case 2: temp = roty1*G; break;
	case 3: temp = rotx1*roty1*G; break;
	case 4: temp = rotz1*rotx1*roty1*G; break;
	case 5: temp = roty2*rotz1*rotx1*roty1*G; break;
	case 6: temp = rotz2*roty2*rotz1*rotx1*roty1*G; break;
	}

	gg[0] = temp.at<double>(0);
	gg[1] = temp.at<double>(1);
	gg[2] = temp.at<double>(2);
	gg[3] = temp.at<double>(3);

}

double MAcumulada(double M[], int link)
{
	double masa = 0;
	for (int i=6; i>=link; i--)
		{
			masa = masa + M[i];
		}

//	printf ("Masa = %f \n", masa);
	return masa;
}

void CenterGravity(double xyz[], double angulos[], double M[], int link)
{
	double xmean = 0, ymean = 0, zmean = 0;
	double masa = 0;

	cv::Mat temp0,temp1,temp2,temp3,temp4,temp5;

	cv::Mat rotx1,roty1, roty2, roty3 ,rotz1, rotz2;
	double c0,c1,c2,c3,c4,c5;
	double s0,s1,s2,s3,s4,s5;

	rotx1 = cv::Mat::eye(4,4,CV_64F);
	roty1 = cv::Mat::eye(4,4,CV_64F);
	roty2 = cv::Mat::eye(4,4,CV_64F);
	roty3 = cv::Mat::eye(4,4,CV_64F);
	rotz1 = cv::Mat::eye(4,4,CV_64F);
	rotz2 = cv::Mat::eye(4,4,CV_64F);

	double right_pose[8];
	//update joint_state
	right_pose [0] = -angulos[0];
	right_pose [1] = -angulos[1];
	right_pose [2] = -angulos[2];
	right_pose [3] = -angulos[3];
	right_pose [4] = -angulos[4];
	right_pose [5] = -angulos[5];

	c0 = cos(right_pose[0]);
	s0 = sin(right_pose[0]);
	roty1.at<double>(0,0) = c0;
	roty1.at<double>(0,2) = s0;
	roty1.at<double>(2,0) = -s0;
	roty1.at<double>(2,2) = c0;

	c1 = cos(right_pose[1]);
	s1 = sin(right_pose[1]);
	rotx1.at<double>(1,1) = c1;
	rotx1.at<double>(1,2) = -s1;
	rotx1.at<double>(2,1) = s1;
	rotx1.at<double>(2,2) = c1;

	c2 = cos(right_pose[2]);
	s2 = sin(right_pose[2]);
	rotz1.at<double>(0,0) = c2;
	rotz1.at<double>(0,1) = -s2;
	rotz1.at<double>(1,0) = s2;
	rotz1.at<double>(1,1) = c2;

	c3 = cos(right_pose[3]);
	s3 = sin(right_pose[3]);
	roty2.at<double>(0,0) = c3;
	roty2.at<double>(0,2) = s3;
	roty2.at<double>(2,0) = -s3;
	roty2.at<double>(2,2) = c3;

	c4 = cos(right_pose[4]);
	s4 = sin(right_pose[4]);
	rotz2.at<double>(0,0) = c4;
	rotz2.at<double>(0,1) = -s4;
	rotz2.at<double>(1,0) = s4;
	rotz2.at<double>(1,1) = c4;

	c5 = cos(right_pose[5]);
	s5 = sin(right_pose[5]);
	roty3.at<double>(0,0) = c5;
	roty3.at<double>(0,2) = s5;
	roty3.at<double>(2,0) = -s5;
	roty3.at<double>(2,2) = c5;

	//Hombro Y
	H[0].at<double>(0,0)=cos(right_pose[0]);
	H[0].at<double>(0,2)=sin(right_pose[0]);
	H[0].at<double>(1,3)=-23.5;
	H[0].at<double>(2,0)=-sin(right_pose[0]);
	H[0].at<double>(2,2)=cos(right_pose[0]);
	H[0].at<double>(2,3)=123.0;

	//Hombro X
	H[1].at<double>(1,1)=cos(right_pose[1]);
	H[1].at<double>(1,2)=-sin(right_pose[1]);
	H[1].at<double>(1,3)=-3.8;
	H[1].at<double>(2,1)=sin(right_pose[1]);
	H[1].at<double>(2,2)=cos(right_pose[1]);
	H[1].at<double>(2,3)=-3.0;

	// Hombro Z
	H[2].at<double>(0,0)=cos(right_pose[2]);
	H[2].at<double>(0,1)=-sin(right_pose[2]);
	H[2].at<double>(0,3)=1.5;
	H[2].at<double>(1,0)=sin(right_pose[2]);
	H[2].at<double>(1,1)=cos(right_pose[2]);
	H[2].at<double>(2,3)=-16.0;

	// Codo Y
	H[3].at<double>(0,0)=cos(right_pose[3]);
	H[3].at<double>(0,2)=sin(right_pose[3]);
	H[3].at<double>(0,3)=-1.8;
	H[3].at<double>(2,0)=-sin(right_pose[3]);
	H[3].at<double>(2,2)=cos(right_pose[3]);
	H[3].at<double>(2,3)=-18.2;

	// Codo Z
	H[4].at<double>(0,0)=cos(right_pose[4]);
	H[4].at<double>(0,1)=-sin(right_pose[4]);
	H[4].at<double>(0,3)=0.4;
	H[4].at<double>(1,0)=sin(right_pose[4]);
	H[4].at<double>(1,1)=cos(right_pose[4]);
	H[4].at<double>(2,3)=-7.8;

	// Muneca
	H[5].at<double>(0,0)=cos(right_pose[5]);
	H[5].at<double>(0,2)=sin(right_pose[5]);
	H[5].at<double>(0,3)=-1.6;
	H[5].at<double>(2,0)=-sin(right_pose[5]);
	H[5].at<double>(2,2)=cos(right_pose[5]);
	H[5].at<double>(2,3)=-27.9;

	// Link 1
	CM[0].at<double>(1,3)=-2.0;
	CM[0].at<double>(2,3)=-3.25;

	// Link 2
	CM[1].at<double>(0,3)=4.0;
	CM[1].at<double>(2,3)=-11.0;

	// Link 3
	CM[2].at<double>(0,3)=-3.2;
	CM[2].at<double>(2,3)=-13.0;

	// Link 4
	CM[3].at<double>(0,3)=2.7;
	CM[3].at<double>(2,3)=-7.8;

	// Link 5
	CM[4].at<double>(0,3)=-3.2;
	CM[4].at<double>(2,3)=-21.0;

	// Link 6
	CM[5].at<double>(2,3)=-13.0;

	switch(link)
		{
		case 1:
			{
				masa = M[1]+M[2]+M[3]+M[4]+M[5]+M[6];
				temp0 = roty1*CM[0]*x0;
				temp1 = roty1*H[1]*CM[1]*x0;
				temp2 = roty1*H[1]*H[2]*CM[2]*x0;
				temp3 = roty1*H[1]*H[2]*H[3]*CM[3]*x0;
				temp4 = roty1*H[1]*H[2]*H[3]*H[4]*CM[4]*x0;
				temp5 = roty1*H[1]*H[2]*H[3]*H[4]*H[5]*CM[5]*x0;
				xmean = (temp0.at<double>(0)*M[1]+temp1.at<double>(0)*M[2]+temp2.at<double>(0)*M[3]+temp3.at<double>(0)*M[4]+temp4.at<double>(0)*M[5]+temp5.at<double>(0)*M[6])/(masa);
				ymean = (temp0.at<double>(1)*M[1]+temp1.at<double>(1)*M[2]+temp2.at<double>(1)*M[3]+temp3.at<double>(1)*M[4]+temp4.at<double>(1)*M[5]+temp5.at<double>(1)*M[6])/(masa);
				zmean = (temp0.at<double>(2)*M[1]+temp1.at<double>(2)*M[2]+temp2.at<double>(2)*M[3]+temp3.at<double>(2)*M[4]+temp4.at<double>(2)*M[5]+temp5.at<double>(2)*M[6])/(masa);
//				printf("rx = %f | ry = %f | rz = %f \n", xmean, ymean, zmean);
				break;
			}
		case 2:
			{
				masa = M[2]+M[3]+M[4]+M[5]+M[6];
				temp1 = rotx1*CM[1]*x0;
				temp2 = rotx1*H[2]*CM[2]*x0;
				temp3 = rotx1*H[2]*H[3]*CM[3]*x0;
				temp4 = rotx1*H[2]*H[3]*H[4]*CM[4]*x0;
				temp5 = rotx1*H[2]*H[3]*H[4]*H[5]*CM[5]*x0;
				xmean = (temp1.at<double>(0)*M[2]+temp2.at<double>(0)*M[3]+temp3.at<double>(0)*M[4]+temp4.at<double>(0)*M[5]+temp5.at<double>(0)*M[6])/masa;
				ymean = (temp1.at<double>(1)*M[2]+temp2.at<double>(1)*M[3]+temp3.at<double>(1)*M[4]+temp4.at<double>(1)*M[5]+temp5.at<double>(1)*M[6])/masa;
				zmean = (temp1.at<double>(2)*M[2]+temp2.at<double>(2)*M[3]+temp3.at<double>(2)*M[4]+temp4.at<double>(2)*M[5]+temp5.at<double>(2)*M[6])/masa;
//				printf("rx = %f | ry = %f | rz = %f \n", xmean, ymean, zmean);
				break;
			}
		case 3:
			{
				masa = M[3]+M[4]+M[5]+M[6];
				temp2 = rotz1*CM[2]*x0;
				temp3 = rotz1*H[3]*CM[3]*x0;
				temp4 = rotz1*H[3]*H[4]*CM[4]*x0;
				temp5 = rotz1*H[3]*H[4]*H[5]*CM[5]*x0;
				xmean = (temp2.at<double>(0)*M[3]+temp3.at<double>(0)*M[4]+temp4.at<double>(0)*M[5]+temp5.at<double>(0)*M[6])/masa;
				ymean = (temp2.at<double>(1)*M[3]+temp3.at<double>(1)*M[4]+temp4.at<double>(1)*M[5]+temp5.at<double>(1)*M[6])/masa;
				zmean = (temp2.at<double>(2)*M[3]+temp3.at<double>(2)*M[4]+temp4.at<double>(2)*M[5]+temp5.at<double>(2)*M[6])/masa;
//				printf("rx = %f | ry = %f | rz = %f \n", xmean, ymean, zmean);
				break;
			}
		case 4:
			{
				masa = M[4]+M[5]+M[6];
				temp3 = roty2*CM[3]*x0;
				temp4 = roty2*H[4]*CM[4]*x0;
				temp5 = roty2*H[4]*H[5]*CM[5]*x0;
				xmean = (temp3.at<double>(0)*M[4]+temp4.at<double>(0)*M[5]+temp5.at<double>(0)*M[6])/masa;
				ymean = (temp3.at<double>(1)*M[4]+temp4.at<double>(1)*M[5]+temp5.at<double>(1)*M[6])/masa;
				zmean = (temp3.at<double>(2)*M[4]+temp4.at<double>(2)*M[5]+temp5.at<double>(2)*M[6])/masa;
//				printf("rx = %f | ry = %f | rz = %f \n", xmean, ymean, zmean);
				break;
			}
		case 5:
			{
				masa = M[5]+M[6];
				temp4 = rotz2*CM[4]*x0;
				temp5 = rotz2*H[5]*CM[5]*x0;
				xmean = (temp4.at<double>(0)*M[5]+temp5.at<double>(0)*M[6])/(masa);
				ymean = (temp4.at<double>(1)*M[5]+temp5.at<double>(1)*M[6])/(masa);
				zmean = (temp4.at<double>(1)*M[5]+temp5.at<double>(2)*M[6])/(masa);
//				printf("rx = %f | ry = %f | rz = %f \n", xmean, ymean, zmean);
				break;
			}

		case 6:
			{
				temp5 = roty3*CM[5]*x0;
				xmean = (temp5.at<double>(0)*M[6])/M[6];
				ymean = (temp5.at<double>(1)*M[6])/M[6];
				zmean = (temp5.at<double>(2)*M[6])/M[6];
//				printf("rx = %f | ry = %f | rz = %f \n", xmean, ymean, zmean);
				break;
			}
		}
	xyz[0] = xmean;
	xyz[1] = ymean;
	xyz[2] = zmean;

}

void LeerAngulo(double th[])
{

	// inicializa el vector con angulos
	for (int i=0; i<8; i++)
	{
		//ROS_INFO("iniciando: %d",i);
		th[i]=0;
	}

	ros::ServiceClient client = n->serviceClient<bender_srvs::States>("right_arm_joints/states");

	bender_srvs::States srv;
	srv.request.select.resize(8);
	for (int i=0; i<8; i++)
	{
		//ROS_INFO("seteando: %d",i);
		srv.request.select[i] = true;
	}

	if (client.call(srv))
	{
		for (int i=0; i<8 ; i++)
		{
			//ROS_INFO("copiando: %d",i);
			th[i]=srv.response.state[i];
		}
	}
	else
	{
		ROS_INFO("Failed to call service: right_arm_joints/states");
	}

}

void LeerCarga(double carga[])
{
	// inicializa el vector
	for (int i=0; i<8; i++)
	{
		//ROS_INFO("iniciando: %d",i);
		carga[i]=0;
	}

	ros::ServiceClient client = n->serviceClient<bender_srvs::States>("right_arm_joints/states");

	bender_srvs::States srv;
	srv.request.select.resize(8);
	for (int i=0; i<8; i++)
	{
		//ROS_INFO("seteando: %d",i);
		srv.request.select[i] = true;
	}

	if (client.call(srv))
	{
		for (int i=0; i<8 ; i++)
		{
			//ROS_INFO("copiando: %d",i);
			carga[i]=srv.response.load[i];
		}
	}
	else
	{
		ROS_INFO("Failed to call service: right_arm_joints/states");
	}

}

void DK_gravity(double xyz[], int link)
{
	/*
	 * Posiciones de centros de masa de cada link del brazo
	 *
	 * */
//	double joint_pos[3];
	cv::Mat temp2;

	// Link 1
	CM[0].at<double>(1,3)=-0.020;
	CM[0].at<double>(2,3)=-0.0325;

	// Link 2
	CM[1].at<double>(0,3)=0.040;
	CM[1].at<double>(2,3)=-0.110;

	// Link 3
	CM[2].at<double>(0,3)=-0.032;
	CM[2].at<double>(2,3)=-0.130;

	// Link 4
	CM[3].at<double>(0,3)=0.027;
	CM[3].at<double>(2,3)=-0.078;

	// Link 5
	CM[4].at<double>(0,3)=-0.032;
	CM[4].at<double>(2,3)=-0.210;

	// Link 6
	CM[5].at<double>(2,3)=-0.110;

	temp2 = a*CM[link-1]*x0;

//	ROS_INFO("test 3");
	xyz[0] = temp2.at<double>(0);
	xyz[1] = temp2.at<double>(1);
	xyz[2] = temp2.at<double>(2);
}

void DK_matrix(double angulos[], int joint)
{
	double right_pose[8];
	//update joint_state
	right_pose [0] = -angulos[0];
	right_pose [1] = -angulos[1];
	right_pose [2] = -angulos[2];
	right_pose [3] = -angulos[3];
	right_pose [4] = -angulos[4];
	right_pose [5] = -angulos[5];

	cv::Mat hini = cv::Mat::eye(4,4,CV_64F);

	//Hombro Y
	H[0].at<double>(0,0)=cos(right_pose[0]);
	H[0].at<double>(0,2)=sin(right_pose[0]);
	H[0].at<double>(1,3)=-23.5;
	H[0].at<double>(2,0)=-sin(right_pose[0]);
	H[0].at<double>(2,2)=cos(right_pose[0]);
	H[0].at<double>(2,3)=123.0;

	//Hombro X
	H[1].at<double>(1,1)=cos(right_pose[1]);
	H[1].at<double>(1,2)=-sin(right_pose[1]);
	H[1].at<double>(1,3)=-3.8;
	H[1].at<double>(2,1)=sin(right_pose[1]);
	H[1].at<double>(2,2)=cos(right_pose[1]);
	H[1].at<double>(2,3)=-3.0;

	// Hombro Z
	H[2].at<double>(0,0)=cos(right_pose[2]);
	H[2].at<double>(0,1)=-sin(right_pose[2]);
	H[2].at<double>(0,3)=1.5;
	H[2].at<double>(1,0)=sin(right_pose[2]);
	H[2].at<double>(1,1)=cos(right_pose[2]);
	H[2].at<double>(2,3)=-16.0;

	// Codo Y
	H[3].at<double>(0,0)=cos(right_pose[3]);
	H[3].at<double>(0,2)=sin(right_pose[3]);
	H[3].at<double>(0,3)=-1.8;
	H[3].at<double>(2,0)=-sin(right_pose[3]);
	H[3].at<double>(2,2)=cos(right_pose[3]);
	H[3].at<double>(2,3)=-18.2;

	// Codo Z
	H[4].at<double>(0,0)=cos(right_pose[4]);
	H[4].at<double>(0,1)=-sin(right_pose[4]);
	H[4].at<double>(0,3)=0.4;
	H[4].at<double>(1,0)=sin(right_pose[4]);
	H[4].at<double>(1,1)=cos(right_pose[4]);
	H[4].at<double>(2,3)=-7.8;

	// Muneca
	H[5].at<double>(0,0)=cos(right_pose[5]);
	H[5].at<double>(0,2)=sin(right_pose[5]);
	H[5].at<double>(0,3)=-1.6;
	H[5].at<double>(2,0)=-sin(right_pose[5]);
	H[5].at<double>(2,2)=cos(right_pose[5]);
	H[5].at<double>(2,3)=-27.9;

	switch (joint)
	{
		case 1: {a = H[1]*H[2]*H[3]*H[4]*H[5]; break;}
		case 2: {a = H[2]*H[3]*H[4]*H[5]; break;}
		case 3: {a = H[3]*H[4]*H[5]; break;}
		case 4: {a = H[4]*H[5]; break;}
		case 5: {a = H[5]; break;}
	}

}

void DK_joint(double xyz[], double angulos[], int joint)
{
	double right_pose[8];
	//update joint_state
	right_pose [0] = -angulos[0];
	right_pose [1] = -angulos[1];
	right_pose [2] = -angulos[2];
	right_pose [3] = -angulos[3];
	right_pose [4] = -angulos[4];
	right_pose [5] = -angulos[5];

	//ROS_INFO("Pose iniciada");

	cv::Mat temp;

	//ROS_INFO("Listo para calcular!");

	//Hombro Y
	H[0].at<double>(0,0)=cos(right_pose[0]);
	H[0].at<double>(0,2)=sin(right_pose[0]);
	H[0].at<double>(1,3)=-23.5;
	H[0].at<double>(2,0)=-sin(right_pose[0]);
	H[0].at<double>(2,2)=cos(right_pose[0]);
	H[0].at<double>(2,3)=123.0;

	//Hombro X
	H[1].at<double>(1,1)=cos(right_pose[1]);
	H[1].at<double>(1,2)=-sin(right_pose[1]);
	H[1].at<double>(1,3)=-3.8;
	H[1].at<double>(2,1)=sin(right_pose[1]);
	H[1].at<double>(2,2)=cos(right_pose[1]);
	H[1].at<double>(2,3)=-3.0;


	// Hombro Z
	H[2].at<double>(0,0)=cos(right_pose[2]);
	H[2].at<double>(0,1)=-sin(right_pose[2]);
	H[2].at<double>(0,3)=1.5;
	H[2].at<double>(1,0)=sin(right_pose[2]);
	H[2].at<double>(1,1)=cos(right_pose[2]);
	H[2].at<double>(2,3)=-16.0;

	// Codo Y
	H[3].at<double>(0,0)=cos(right_pose[3]);
	H[3].at<double>(0,2)=sin(right_pose[3]);
	H[3].at<double>(0,3)=-1.8;
	H[3].at<double>(2,0)=-sin(right_pose[3]);
	H[3].at<double>(2,2)=cos(right_pose[3]);
	H[3].at<double>(2,3)=-18.2;

	// Codo Z
	H[4].at<double>(0,0)=cos(right_pose[4]);
	H[4].at<double>(0,1)=-sin(right_pose[4]);
	H[4].at<double>(0,3)=0.4;
	H[4].at<double>(1,0)=sin(right_pose[4]);
	H[4].at<double>(1,1)=cos(right_pose[4]);
	H[4].at<double>(2,3)=-7.8;

	// Muneca
	H[5].at<double>(0,0)=cos(right_pose[5]);
	H[5].at<double>(0,2)=sin(right_pose[5]);
	H[5].at<double>(0,3)=-1.6;
	H[5].at<double>(2,0)=-sin(right_pose[5]);
	H[5].at<double>(2,2)=cos(right_pose[5]);
	H[5].at<double>(2,3)=-27.9;

	// Efector
	H[6].at<double>(2,3)=-16.0;

	//ROS_INFO("Matriz Calculada 1");

	switch (joint)
	{
	case 1: {a = H[0]; break;}
	case 2: {a = H[0]*H[1]; break;}
	case 3: {a = H[0]*H[1]*H[2]; break;}
	case 4: {a = H[0]*H[1]*H[2]*H[3]; break;}
	case 5: {a = H[0]*H[1]*H[2]*H[3]*H[4]; break;}
	case 6: {a = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]; break;}
	default:{a = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6]; break;}

	}

	//ROS_INFO("Matriz Calculada 2");
//	homogenea = a;
	temp = a*x0;
	xyz[0] = temp.at<double>(0);
	xyz[1] = temp.at<double>(1);
	xyz[2] = temp.at<double>(2);

}

void CargaMotoresCalc(double torque[],double th[], double M[])
{
	//Parametros
	// Masa en Kilogramos
	// Distancia en centimetros

//	double M[6],L[6];
//
//	M[0] = 0.316; // este es el que está pegado a Bender
//	M[1] = 0.216;
//	M[2] = 0.282;
//	M[3] = 0.256;
//	M[4] = 0.243;
//	M[5] = 0.222;
//	M[6] = 0.430;
//	// sum M4+M5+M6 = 0.895
//	L[5] = 0.029663; // numero no tan mágico
//	L[3] = 0.013806; //	numero super mágico

//	torque[3] = std::cos(th[3]-pi/2)*(M[4]+M[5]+M[6])*L[3]*g;
//	torque[5] = std::cos(th[5]+th[3]-pi/2)*M[6]*L[5]*g;

	double MaxTorque106 = 2500;
	double MaxTorque64 = 570;
	double MaxTorque28 = 410;

	double grav[3];
	double masa = 0;
	double tx,ty,tz;
	double erre[3];


	CenterGravity(erre,th,M,1);
	GetGravity(grav, 1, th);
	masa = MAcumulada(M,1);
	tx = masa * (erre[1]*grav[2] - erre[2]*grav[1]);
	ty = masa * (erre[0]*grav[2] - erre[2]*grav[0]);
	tz = masa * (erre[0]*grav[1] - erre[1]*grav[0]);
//	printf("MASA 4 = %f\n", masa);
//	printf("GG = %f | %f | %f |\n",grav[0], grav[1], grav[2]);
//	printf("RR = %f | %f | %f |\n",erre[0], erre[1], erre[2]);
//	printf("Tx = %f | Ty = %f | Tz = %f\n", tx,ty,tz);
	torque[0] = ty/MaxTorque106;


	CenterGravity(erre,th,M,2);
	GetGravity(grav, 2, th);
	masa = MAcumulada(M,2);
//	printf("MASA 4 = %f\n", masa);
//	printf("GG = %f | %f | %f |\n",grav[0], grav[1], grav[2]);
//	printf("RR = %f | %f | %f |\n",erre[0], erre[1], erre[2]);
//	printf("Tx = %f | Ty = %f | Tz = %f\n", tx,ty,tz);
	tx = masa * (erre[1]*grav[2] - erre[2]*grav[1]);
	ty = masa * (erre[0]*grav[2] - erre[2]*grav[0]);
	tz = masa * (erre[0]*grav[1] - erre[1]*grav[0]);

	torque[1] = -tx/MaxTorque106;


	CenterGravity(erre,th,M,3);
	GetGravity(grav, 3, th);
	masa = MAcumulada(M,3);
	tx = masa * (erre[1]*grav[2] - erre[2]*grav[1]);
	ty = masa * (erre[0]*grav[2] - erre[2]*grav[0]);
	tz = masa * (erre[0]*grav[1] - erre[1]*grav[0]);

	torque[2] = -tz/MaxTorque64;


	CenterGravity(erre,th,M,4);
	GetGravity(grav, 4, th);
	masa = MAcumulada(M,4);
	tx = masa * (erre[1]*grav[2] - erre[2]*grav[1]);
	ty = masa * (erre[0]*grav[2] - erre[2]*grav[0]);
	tz = masa * (erre[0]*grav[1] - erre[1]*grav[0]);
//	printf("MASA 4 = %f\n", masa);
//	printf("GG = %f | %f | %f |\n",grav[0], grav[1], grav[2]);
//	printf("RR = %f | %f | %f |\n",erre[0], erre[1], erre[2]);
//	printf("Tx = %f | Ty = %f | Tz = %f\n", tx,ty,tz);
	torque[3] = ty/MaxTorque106;


	CenterGravity(erre,th,M,5);
	GetGravity(grav, 5, th);
	masa = MAcumulada(M,5);
	tx = masa * (erre[1]*grav[2] - erre[2]*grav[1]);
	ty = masa * (erre[0]*grav[2] - erre[2]*grav[0]);
	tz = masa * (erre[0]*grav[1] - erre[1]*grav[0]);
//	printf("MASA 5 = %f\n", masa);
//	printf("GG = %f | %f | %f |\n",grav[0], grav[1], grav[2]);
//	printf("RR = %f | %f | %f |\n",erre[0], erre[1], erre[2]);
//	printf("Tx = %f | Ty = %f | Tz = %f\n", tx,ty,tz);
	torque[4] = tz/MaxTorque64;


	CenterGravity(erre,th,M,6);
	GetGravity(grav, 6, th);
	masa = MAcumulada(M,6);
	tx = masa * (erre[1]*grav[2] - erre[2]*grav[1]);
	ty = masa * (erre[0]*grav[2] - erre[2]*grav[0]);
	tz = masa * (erre[0]*grav[1] - erre[1]*grav[0]);
//	printf("MASA 6 = %f\n", masa);
//	printf("GG = %f | %f | %f |\n",grav[0], grav[1], grav[2]);
//	printf("RR = %f | %f | %f |\n",erre[0], erre[1], erre[2]);
//	printf("Tx = %f | Ty = %f | Tz = %f\n", tx,ty,tz);
	torque[5] = ty/MaxTorque28;


}

void SoltarJoint(double torque[], double carga[])
{
//	tor.request.torque_enable = true;

	double eps1, eps2, eps3, eps4, eps5, eps6;

	eps1 = 0.18;
	eps2 = 0.1;
	eps3 = 0.08;
	eps4 = 0.07;
	eps5 = 0.08;
	eps6 = 0.08; //eps < 0.1

	right_limite[0] = 0.05;
	right_limite[1] = 1;
	right_limite[2] = 1;
	right_limite[3] = 0.03;
	right_limite[4] = 1;
	right_limite[5] = 0.05;
	right_limite[6] = 1;
	right_limite[7] = 1;

	joint_select[0] = true;
//	joint_select[1] = true;
//	joint_select[2] = true;
	joint_select[3] = true;
//	joint_select[4] = true;
	joint_select[5] = true;

	// HOMBRO Y
	if (carga[0] < (torque[0]-(eps1))) // hacia abajo
	{
		LimitarTorqueMax(right_limite,1);
		SetPosicion(th, joint_select);
	}
	else if(carga[0] > torque[0]+eps1 ) // hacia arriba
	{
		LimitarTorqueMax(right_limite,1);
		SetPosicion(th, joint_select);
	}
	else
	{
		right_limite[0] = 0.5;
		LimitarTorqueMax(right_limite,1);
	}


	// HOMBRO x
//	if (carga[1] < (torque[1]-(eps2))) // hacia abajo
//	{
//		LimitarTorqueMax(right_limite,2);
//		SetPosicion(th, joint_select);
//	}
//	else if(carga[1] > torque[1]+eps2 ) // hacia arriba
//	{
//		LimitarTorqueMax(right_limite,2);
//		SetPosicion(th, joint_select);
//	}
//	else
//	{
//		right_limite[1] = 0.3;
//		LimitarTorqueMax(right_limite,2);
//	}

	// HOMBRO z
//	if (carga[2] < (torque[2]-(eps3))) // hacia abajo
//	{
//		LimitarTorqueMax(right_limite,3);
//		SetPosicion(th, joint_select);
//	}
//	else if(carga[2] > torque[2]+eps3 ) // hacia arriba
//	{
//		LimitarTorqueMax(right_limite,3);
//		SetPosicion(th, joint_select);
//	}
//	else
//	{
//		right_limite[1] = 0.3;
//		LimitarTorqueMax(right_limite,3);
//	}

//	 CODO Y
	if (carga[3] < torque[3] - (eps4)) // hacia abajo
	{
		LimitarTorqueMax(right_limite,4);
		SetPosicion(th, joint_select);
	}
	else if(carga[3] > torque[3] + eps4 ) // hacia arriba
	{
		LimitarTorqueMax(right_limite,4);
		SetPosicion(th, joint_select);
	}
	else
	{
		right_limite[3] = 0.3;
		LimitarTorqueMax(right_limite,4);
	}

	// CODO Z
//	if (torque[4] > (carga[4]+(eps5))) // hacia abajo
//	{
//		LimitarTorqueMax(right_limite,5);
//		SetPosicion(th, joint_select);
//	}
//	else if(-torque[4] < carga[4]+eps5 ) // hacia arriba
//	{
//		LimitarTorqueMax(right_limite,5);
//		SetPosicion(th, joint_select);
//	}
//	else
//	{
//		right_limite[4] = 0.1;
//		LimitarTorqueMax(right_limite,5);
//	}

	// MUNECA Y
	if (torque[5] > (carga[5]+(eps6))) // hacia abajo
	{
		LimitarTorqueMax(right_limite,6);
		SetPosicion(th, joint_select);
	}
	if(-torque[5] < carga[5]+eps6 ) // hacia arriba
	{
		LimitarTorqueMax(right_limite,6);
		SetPosicion(th, joint_select);
	}
	else
	{
		right_limite[5] = 0.25;
		LimitarTorqueMax(right_limite,6);
	}

}

bool LimitarTorqueMax(double limiteTorque[], int joint)
{
	bool res;

	switch (joint) // joint entre 1 y 6
	{
	case 1: tl.request.motor_name = "right_hombro_1"; break;
	case 2: tl.request.motor_name = "right_hombro_2"; break;
	case 3: tl.request.motor_name = "right_hombro_3"; break;
	case 4: tl.request.motor_name = "right_codo_1"; break;
	case 5: tl.request.motor_name = "right_codo_2"; break;
	case 6: tl.request.motor_name = "right_muneca"; break;
	}

	tl.request.torque_limit = limiteTorque[joint-1];

	if(torque_limit.call(tl))
		return res = true;
	else
	{
		return res = false;
		ROS_INFO("Failed to call service: right_arm_joints/torque_limit");
	}
	return res;
}











bool SetPosicion(double th[], bool select[])
{
	pose.positions.resize(8);
	pose.select.resize(8);
	pose.speed.resize(8);

	for (int i=0;i<8;i++)
	{
		pose.positions[i] = th[i];
		pose.select[i] = select[i];
		pose.speed[i] = 0.1;
	}

	call_pose.publish(pose);

	return true;

}

#include "bender_planning_old/hMatrix.h"

// como convencion:
//		|	0	|	1	|	2	|	3	|
//		|	4	|	5	|	6	|	7	|
//		|	8	|	9	|	10	|	11	|
//		|	12	|	13	|	14	|	15	|


void hMatrix::hTransform( double * matrix , double x, double y, double z, double * resp)
{
	resp[0]=matrix[0]*x+matrix[1]*y+matrix[2]*z+matrix[3];
	resp[1]=matrix[4]*x+matrix[5]*y+matrix[6]*z+matrix[7];
	resp[2]=matrix[8]*x+matrix[9]*y+matrix[10]*z+matrix[11];
	resp[3]=matrix[12]*x+matrix[13]*y+matrix[14]*z+matrix[15];
}

void hMatrix::hMatrixMultip( double * M1, double * M2)		//M2 es a su vez la respuesta
{
	double temp[16];
	for(int i=0; i<16; ++i){
		temp[i]=M2[i];
		M2[i]=0;
	}
	for( int j=0;j<4;j++)
		for( int i=0;i<4;i++)
			for( int k=0;k<4;k++)
				M2[i+4*j]+= M1[k+4*j]*temp[i+4*k];
}

double * hMatrix::hIdentity()
{
	double * resp = new double[16];
	for(int i=0;i<16;i++)
		if(i==0||i==5||i==10||i==15)
			resp[i]=1;
		else
			resp[i]=0;
	return resp;
}

void hMatrix::hIdentity(double * resp)
{
	for(int i=0;i<16;i++)
		if(i==0||i==5||i==10||i==15)
			resp[i]=1;
		else
			resp[i]=0;
}

void hMatrix::DH_Matrix(double alpha, double a, double d, double theta, double*T){
	double M[16]={	cos(theta),				-sin(theta),			0,				a,
					sin(theta)*cos(alpha),	cos(theta)*cos(alpha),	-sin(alpha),	-sin(alpha)*d,
					sin(theta)*sin(alpha),	cos(theta)*sin(alpha),	cos(alpha),		cos(alpha)*d,
					0,						0,						0,				1};

	for(int i=0;i<16;++i)
		T[i]=M[i];
}
void hMatrix::hRotateX(double theta, double * M1)		//Se agregan premultiplicando!
{
	double temp[16]={	1			,0			,0			,0,
						0			,cos(theta)	,-sin(theta),0,
						0			,sin(theta)	,cos(theta)	,0,
						0			,0			,0			,1};
	hMatrixMultip(temp,M1);
	
}
void hMatrix::hRotateY(double theta, double * M1)		//Se agregan premultiplicando!
{
	double temp[16]={	cos(theta)	,0			,-sin(theta)	,0,
						0			,1			,0			,0,
						sin(theta)	,0			,cos(theta)	,0,
						0			,0			,0			,1};
	hMatrixMultip(temp,M1);
	
}
void hMatrix::hRotateZ(double theta, double * M1)		//Se agregan premultiplicando!
{
	double temp[16]={	cos(theta)	,-sin(theta),0			,0,
						sin(theta)	,cos(theta)	,0			,0,
						0			,0			,1			,0,
						0			,0			,0			,1};
	hMatrixMultip( temp,M1);
	
}
void hMatrix::hTranslation(double x, double y, double z, double * M1)
{
	double temp[16]={	1			,0			,0			,x,
						0			,1			,0			,y,
						0			,0			,1			,z,
						0			,0			,0			,1};
	hMatrixMultip( temp,M1);
	
}
void hMatrix::hScale(double scale , double * M1)
{
	double temp[16]={	scale		,0			,0			,0,
						0			,scale		,0			,0,
						0			,0			,scale		,0,
						0			,0			,0			,1};
	hMatrixMultip( temp,M1);
	
}


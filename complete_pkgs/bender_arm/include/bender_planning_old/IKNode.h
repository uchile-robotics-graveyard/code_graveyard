#ifndef _PR_IKNODE_
#define _PR_IKNODE_
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#ifndef _PR_OMNODE_
#include "bender_planning_old/OMNode.h"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>

#include "bender_planning_old/hMatrix.h"
#include "bender_planning_old/QueryTimer.h"
#include "bender_planning_old/distbrazo.h"

#define DoF 5


using namespace hMatrix;

class IKNode
{

public:

	IKNode * father;				//Puntero al Padre
	IKNode * * sons;				//Puntero a los Hijos
	OMNode * Map;					//Mapa de Obstaculos
	double state[DoF];				//Configuracion del Nodo
	double distance;				//Distancia al objetivo
	double param;					//Dist. Angular al inicio
	bool isLeaf;					//True: Es del Borde

	IKNode(double * x,double p, double * pFinal, OMNode * M);
	~IKNode();

	void setState(double * x);		//Setea la configuracion del Nodo
	void setFather(IKNode * x);		//Setea el puntero al padre
	IKNode * GetMin();				//Retorna el puntero al minimo suyo o de sus hijos
	void Expand(IKNode * tree);		//Expande el Nodo.

	bool operator == (IKNode * x);	//Comprueba igualdad con otro nodo.
	bool operator == (double * x);	//Comprueba igualdad con una config.
	bool compare(IKNode * x);		//Compara con otro nodo

private:
	
	double * destination;			//Pose Objetivo 3D
	bool isSet;						//Confirma que este completamente configurado el nodo
	int nSons;						//Numero de Hijos del Nodo
	int fatherNumber;				//No usado
	void calcDist();				//Calcula la distancia al objetivo
	bool IsHere(double * st);		//Revisa si una configuracion dada ya existe en esta rama


};

class IKNodeException
{
};

double * AppStar(double * aPose, double * fPosition, OMNode * Map,int brazo, bool * success, bool planGrip);
double anguloRotacion(double x1, double y1);
#endif

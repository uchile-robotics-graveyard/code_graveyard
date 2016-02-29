#include "bender_planning_old/IKNode.h"

#define _PR_UMBRAL_A_S_ 0.3
int Brazo;
double CorrAng=-6;



IKNode::IKNode(double * x, double p, double * pFinal, OMNode * M)
{
	Map=M;
	param=p;
	destination = pFinal;
	for(int i=0;i<DoF;i++)
		state[i]=x[i];
	calcDist();
	isLeaf=true;
}
IKNode::~IKNode()
{
	if(!isLeaf && nSons > 0)
//		sons.clear();
		delete[] sons;
}

void IKNode::setFather(IKNode * x)
{
	father=x;
}

IKNode * IKNode::GetMin()
{
	if(isLeaf)
		return this;
	if (nSons == 0)
	{
		printf("AppStar: Nodo interno con 0 hijos\n");
		return NULL;
	}
	QueryTimer Reloj=QueryTimer();
	Reloj.Start();
	IKNode * resp=sons[0];
	for(int i=0;i<nSons;i++)
	{

		IKNode *sonsGetMin = sons[i]->GetMin();
		if (sonsGetMin == NULL)
			return NULL;
		if( !(resp->compare(sonsGetMin)))
			resp=sonsGetMin;
/*
		IKNode *sonsGetMin = sons[i]->GetMin();
		if( !(resp->compare(sons[i]->GetMin())))
			resp=sons[i]->GetMin();
*/
			Reloj.Stop();
		if(Reloj.Duracion()>=5){
			printf("Trayectoria infactible\n");
			return NULL;
			}
	}
	return resp;
}

void IKNode::calcDist()
{
	double dist[4];
	double M[16];
	hIdentity(M);
	double M2[16];
	hIdentity(M2);
	double M3[16];
	hIdentity(M3);
	
/*	hRotateX(state[2],M);
	hRotateZ(state[1],M);
	hRotateY(state[0],M);
	hTransform(M,0,0,0,dist);
	for(int i=1;i<6;i++)
	{
		hIdentity(M);
		hTranslation(0,_PR_L1*i/5.0,0,M);
		hRotateX(state[2],M);
		hRotateZ(state[1],M);
		hRotateY(state[0],M);
		hTransform(M,0,0,0,dist);
		if(!Map->isFree(dist[0],dist[1],dist[2]))
		{
			distance=9999;
			return;
		}
	}
	for(int i=1;i<11;i++)
	{	
		hIdentity(M);
		hTranslation(0,_PR_L2*i/10.0,0,M);
		hRotateX(state[4],M);
		hRotateY(state[3],M);
		hTranslation(0,_PR_L1,0,M);
		hRotateX(state[2],M);
		hRotateZ(state[1],M);
		hRotateY(state[0],M);
		hTransform(M,0,0,0,dist);
		if(!Map->isFree(dist[0],dist[1],dist[2]))
		{
			distance=9999;
			return;
		}
	}*/
	int signo= Brazo==1? 1:-1;
	if(	signo*state[0]>37.8 || signo*state[0]<-90 ||
		state[1]<-180 || state[1]>15.0 ||
		state[2]>88.0 || state[2]<-88.0 ||
		abs(state[3]>150.0) || (state[1]>=5 && state[0]>10)){
		distance=9999;
		return;
	}


	//hIdentity(M);
	//hRotateY(-state[5]*M_PI/180.0,M);
	/*hTranslation(0,0,-15.5,M);
	hRotateZ(state[4]*M_PI/180.0,M);
	hTranslation(0,0,-32,M);
	hRotateY(state[3]*M_PI/180.0,M);
	hTranslation(0,0,-18,M);
	hRotateZ(state[2]*M_PI/180.0,M);
	hTranslation(0,0,-18,M);
	hRotateY(state[1]*M_PI/180.0,M);
	hTranslation(0,0,0,M);
	hRotateZ(state[0]*M_PI/180.0,M);*/
	//hRotateY(-6*M_PI/180.0,M);
	//hTranslation(-DistHombroBaseX,-DistHombroBaseY,-DistHombroBaseZ,M);
	//calcular distancia segun matrices DH
	double ROT[16]; hIdentity(ROT);
	//hRotateX(2*M_PI/180,ROT); 
	hRotateY(CorrAng*M_PI/180.0,ROT);
	double T01[16]; DH_Matrix(0,0,0,state[0]*M_PI/180.0,T01);
	double T12[16]; DH_Matrix(-90*M_PI/180.0,2,-4,state[1]*M_PI/180.0,T12);
	double T23[16]; DH_Matrix(90*M_PI/180.0,0,-18,state[2]*M_PI/180.0,T23);
	//T34 no cumple las condiciones para calcularse con matriz DH,
	//por lo tanto se utiliza el producto de traslaci�n en y=(-18), rotaci�n en z=(-th), rotaci�n en x=(pi/2)
	double th=state[3]*M_PI/180.0;
	double T34[16]={	cos(th), -sin(th), 0, 0 ,
						0,		  0		 , 1, 0 , 
						-sin(th),-cos(th), 0,-19,
						0,		  0		 , 0, 1 };

	double T45[16]; DH_Matrix(90*M_PI/180.0,1,-32,state[4]*M_PI/180.0,T45);
	double T05[16]; hIdentity(T05);
	//T05=ROT*T01*T12*T23*T34*T45
	hMatrixMultip(T34,T45);
	hMatrixMultip(T23,T45);
	hMatrixMultip(T12,T45);
	hMatrixMultip(T01,T45);
	hMatrixMultip(ROT,T45);
	hMatrixMultip(T45,T05);
	if(Brazo==1) hTranslation(-DistHombroBaseX,-DistHombroBaseY,-DistHombroBaseZ,T05);
	else hTranslation(-DistHombroBaseX,DistHombroBaseY,-DistHombroBaseZ,T05);
	
	//Punta del grip en coord (0,0,-18) respecto de la mu�eca
	hTransform(T05,0,0,-15,dist); 

	//distancia al objetivo
	distance=sqrt(	 (dist[0]-destination[0])*(dist[0]-destination[0])
					+(dist[1]-destination[1])*(dist[1]-destination[1])
					+(dist[2]-destination[2])*(dist[2]-destination[2]));


}

void IKNode::Expand(IKNode * tree)			//ESTA FUNCION ES INDEPENDIENTE DEL DoF!!!
{
	double t[5];// = new double[5];
	int count=0;
	for(int i=0;i<5;i++)
		t[i]=state[i];
	bool condition[243];		//OJO
	for(int i=0;i<3;i++)
	{
		t[0]=state[0]+0.5*(i-1);
		for(int j=0;j<3;j++)		
		{
			t[1]=state[1]+0.5*(j-1);
			for(int k=0;k<3;k++)
			{
				t[2]=state[2]+0.5*(k-1);
				for(int l=0;l<3;l++)
				{
					t[3]=state[3]+1*(l-1); 
					for(int m=1;m<2;m++)//grado de libertad muerto.
					{
						t[4]=state[4]+1*(m-1);
						if(*father==t)
							condition[m+3*l+9*k+27*j+81*i]=false;
						else
						{
							condition[m+3*l+9*k+27*j+81*i]= !tree->IsHere(t);
							if(condition[m+3*l+9*k+27*j+81*i]){
								count++;
							}
							else{
								//printf("No se cumple condicion %d %d %d %d %d !!! \n", i, j, k, l ,m);
							}
						}
					}
				}
			}
		}
	}
			
	nSons=count;
	sons=new IKNode*[nSons];
	//sons.resize(nSons);
	count = 0;
	for(int i=0;i<3;i++)
	{
		t[0]=state[0]+0.5*(i-1);	
		for(int j=0;j<3;j++)
		{
			t[1]=state[1]+0.5*(j-1);
			for(int k=0;k<3;k++)
			{
				t[2]=state[2]+0.5*(k-1);
				for(int l=0;l<3;l++)
				{
					t[3]=state[3]+1*(l-1);
					for(int m=1;m<2;m++)//grado de libertad muerto.
					{
						t[4]=state[4]+1*(m-1);
						if(condition[m+3*l+9*k+27*j+81*i])
						{
							double pa=0;
							for(int n=0;n<5;n++)
								pa=pa+pow(state[n]-t[n],2);
							pa=sqrt(pa);
							sons[count]=new IKNode(t,param+pa,destination,Map);
							sons[count]->setFather(this);
							count++;
							
						}
					}
				}
			}
		}
	}
	
	isLeaf=false;

}

bool IKNode::operator ==(IKNode *x)
{
	for(int i=0;i<DoF;i++)
	{
		if(state[i]!=x->state[i])
			return false;
	}
	return true;
}

bool IKNode::operator ==(double *x)
{
	for(int i=0;i<DoF;i++)
	{
		if(state[i]!=x[i])
			return false;
	}
	return true;
}


bool IKNode::compare(IKNode *y)
{
	// if(distance+param < y->distance+y->param)*/
	if(distance < y->distance )
		return true;
	else
		return false;
}

bool IKNode::IsHere(double * st)
{
	if(isLeaf)
		return *this==st;
	for(int i=0;i<nSons;i++)
		if(sons[i]->IsHere(st))
			return true;
	return false;
}


double * AppStar(double * aPose, double * fPosition,OMNode * Map, int brazo)//recibe poses en radianes
{
	Brazo=brazo;
	//CorrAng=anguloRotacion(fPosition[0],fPosition[1]);
	double posinit[DoF];
	for(int i=0;i<DoF;i++)
		posinit[i]=aPose[i]*180/M_PI;			//Se guarda un buffer de la pose inicial en grados

	IKNode * tree = new IKNode(posinit,0,fPosition,Map);	//Se inicializa el arbol
	if(tree->distance< _PR_UMBRAL_A_S_){			//Se revisa que la solucion no sea trivial
		double *rtriv=new double[6];
		rtriv[0]=1;
		for(int i=1;i<6;++i)
			rtriv[i]=aPose[i-1];
		return rtriv;
	}
	tree->father=tree;							//Se corre el algoritmo sobre la raiz.
	IKNode * min = tree;
	min->Expand(tree);
	min=tree->GetMin();	
	while(min->distance > _PR_UMBRAL_A_S_)		//Se itera hasta que la distancia sea menor que un umbral
	{
		min->Expand(tree);						//Se expande el minimo
		IKNode	* getMin = min->GetMin();
		if (getMin == NULL)
			break;
		if(getMin->compare(min))	//Si hay un valor menor que el nodo expandido
			min=getMin;//min->GetMin();						//en sus hijos, entonces es el nuevo minimo
		else										//Si no, se busca el minimo en todo el arbol
			min=tree->GetMin();
	}
	printf("%f \n",min->distance);
	IKNode *aux=min;
	int i=0;
	while(!(*aux==tree))						//Se calcula la cantidad de poses de la trayectoria
	{
		aux=aux->father;
		i++;
	}
	double * r = new double[1+DoF*i];			//El primer valor de la respuesta es la cantidad de 
	r[0]=i;										//poses de la trayectoria
	aux=min;
	while(!(*aux==tree))						//Se almacenan las poses ordenadas en la respuesta
	{
		i--;
		for(int j=0;j<DoF;j++)
			r[1+DoF*i+j]=aux->state[j]*M_PI/180.0;
		aux=aux->father;
	}
	delete tree;
	return r;
}

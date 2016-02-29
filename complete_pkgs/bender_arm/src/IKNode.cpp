#include "bender_planning_old/IKNode.h"
#include <signal.h>

#define _PR_UMBRAL_A_S_ 0.3
int Brazo;
bool PlanGrip;


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

	int signo= Brazo==1? 1:-1;
	
	if(	(signo*state[1]>1.0 && abs(state[0])+2/3*abs(state[3])<16.0) ||
 		(abs(state[0])<28.0 && signo*state[1]>13.7) ||
		(signo*state[0]<-49.2 && signo*state[1]>25) ||
		(signo*state[0]<-28 && signo*state[1]>0.0122*state[0]*state[0]+0.3356*signo*state[0]+10) || //ajuste polinomial de 2do orden para st0 vs st1
		signo*state[1]<-100.0 ||
		abs(state[2])>88.0 ||
		abs(state[3])>113.0 ||
		signo*state[0]>60.0 ||
		signo*state[1]>27.15
	  )
	{
		//printf("Pose con distancia infinita: %f %f %f %f \n",state[0],state[1],state[2],state[3]);
		distance=9999;
		return;
	}
	

	double ca1=cos(-signo*state[0]*M_PI/180.0);
	double ca2=cos(state[1]*M_PI/180.0);
	double ca3=cos(-signo*state[2]*M_PI/180.0);
	double ca4=cos(-state[3]*M_PI/180.0);
	double sa1=sin(-signo*state[0]*M_PI/180.0);
	double sa2=sin(state[1]*M_PI/180.0);
	double sa3=sin(-signo*state[2]*M_PI/180.0);
	double sa4=sin(-state[3]*M_PI/180.0);
	double TF[16];
	TF[0]=ca1*ca3*ca4+sa1*(ca2*sa4-sa2*sa3*ca4);
	TF[1]=ca1*sa3+sa1*sa2*ca3;
	TF[2]=ca1*ca3*sa4+sa1*(-ca2*ca4-sa2*sa3*sa4);
	TF[3]=ca1*(ca3*(-1.5*ca4-35.65*(sa4+0.050491))+1.5)+sa1*(ca2*(35.65*ca4-1.5*sa4+34.2)+sa2*sa3*(1.5*ca4+35.65*(sa4+0.050491))+3);

	TF[4]=-ca2*sa3*ca4-sa2*sa4;
	TF[5]=ca2*ca3;
	TF[6]=sa2*ca4-ca2*sa3*sa4;
	TF[7]=ca2*sa3*(1.5*ca4+35.65*(sa4+0.050491))+sa2*(-35.65*ca4+1.5*sa4-34.2)+signo*3.8;

	TF[8]=ca1*(sa2*sa3*ca4-ca2*sa4)+sa1*ca3*ca4;
	TF[9]=sa1*sa3-ca1*sa2*ca3;
	TF[10]=ca1*(ca2*ca4+sa2*sa3*sa4)+sa1*ca3*sa4;
	TF[11]=ca1*(ca2*(-35.65*ca4+1.5*sa4-34.2)+sa2*sa3*(-1.5*ca4-35.65*(sa4+0.050491))-3)+sa1*(ca3*(-1.5*ca4-35.65*(sa4+0.050491))+1.5);

	TF[12]=0;
	TF[13]=0;
	TF[14]=0;
	TF[15]=1;

	if(PlanGrip){
		double anguloYmuneca=asin(ca1*(ca2*ca4+sa2*sa3*sa4)+sa1*ca3*sa4);
		double anguloZmuneca=asin(sa1*sa3-ca1*sa2*ca3);
		double rotYmuneca[16]={	cos(anguloYmuneca)	,0			,-sin(anguloYmuneca)	,0,
								0			,1			,0			,0,
								sin(anguloYmuneca)	,0			,cos(anguloYmuneca)	,0,
								0			,0			,0			,1};
		double rotZmuneca[16]={	cos(anguloZmuneca)	,-sin(anguloZmuneca),0			,0,
								sin(anguloZmuneca)	,cos(anguloZmuneca)	,0			,0,
								0			,0			,1			,0,
								0			,0			,0			,1};
		hMatrixMultip(TF,rotZmuneca);
		hMatrixMultip(rotZmuneca,rotYmuneca);
		hIdentity(TF);
		hMatrixMultip(rotYmuneca,TF);
	}

	hRotateY(CorrAng*M_PI/180.0,TF);
	
	hTranslation(-DistHombroBaseX,-signo*DistHombroBaseY,-DistHombroBaseZ,TF);

	//Punta del grip en coord (0,0,-17.5) respecto de la muneca
	if(PlanGrip)

		hTransform(TF,0,0,-16,dist);
	else
		hTransform(TF,0,0,0,dist);

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
							if(condition[m+3*l+9*k+27*j+81*i])
								count++;

						}
					}
				}
			}
		}
	}
	printf("Count: %d \n",count);
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


double * AppStar(double * aPose, double * fPosition,OMNode * Map, int brazo, bool * success, bool planGrip)//recibe poses en radianes
{
	PlanGrip=planGrip;
	Brazo=brazo;
	*success=true;
	//CorrAng=anguloRotacion(fPosition[0],fPosition[1]);
	double posinit[DoF];
	for(int i=0;i<DoF;i++)
		posinit[i]=aPose[i]*180.0/M_PI;			//Se guarda un buffer de la pose inicial en grados

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
		if (getMin == NULL){
			printf("Pose objetivo no alcanzada\n");
			printf("Planificacion realizada hasta la pose: %f %f %f %f \n",min->state[0],min->state[1],min->state[2],min->state[3]);
			*success=false;
			break;
		}
		if(getMin->compare(min))	//Si hay un valor menor que el nodo expandido
			min=getMin;//min->GetMin();						//en sus hijos, entonces es el nuevo minimo
		else										//Si no, se busca el minimo en todo el arbol
			min=tree->GetMin();
	}
	printf("Distancia al objetivo: %f \n",min->distance);
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


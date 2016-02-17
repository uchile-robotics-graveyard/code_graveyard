#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "stdafx.h"
#include "serialport.h"
#include "QueryTimer.h"
#include "OMNode.h"
#include "IKNode.h"
#include "RRT4D.h"

#include "spline.h"

#define BUFSIZE 50
#define PI 3.14159265
#define TOL 0.0005
#define rad2dec 195.18762221 // 511*180°/(pi*150°)
#define dec2rad 0.0051233 //1 dec_motor = 0.005 rad

#define COM_PORT (8) // DLF 26 junio 2011 Defnir el puerto serial COM respectivo para el PC

#define HOMBROPAN1 (101)
#define HOMBROPAN2 (111)
#define HOMBROPITCH1 (102)
#define HOMBROPITCH2 (112)
#define CODOROLL (103)
#define CODOPITCH (104)
#define MUNECA (105)
#define GRIPDER (106) //24JUN2011
#define GRIPIZQ (107) //25JUN2011
#define HOMBROPAN12 (201)
#define HOMBROPAN22 (211)
#define HOMBROPITCH12 (202)
#define HOMBROPITCH22 (212)
#define CODOROLL2 (203)
#define CODOPITCH2 (204)
#define MUNECA2 (205)
#define GRIP2 (206)
// << DLF 1jun2011
#define HOMBROPAN3 (90) 
#define HOMBROPITCH3 (91) 
#define CODO3 (92)
#define MUNECA3 (93)
#define RGRIP3 (94)
#define LGRIP3 (95)
// DLF 1jun2011 >>
#define OKvar (1)
#define ERRORvar (0)
// DLF 8jun2011 >>

#define OFFSET101 (127) //posicion del cero actual en pasos de servo (0-1023) segun ID
#define OFFSET102 (122)
#define OFFSET103 (430)
#define OFFSET104 (281)
#define OFFSET105 (466)
#define OFFSET106 (729)//antes 430
#define OFFSET107 (411)
#define OFFSET111 (887)
#define OFFSET112 (894)
#define OFFSET201 (0) 
#define OFFSET202 (0)
#define OFFSET203 (0)
#define OFFSET204 (0)
#define OFFSET205 (0)
#define OFFSET206 (0)
#define OFFSET207 (0)
#define OFFSET211 (0)
#define OFFSET212 (0)
// << DLF 1jun2011          from      ;  up to
#define OFFSET90 (565) // 400 => -x°  ;  720 => x° 
#define OFFSET91 (116) // 100 => -5°  ;  423 => 90°
#define OFFSET92 (453) // 146 => -90°  ;  760 => 90°
#define OFFSET93 (340) // 33 => -90°  ;  647 => 90°
#define OFFSET94 (780) // 774 => 0°  ;  359 ~=> 90°
#define OFFSET95 (230) // 224 => 0°  ;  673 ~=> 90°
#define PRESENTLOADL (0x28) // 
#define PRESENTLOADH (0x29) //
#define MOVING (0x2E)
#define VELOCIDAD1_GRIPPER (500)
// DLF 14jun2011 >>

#define IDBROADCAST (0xFE)
#define TORQUEENABLE (0x18)
#define COMPLIANCEMARGINCW (0x1A)
#define COMPLIANCEMARGINCCW (0x1B)
#define COMPLIANCESLOPECW (0x1C)
#define COMPLIANCESLOPECCW (0x1D)
#define GOALPOSITIONL (0x1E)
#define GOALPOSITIONH (0x1F)
#define MOVINGSPEEDL (0x20)
#define MOVINGSPEEDH (0x21)
#define PRESENTPOSITIONL (0x24)
#define PRESENTPOSITIONH (0x25)
#define PUNCHLOW (0x30)
#define PUNCHHIGH (0x31)


#define READ_DATA (0x02)
#define WRITE_DATA (0x03)
#define WRITE_REG (0x04)  
#define ACTION (0x05)

#define Lz 115.5
#define Lx2 2
#define Ly 26
#define Lx 2
#define L1 13.5
#define L11 4.5
#define L2 23.2
#define L3 31.7
#define L4 6

#define MAX_TIEMPO_CINEMATICA_INVERSA (2)

#define t1 angulos[0]
#define t2 angulos[1]
#define t3 angulos[2]
#define t4 angulos[3]

int generarCoef(double qi[],double qo[],double tiempo,double coef[][4]);
double Angulo(int Joint, double tiempo, double coef[][4]);
double Velocidad(int Joint, double tiempo, double coef[][4]);
int rad2servo(double rad, int i,int brazo);
int rad2servo3(double rad, int i,int brazo);  //DLF 1JUN2011
double servo2rad3(int servo,int i,int brazo); //DLF 1JUN2011
int rad2servohuijse(double rad, int i); 
int rad2servo2(double rad,int i);
double servo2rad(int servo, int i,int brazo);
double servo2radhuijse(int servo, int i);
double servo2rad2(int servo, int i);
int rads2servo(double rads);
void cinematicaInversa(double q[],double po[],double qi[]);
void cinematicaInversa2(double q[],double po[],double qi[]);
unsigned char* generarBuffer(int id, int inst, int nParams,char *params);
	
void calcularJacobiano(double jacob[][4], double angulos[]);
void calcularJacobiano2(double jacob[][4], double angulos[]);
void cinematicaDirecta(double pose[],double angulos[]);
void cinematicaDirecta2(double pose[],double angulos[]);
void diferencia(int m,double po[], double pa[], double diferencia[]);
void suma(int m,double qi[], double delta[], double qo[]);
void trasponer(int m, int n, double a[][4], double b[][3]);
void matxvector(int m, int n, double matriz[][3], double vector[] ,double resultado[] );
void matxvector2(int m, int n, double matriz[][4], double vector[] ,double resultado[] );
void constxvector(int m ,double a, double b[], double resultado[]);


void matrizRotacion(double matrizRot[][3], double angulos[]);


class Uarm{
private:
	CSerialPort puerto;
	
public:	
	DWORD dwMask;
	Uarm();
	int planificador(double qi[],double po[]);
	int planificadorDirecto(double qi[],double qo[],int posicionOriginal);
	int planificador2(double qi[],double po[]);
	int planificadorDirecto2(double qi[],double qo[],int posicionOriginal);
	void MoverBrazoB(double x, double y, double z, int Brazo);
	void MoverBrazoC(double x, double y, double z, int Brazo);
	void MoverBrazo(double x, double y, double z,int Brazo);
	void MoverBrazoAng(double q0, double q1, double q2,double q3,int Brazo);
	void AccionarBrazo(double *tr,int Brazo);
	void AccionarBrazo2(double *tr,int Brazo);
	void AtacarVaso(double x, double y, double z);
	void Servir();
	void PosInicial(int Brazo);
	void torqueOnOff(int OnOff);
	void torqueMotorOnOff(int id, int OnOff); //30MAY2012
	void AbrirCerrarGrip(int Abrir);
	int leerPoseServo(double qi[],int version);
	int leerPoseServo2(double qi[],int version);
	int leerPoseServo3(double qi[]);  // DLF 1JUN2011
	int leerTorqueGrip3(int load[]);  // DLF 5JUN2011
	int leerTorqueServo3(int id); // DLF 13JUN2011
	bool leerMoving(int id); // DLF 8JUN2011
	int cerrarGripper3(int loadMode); // DLF 14JUN2011
	int poseReposoGripper3(); // DLF 14JUN2011
	int poseAgarreGripper3(); // DLF 14JUN2011
	int abrirGripper3(); // DLF 14JUN2011
	int levantarObjetoGripper3(); // DLF 14JUN2011
	int poseIntermediaGripper3(); // DLF 14JUN2011
	bool BrazoMov(int Brazo); // 24JUN2011
	void MoverGripDerAng(double angulo, int velocidad); // 24JUN2011
	void AbrirGripBrazoDer();// 24JUN2011
	int CerrarGripBrazoDer(int loadMode);// 24JUN2011
	void TomarObjeto(double x, double y, double z);//01JUL2011
	void EntregarObjeto();//01JUL2011
	void PosicionReposo();//30MAY2012
	void SalirPosicionReposo();//30MAY2012
	void MoverMunecaDer(double angulo, int velocidad);//30MAY2012

	void SetComplianceMargin(int id,int CWValue,int CCWValue); //23MAY2012
	void SetComplianceSlope(int id,int CWValue,int CCWValue); //23MAY2012
	void SetAllComplianceSlope(int CWValue,int CCWValue); //23MAY2012
	int SelectSlopeLevel(double total, double actual); //23MAY2012
	void SetPunch(int id,int value); //23MAY2012
	void SetAllPunch(int value); //23MAY2012

	void cerrarPuerto();
	void OrientarGrip(void);
	void Saludar();
	void Celebrar(int Brazo);
	void IndicarAlFrente();
	void IndicarIzquierda();
	void IndicarDerecha();
	void anguloGrip(double angle);
	void anguloMuneca(double angle);
	void anguloMuneca3(double angle, int velocidad);
	void anguloGrip3(double angle, int velocidad);
	int anguloSegmentoGripper3(double angle, int segmento, int velocidad);
	void moverAnguloGripper3(int size, int segmento[], double q[], int velocidad);

	~Uarm();
};



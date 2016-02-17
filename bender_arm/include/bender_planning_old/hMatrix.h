#ifndef _HMATRIX_H_
#define _HMATRIX_H_

#include <math.h>

namespace hMatrix
{

void hTransform( double * matrix , double x, double y, double z, double * resp);
void hMatrixMultip( double * M1, double * M2);
double * hIdentity();
void hIdentity(double * resp);
void DH_Matrix(double alpha, double a, double d, double theta, double*T);
void hRotateX(double theta, double * M1);
void hRotateY(double theta, double * M1);
void hRotateZ(double theta, double * M1);
void hTranslation(double x, double y, double z, double * M1);
void hScale(double scale , double * M1);

}

#endif

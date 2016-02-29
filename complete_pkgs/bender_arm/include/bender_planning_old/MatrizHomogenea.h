#include "Matrix.h"

class MatrizHomogenea:public Matrix
{
public:
	MatrizHomogenea();

	void DH(double alpha, double a, double d, double theta);
	void translate(double x,double y,double z);
	void transform(double x, double y, double z, double * resp);
	
	const std::vector<std::vector<double> > & operator = (const Matrix & x);
	const std::vector<std::vector<double> > & operator = (const std::vector<std::vector<double> > & x);

	Matrix op;
};
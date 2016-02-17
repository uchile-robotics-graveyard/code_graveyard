#include <vector>
#include <stdlib.h>
#include <math.h>

class Matrix
{
public:
	Matrix();
	Matrix(const Matrix & x);

	const std::vector<std::vector<double> > & operator = (const Matrix & x);
	const std::vector<std::vector<double> > & operator = (const std::vector<std::vector<double> > & x);
	const std::vector<std::vector<double> > & operator * (const Matrix & x);
	const std::vector<std::vector<double> > & operator * (const std::vector<std::vector<double> > & x);
	double & operator()(int x,int y);


	std::vector<std::vector<double> > data;
	std::vector<std::vector<double> > buffer;
	
};
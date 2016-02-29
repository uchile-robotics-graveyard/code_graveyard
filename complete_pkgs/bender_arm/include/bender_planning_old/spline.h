#ifndef _SPLINE_H_
#define _SPLINE_H_

template <class T>  // x must have operators +, -
class L_Spline1DGen
{
public:
	std::vector<double> x;  // Values for independant variable
	std::vector<T> y;  // Values from the function
	std::vector<T> y2; // Internal values (deriatives)
	int n;

	L_Spline1DGen() : n(0) { }
	inline void addFrom(double xVal, T yVal) {n++; x.resize(n);y.resize(n); x[n-1] = xVal; y[n-1]=yVal;}
	inline void push_back(double xVal, T yVal) {n++; x.resize(n);y.resize(n); x[n-1] = xVal; y[n-1]=yVal;}

	void compute()
	{
		if (x.size() < 3)
			throw "Spline no tiene suficientes puntos";
		std::vector<double> aim(n), ai(n), aiM(n), bim(n), bi(n), biM(n);
		std::vector<T> c(n);
		T zero;
		int i;
		double fac;
		zero = y[0] - y[0];
		y2.resize(n);
		// a[i][i-1] = aim[i] ; a[i][i] = ai[i] ; a[i][i+1] = aiM[i]
		// b[i][i-1] = bim[i] ; b[i][i] = bi[i] ; b[i][i+1] = biM[i]
		for (i=1; i<n-1; i++)
		{
			aim[i] = (x[i] - x[i-1])/6;
			ai[i]  = (x[i+1]-x[i-1])/3;
			aiM[i] = (x[i+1]-x[ i ])/6;
		}
		ai[0] = 1; // Para spline suave
		aiM[0] = 0;
		aim[n-1] = 0;
		ai[n-1] = 1;
		for (i=1; i<n-1; i++)
			c[i] = (y[i+1]-y[i]) /(x[i+1]-x[i]) - (y[i]-y[i-1]) /(x[i]-x[i-1]);
		c[0] = zero;
		c[n-1] = zero;
		for (i=0; i<n; i++)
		{
			bim[i] = 0;  // La identity al principio
			bi[i] = 1;
			biM[i] = 0;
		}
		// [...                                                   | ...]
		// [...  aim[i-1]  ai[i-1]   aiM[i-1]                     | ...]
		// [...            aim[i]     ai[i]     aiM[i]            | ...]
		// [...                      aim[i-1]   ai[i]    aiM[i+1] | ...]
		// [...                                                   | ...]
		for (i=0; i<n-2; i++) // eliminar hacia abajo [A|B]
		{
			// Dejar la diagonal en 1
			// aim[i] = 0
			fac = 1.0/ai[i];
			ai[i] *= fac;
			aiM[i] *= fac;
			bim[i] *= fac;
			bi[i] *= fac;
			biM[i] *= fac;
			// Restar la fila de arriba a la de abajo
			fac = aim[i+1]/ai[i];
			aim[i+1] -= fac*ai[i];
			ai[i+1] -= fac*aiM[i];
			bim[i+1] -= fac*bi[i+1];
			bi[i+1] -= fac*biM[i+1];
		}
		fac = 1.0/ai[n-2];
		ai[n-2] *= fac;
		bim[n-2] *= fac;
		bi[n-2] *= fac;

		// [...                                                   | ...]
		// [...  aim[i-1]  ai[i-1]   aiM[i-1]                     | ...]
		// [...            aim[i]     ai[i]     aiM[i]            | ...]
		// [...                      aim[i-1]   ai[i]    aiM[i+1] | ...]
		// [...                                                   | ...]
		for (i=n-2; i>=1; i--)
		{
			// Los aim = 0
			fac = aiM[i-1]/ai[i];
			aiM[i-1] -= fac*ai[i];
			ai[i-1] -= fac*aim[i];
			biM[i-1] -= fac*bi[i];
			bi[i-1] -= fac*bim[i];
		}
		fac = 1.0/ai[0];
		ai[0] *= fac;
		bi[0] *= fac;
		biM[0] *= fac;
		for (i=1; i<n-1; i++)
			y2[i] = bim[i]*c[i-1] + bi[i]*c[i] + biM[i]*c[i+1];
		y2[0] = zero;
		y2[n-1] = zero;
	}

	void evaluate(double xVal, T &ret, T &retd1)
	{
		double h, a, b, ad1, bd1;
		int k,k1,k2;
		k1=0;
		k2=n-1;
		while (k2-k1 > 1)
		{
			k=(k2+k1) >> 1;
			if (x[k] > xVal)
				k2=k;
			else
				k1=k;
		}
		if (xVal < x[0] || xVal > x[x.size()-1])
			printf("L_Spline1DGen::evaluate() : advertencia: evaluacion fuera de la spline\n");
		if (k1>1 && k2<n-1 && (x[k1] > xVal || x[k2] < xVal))
			throw "L_Spline::evaluate() : error de rango";
		h=x[k2]-x[k1];
		a=(x[k2]-xVal)/h;
		b=(xVal-x[k1])/h;
		ret = a*y[k1]+b*y[k2]+((a*a*a-a)*y2[k1]+(b*b*b-b)*y2[k2])*(h*h)/6.0;
		
		ad1=-1/h; //d/dxVal (a)
		bd1=1/h; //d/dxVal (b)
		retd1=ad1*y[k1]+bd1*y[k2]+((3*a*a*ad1-ad1)*y2[k1]+(3*b*b*bd1-bd1)*y2[k2])*(h*h)/6.0;
	}
};

#endif

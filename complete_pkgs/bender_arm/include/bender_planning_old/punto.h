#ifndef _PUNTO_H_

#define _PUNTO_H_


class Punto4D
{
public:
	double x;
	double y;
	double z;
	double w;
};

inline Punto4D operator + (const Punto4D &p1,const Punto4D &p2){
	Punto4D res;
	res.x=p1.x+p2.x;
	res.y=p1.y+p2.y;
	res.z=p1.z+p2.z;
	res.w=p1.w+p2.w;

	return res;
}
inline Punto4D operator - (const Punto4D &p1,const Punto4D &p2){
	Punto4D res;
	res.x=p1.x-p2.x;
	res.y=p1.y-p2.y;
	res.z=p1.z-p2.z;
	res.w=p1.w-p2.w;

	return res;
}

inline Punto4D operator * (const Punto4D &p1, double factor){
	Punto4D res;
	res.x=(p1.x)*factor;
	res.y=(p1.y)*factor;
	res.z=(p1.z)*factor;
	res.w=(p1.w)*factor;

	return res;
}

inline Punto4D operator * (double factor, const Punto4D &p1){
	Punto4D res;
	res.x=(p1.x)*factor;
	res.y=(p1.y)*factor;
	res.z=(p1.z)*factor;
	res.w=(p1.w)*factor;

	return res;
}

inline Punto4D operator / (const Punto4D &p1, double factor){
	Punto4D res;
	res.x=p1.x/factor;
	res.y=p1.y/factor;
	res.z=p1.z/factor;
	res.w=p1.w/factor;

	return res;
}

typedef struct{
	double x;
	double y;
	double z;
} Punto3D;

#endif

#ifndef _PR_OMNODE_
#define _PR_OMNODE_
#include <math.h>
class OMNode{

public:
	OMNode * leafs[8];
	int state;		//0: libre, 1:parcialmente ocupado, 2:ocupado
	int index;
	bool isleaf[8];
	double x,y,z;

	OMNode(int index,double x,double y,double z);
	bool isFree(double x, double y, double z);
	void setOcupied(double x,double y,double z);
};

#endif
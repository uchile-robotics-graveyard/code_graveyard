#include "bender_planning_old/OMNode.h"

OMNode::OMNode(int i,double xt,double yt,double zt)
{
	index=i;
	state=0;
	x=xt;
	y=yt;
	z=zt;
	for(int i=0;i<8;i++)
		isleaf[i]=false;
}
bool OMNode::isFree(double xt,double yt, double zt)
{
	if(state==0)
		return true;
	if(state==2)
		return false;
	int i=0;
	if(xt>x)
		i=4;
	if(yt>y)
		i+=2;
	if(zt>z)
		i++;
	if(isleaf[i])
	{
		//if(abs(xt-5)+abs(yt-10)+abs(zt-10)<1.8)
		//	return false;
		if(!leafs[i]->isFree(xt,yt,zt))
			return false;
	}
	return true;
}
void OMNode::setOcupied(double xt, double yt, double zt)
{
	if(index==3)
	{
		state=2;
		return;
	}
	state=1;
	int i=0;
	int j=0;
	int k=0;
	if(xt>x)
		i=1;
	if(yt>y)
		j=1;
	if(zt>z)
		k=1;
	if(isleaf[i])
	{
		leafs[4*i+2*j+k]->setOcupied(xt,yt,zt);
	}
	else
	{
		double step=10.0/((double)(pow(2.0,index)));
		isleaf[4*i+2*j+k]=true;
		leafs[4*i+2*j+k]=new OMNode(index+1,x+(2*i-1)*step,y+(2*j-1)*step,z+(2*k-1)*step);
		leafs[4*i+2*j+k]->setOcupied(xt,yt,zt);
	}
}

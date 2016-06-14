#include "funcionesH.h"

enum { constant, circular, symmetric, replicate };
enum {pre,post,both};

void padarrayf(vectorD *b,int &nb, vectorD *a,int &na,int method,vector<int> &padSize,double padVal,int direction);
void ConstantPad1(vectorD *b, int &nb, vectorD *a, int &na, vector<int> &padSize,double padVal,int direction);
//vectorF CircularPad(vectorF a,vector<int> padSize,int direction);


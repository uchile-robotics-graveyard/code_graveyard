#include <vector>
#include <deque>
#include "opencv/highgui.h"
#include <math.h>
#include <pthread.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
//vector<vector<double> > sumaMat(vector<vector<double> > mat1,vector<vector<double> > mat2);
//vector<vector<double> > sumaMat(vector<vector<double> > mat1,double num);
//vector<vector<double> > multiplicaMat(vector<vector<double> > mat1,vector<vector<double> > mat2);
void sumaMat(vector<vector<double> > &matresult, vector<vector<double> > &mat1,vector<vector<double> > &mat2);
void sumaMat(vector<vector<double> > &matresult,vector<vector<double> > &mat1,double num);
void multiplicaMat(vector<vector<double> > &matresult, vector<vector<double> > &mat1,vector<vector<double> > &mat2);
void multiplicaVec(vector<double> &res,vector<vector<double> > &mat1,vector<double> &vec);

double multiplicaVec(vector<vector<double> > &mat1,vector<double> &vec);
void mexTransf(double *dst,double *src,int W, int H, int K);
void transfMex(double *dst,double *src,int W, int H, int K);


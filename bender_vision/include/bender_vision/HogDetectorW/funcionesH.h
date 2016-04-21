//============================================================================
// Name        : funcionesH.h
// Author      : Wilma Pairo
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#pragma once

#include <vector>
#include <deque>

#include "opencv/highgui.h"
#include <math.h>
#include <pthread.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <string>

#define MAX_DEPTH 10000

#define block_types_Other 'O'
    #define block_types_Filter    'F';
    #define block_types_PCAFilter 'P';
    #define block_types_SepQuadDef 'D';

using namespace std;
using namespace cv;

template <class T>
void resizew(std::vector<std::vector<T> > &m, int ni, int nj)
{
    m.resize(ni);
    for (int i=0; i<ni; i++)
        m[i].resize(nj);
}


template <class T>
class vectorw
{
public:
  std::vector<T> v;
  T& operator[](typename std::vector<T>::size_type n) {return v[n-1];}
};
class Par
{
public:
    double d;
    int i;
    bool operator < (const Par &b) const {return d > b.d;}
};
typedef struct bbox_predIn{
    vector<vector<double> > A;
    vector<double> x1;
    vector<double> y1;
    vector<double> x2;
    vector<double> y2;
    vector<double> w;
    vector<double> h;
}bbox_predIn;

//typedef vector<vector<vector<double> > > vectorF;
typedef vector<vector<vector<float> > > vectorW;
typedef struct obj{
    int blocklabel;
    int flip;
    vector<int> size;
    int symbol;
}obj;

typedef struct indices{
    int x;
    int y;
}indices;
/*typedef struct vectorD{
	int with;
	int height;
	vector<double> var;
}vectorD;*/
class vectorD{
public:
	int with;
	int height;
	vector<double> var;
    //int numVals;
    vectorD(void);
    vectorD(int width, int height, vector<double> &val);
	~vectorD(void);
	//vectorD & operator = (const vectorD &otro);
	//int numReg (void);
};
typedef struct filter{
    int blocklabel;
    std::vector<int> size;
    int flip;
    int symbol;

}filter;

typedef struct rule{
    char type[2];
    int lhs;
    vector<int> rhs;
    vector<double> detwindow;
    vector<double> shiftwindow;
    int i;
    vector<vector<double> > anchor;
    vector<obj> offset;
    vector<obj> loc;//loss?

    vector<obj> def;
    //int offset;//dentro lleva un registro llamado blocklabel
    //int loc;//igual al anterior
    vector<int> blocks;
    //Estos se llenan en el proceso
    vectorD score[100];
    int nScore;
    vector<vector<vector<int> > > Ix;
    vector<vector<vector<int> > > Iy;
    //Creando funciones
    vector<vector<double> > loss;
}rule;

typedef struct symbol{
    char type[2];
    vector<int> filter;
    //se llena en el proceso
    vectorD score[100];
    int nScore;

}symbol;
typedef struct block{
    vector<double> w;
    vector<double> lb;
    int learn;
    int reg_mult;
    int dim;
    vector<int> shape;
    char type[2];
    //campo que se llena en el proceso
    vector<double> w_flipped;

}block;

typedef struct feature{
    int sbin;
    int dim;
    int truncation_dim;
    bool extra_octave;//booleano
    int bias;
}feature;

typedef struct stat1{
    vector<double> slave_problem_time;
    vector<double> data_mining_time;
    vector<double> pos_latent_time;
    vector<double> filter_usage;
}stat1;

typedef struct bboxpre{
    vector<double> x1;
    vector<double> y1;
    vector<double> x2;
    vector<double> y2;
}bboxpre;

typedef struct pyras{
    vector<vector<vector<vector<double> > > > feat;
    vector<double> scales;
    vector<int> imsize;
    int num_levels;
    vector<int> valid_levels;
    int padx;
    int pady;
}pyras;

class Modelo {
public:
    char classe[200];//1
    int year;//2
    char note[200];//3
    vector<filter> filters;//4
    vector<vector<rule> >  rules;//5
    vector<symbol> symbols;//6
    int numfilters;//7
    int numblocks;//8
    int numsymbols;//9
    int start;//10
    vector<int> maxsize;//11
    vector<int> minsize;//12
    int interval;//13
    int sbin;//14
    double thresh;//15
    char type[2];//16
    vector<block> blocks;//17
    feature features;//18
    stat1 stats;//19
    vector<bboxpre> bboxpred;//20

    //campo que se incrementa en el proceso
    vectorD scoretpt[100];
    int nScoretpt;

public:
    Modelo();
	~Modelo(void);
	void lecturaModelo(char *direcModelo);
};




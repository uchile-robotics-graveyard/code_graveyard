#ifndef VECTOR_3D__
#define VECTOR_3D__

#include <vector>

using std::vector;

//typedef vector<vector<vector<double> > > vectorF;


class vector3d
{
public:
    vector<double> v;
    int na, nb, nc;
    void resize(int na1, int nb1, int nc1) {this->na = na1; this->nb = nb1; this->nc = nc1;v.resize(na*nb*nc);}
    double &operator()(int a, int b, int c) {return v[c + b*nc + a*nb*nc];}
    int size(int i) {if (i==0) return na; if (i==1) return nb; if (i==2) return nc; return -1;}
    double *data() {return v.data();}
};

class vector3f
{
public:
    vector<float> v;
    int na, nb, nc;
    void resize(int na, int nb, int nc) {this->na = na; this->nb = nb; this->nc = nc; v.resize(na*nb*nc);}
    float &operator()(int a, int b, int c) {return v[c + b*nc + a*nb*nc];}
    int size(int i) {if (i==0) return na; if (i==1) return nb; if (i==2) return nc; return -1;}
    float *data() {return v.data();}
};

class vector2d
{
public:
    vector<double> v;
    int na, nb;
    void resize(int na, int nb) {na = this->na; nb = this->nb; v.resize(na*nb);}
    double &operator()(int a, int b) {return v[b + a*nb];}
    int size(int i) {if (i==0) return na; if (i==1) return nb; return -1;}
    double *data() {return v.data();}
};

class vector2f
{
public:
    vector<float> v;
    int na, nb;
    void resize(int na, int nb) {na = this->na; nb = this->nb; v.resize(na*nb);}
    float &operator()(int a, int b) {return v[b + a*nb];}
    int size(int i) {if (i==0) return na; if (i==1) return nb; return -1;}
    float *data() {return v.data();}
};
#endif // VECTOR_3D__

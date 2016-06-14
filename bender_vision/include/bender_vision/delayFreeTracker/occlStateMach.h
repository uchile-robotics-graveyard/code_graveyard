#ifndef OCCLSTATEMACH_H_
#define OCCLSTATEMACH_H_
#include <vector>
using namespace std;

enum OcclState {undef=-1, visible=0, ocluido=1};

class OcclObs
{
public:
	bool trans_det;
	bool in;
	bool out;

	double likelihood(int state);
	double ptransic(int state1, int state2) {double A[2][2]={{0.9, 0.1}, {0.1, 0.9}}; return A[state1][state2];}
	static double initProb(int state) {if (state==0) return 0.95; else return 0.05;}
};

class OcclStateMach
{
public:
	OcclStateMach();

	void viterbi(OcclObs obs);
	int predictedState() {return X[X.size()-1];}
	void reduceMem();

	bool goBack;
	int nmax;
	vector<double> ppathlog[2];
	vector<int> ipath[2]; // Probabilidades de cada ruta
	vector<int> X;
};



#endif // OCCLSTATEMACH_H_

#include "funcionesH.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef struct Rect_Hog{
    double x;
    double y;
    double width;
    double height;
    double w;
}Rect_Hog;

class funcHoG{
public:
    funcHoG();
	~funcHoG(void);
	int   stopFlag;
	Mat im;
    void run(Modelo &model, double thresh);
    void test(Modelo &model, double thresh,int num_detect);
    int initializationEnded() {return initEnded;}
    void stop();
    void reduceImg(Mat &img, double dimImg);
    //void cleanVectorf(vectorD &A);
    //void cleanSymbolVectorf(vector <symbol> &B);
    vector<Rect_Hog> box;
private:

    void model_sort(Modelo &m, int i, vector<int> &L, vector<int> &V, int num_param);
    void gdetect_dp(pyras &pyra, Modelo &model,cv::Mat &im);
    void symbol_score(Modelo &model, int s);
    void apply_rule(Modelo &model,vector<rule> &r,int posRule,int pady, int padx);
    void apply_structural_rule(Modelo &model,rule &r,int pady, int padx);
    void apply_deformation_rule(Modelo &model, vector<rule> &r);
    void reshape(vectorD *auxW,int &nauxW, vector<double> &w, vector<int> &shape);
    void flipfeat(vectorD *auxf,int &nauxf,vectorD *f,int &nf);
    void ordenFunc(vectorD *f1,int &nf1,vectorD *f,int &nf, vector<int> &p);
    void loc_feat(vectorD &f, Modelo &model, int num_levels);
    void model_get_block(vectorD *w,int &nw, Modelo &m, obj &obje);
    int filter_responses(Modelo &model, pyras &pyra,cv::Mat &im);
    int min(int a, int b);
    void featpyramid(pyras &pyra,Mat &im, Modelo &model, int &padx, int &pady);
    void findMayor(vector<vector<double> > &score, double thresh, vector<double> &tmpI, vector<indices> &indi);
    void findMayorD(vectorD &score, double thresh,
    		vector<double> &tmpI, vector<indices> &indi);
    void ordenarVector(vector<double> &lista, vector<int> &indX);
    void gdetect_parse(vector<vector<double> > &ds1,vector<vector<double> > &bs1,vector<vector<double> > &trees, Modelo &model,pyras &pyra,double thresh,int max_num);
    void gdetect(vector<vector<double> > &ds,vector<vector<double> > &bs,vector<vector<double> > &trees,
             pyras &pyra, Modelo &model, double thresh, int max_num, cv::Mat im);
    void imgdetect(Mat &im,Modelo &model,double thresh,
               vector<vector<double> > &ds,vector<vector<double> > &bs,
               vector<vector<double> > &trees,int num_detect);
    void maximoVec(vector<double> &vec, double &mayor);
    void maximoMat(vector<vector<double> > &mat, double &mayor);
    void buscaElem(vector<vector<double> > &vec,double val,vector<int> &elem);
    void reduceboxes(vector<vector<double> > &b, Modelo &model, vector<vector<double> > &bs);
    void bboxpred_input(bbox_predIn &bbox, vector<vector<double> > &ds,vector<vector<double> > &bs);
    void bboxpred_get(vector<vector<double> > &ds_pred,vector<bboxpre> &bboxpred, vector<vector<double> > &ds, vector<vector<double> > &bs);
    void nms(vector<int> &pick,vector<vector<double > >&boxes, double overlap);

    pthread_mutex_t oqMutex;
    int initEnded;
};






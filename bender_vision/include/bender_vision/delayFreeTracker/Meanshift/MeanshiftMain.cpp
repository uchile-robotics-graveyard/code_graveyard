#include "ObjectTracker.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/ml/ml.hpp>
#include "../RectFrame.h"
#include <string>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include <sys/time.h>

using namespace std;
using namespace cv;

Rect reduceRect(Rect rect, int escRect)
{
	Rect rectita;
	float a=escRect/100.0;
	float b=1.0-escRect/100.0;
	rectita.x = rect.x+(rect.width/2)*b;
	rectita.y = rect.y+(rect.height/2)*b;
	rectita.width = rect.width*a;
	rectita.height = rect.height*a;
	return rectita;
}

int main_meanshift()
{
	vector<string> folder_names;
	Mat image, imageDraw;
	int frameInicial = 0;
	int stepHoG = 0;
	vector<Rect> rects;
	vector<int> ids, frs;

	/*for (int i=20;i<43;i++){  // 4-42
		char folder[500];
		sprintf(folder,"videos/v%d",i);
		folder_names.push_back(folder);
	}*/
	/*for (int i=44;i<66;i++){  // 4
		char folder[500];
		sprintf(folder,"videos/v%d",i);
		folder_names.push_back(folder);
	}*/

	for (int i=2;i<4;i++){  // 4
		char folder[500];
		sprintf(folder,"videosAnt/VD%d",i);
		folder_names.push_back(folder);
	}

	for (int ivid=0; ivid<(int)folder_names.size(); ivid++)
	{
		vector<RectFrame> GT, HOG;
		vector<int> HOGcorr;
		CObjectTracker *m_pObjectTracker = NULL;

		char folder[500];
		sprintf(folder, "%s", folder_names[ivid].c_str());

		bool retb = RectFrame::leerGT(folder, GT);
		if (retb == false)
			{std::cout << "No se puede abrir datosGT.txt" << std::endl;}

		retb = RectFrame::leerHOG(folder, HOG, stepHoG);
		if (retb == false)
			{std::cout << "No se puede abrir datosHOG_step.txt" << std::endl;}

		retb = RectFrame::leerHOGCorr(folder, HOG, HOGcorr, stepHoG);
		if (retb == false)
			{std::cout << "No se puede abrir datosHOGCorr.txt" << std::endl;}

		int iHOG = 0;
		for(unsigned int i=frameInicial; i<GT.size(); i++)
		{

			//Para ver el tiempo de procesamiento
			/*timeval tim;
			gettimeofday(&tim, NULL);
			double t1 = tim.tv_sec + (tim.tv_usec / 1000000.0);*/
			//fin de tiempo de proc

			if (is_valid(GT[i].rect) == false)
				{cout << "Ground truth no valido" << endl; getchar(); exit(1);}

			char arch[500];
			sprintf(arch, "%s/frame_%04d.png", folder, i);

			std::cout << arch <<" "<<i << " " << iHOG <<"-->"<<HOG[iHOG].frame << std::endl;

			image = imread(arch,1);
			if (image.empty())
			{
				std::cout << "Todas las imagenes fueron procesadas" << std::endl;
				break;
			}
			Mat image640 = image.clone();
			resize(image640, image, Size(320,240));
			imageDraw = image.clone();

			while (iHOG < (int)HOG.size() && i >= HOG[iHOG].frame)
				iHOG++;
			std::cout << arch <<"---> "<<i << " " << iHOG <<"-->"<<HOG[iHOG].orig_frame<<" "<<HOG[iHOG].frame << std::endl;

			Rect msh;
			if (m_pObjectTracker == NULL)
			{
				m_pObjectTracker = new CObjectTracker(image.cols, image.rows, MD_RGB);
				int x, y, w, h;
				Rect rec = GT[i].rect;//reduceRect(GT[i].rect,50);
				//Rect rec = reduceRect(HOG[iHOG].rect,50);
				/*w = HOG[iHOG].rect.width;
				h = HOG[iHOG].rect.height;
				x = HOG[iHOG].rect.x + w/2;
				y = HOG[iHOG].rect.y + h/2;*/
				w = rec.width;
				h = rec.height;
				x = rec.x + w/2;
				y = rec.y + h/2;

				//reducir la magen del HOG

				m_pObjectTracker->ObjectTrackerInitObjectParameters(x, y, w, h);
			}
			m_pObjectTracker->ObjeckTrackerHandlerByUser(image.data);

			rectangle(imageDraw, GT[i].rect, Scalar(0,255,0));//GT verde
			if (GT[i].puntaje == 1)
			{
				line(imageDraw, Point(GT[i].rect.x, GT[i].rect.y), Point(GT[i].rect.x+GT[i].rect.width, GT[i].rect.y+GT[i].rect.height), Scalar(0,255,0));
				line(imageDraw, Point(GT[i].rect.x+GT[i].rect.width, GT[i].rect.y), Point(GT[i].rect.x, GT[i].rect.y+GT[i].rect.height), Scalar(0,255,0));
			}
		    //rectangle(imageDraw, HOG[iHOG].rect, Scalar(0,0,255)); // HOG rojo
			msh = Rect(m_pObjectTracker->x(), m_pObjectTracker->y(), m_pObjectTracker->w(), m_pObjectTracker->h());
			rectangle(imageDraw, msh, Scalar(255,0,0)); // meanshift azul

			imshow("Imagen", imageDraw);
			waitKey(1);

			rects.push_back(msh);
			ids.push_back(ivid);
			frs.push_back(i);
			//Para calcular el tiempo de procesamiento
			/*gettimeofday(&tim, NULL);
			double t2 = tim.tv_sec + (tim.tv_usec / 1000000.0);
			printf("\n%.6lf tiempo Evaluacion\n", t2 - t1);
			waitKey(0);*/
			//fin del tiempo de proc
		}

		delete m_pObjectTracker;
		m_pObjectTracker = 0;

	}

	ofstream of;
	of.open("msh.txt");
	for (int i=0; i<(int)rects.size(); i++)
	{
		of << ids[i] << "  " << frs[i] << "  " << rects[i].x << "  " << rects[i].y<< "  " << rects[i].width << "  " << rects[i].height << endl;
	}
	of.close();
	return 0;
}

#ifndef RECT_FRAME_H
#define RECT_FRAME_H

#include <vector>
#include <string>
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class RectFrame
{
public:
	Rect rect;
	unsigned int orig_frame;
	unsigned int frame;
	string frame_str;
	double puntaje;
	double tiempo;

	static bool leerGT(const char *folder,  vector<RectFrame> &GT);
	static bool leerHOG(const char *folder,  vector<RectFrame> &HOG, int stepHoG);
	static bool leerHOGCorr(const char *folder, const vector<RectFrame> &HOG, std::vector<int> &HOGcorr, int stepHoG);
	static bool leerGToccl05(const char *folder,  vector<RectFrame> &GT);
	static bool leerGToccl1(const char *folder,  vector<RectFrame> &GT);
};

inline ostream& operator<<(ostream& os, const RectFrame& rf)
{
    os << "frames: " << rf.orig_frame << " " << rf.frame << " [" << rf.rect.x << "," << rf.rect.y << "->" << rf.rect.x+rf.rect.width << "," << rf.rect.y+rf.rect.height << "]";
    return os;
}

inline bool is_valid(Rect bbx)
{
	//if (bbx.x >= 0 && bbx.y >= 0 && bbx.x+bbx.width < 320 && bbx.y+bbx.height < 240 && bbx.width > 0 && bbx.height > 0)
	if (bbx.x >= 0 && bbx.y >= 0 && bbx.width > 0 && bbx.height > 0)
		if (bbx.width>10 && bbx.height>10)
			return true;
	return false;
}

inline Rect repairBB(Rect bbx)
{
	if (bbx.x < 0)
		bbx.x = 0;
	if (bbx.y < 0)
		bbx.y = 0;
	if (bbx.x + bbx.width >= 320)
		bbx.width = 319 - bbx.x;
	if (bbx.y + bbx.height >= 240)
		bbx.height = 239 - bbx.y;
	return bbx;
}

inline Point center_of_mass(Rect bbx)
{
	Point res;
	res.x = bbx.x + bbx.width/2;
	res.y = bbx.y + bbx.height/2;
	return res;
}

#endif // RECT_FRAME_H

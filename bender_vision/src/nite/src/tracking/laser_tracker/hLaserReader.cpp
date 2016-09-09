#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "tracking/laser_tracker/PersonTracker.h"
#include "tracking/laser_tracker/utilities.h"
#include "stdio.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/SetCameraInfo.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include "bender_msgs/data_follow.h"
#include <std_srvs/Empty.h>

//Para la lectura del laser
#define ScaleFactor 1/12      // Used to draw laser readings

#define RESOLUTION 0.3515625

using namespace std;
using namespace cv;

float g_DepthHist[10000];

PersonTracker* pt;

double currR=-2,currTheta=0;
bool init=false,lost=true;

void DibujarBase(IplImage * BASE) {
	cvSetZero(BASE);
	//double sin45 = sin(Deg2Rad(45.0));
	double cos45 = cos(Deg2Rad(45.0));

	int a = int(9000*ScaleFactor/2);
	int b = int(8000*ScaleFactor/2*cos45);

	CvPoint pCenter    = cvPoint(a  ,a  );
	CvPoint pUp        = cvPoint(a  ,0  );
	CvPoint pDown      = cvPoint(a  ,2*a);
	CvPoint pLeft      = cvPoint(0  ,a  );
	CvPoint pRight     = cvPoint(2*a,a  );
	CvPoint pUpLeft    = cvPoint(a-b,a-b);
	CvPoint pUpRight   = cvPoint(a+b,a-b);
	CvPoint pDownLeft  = cvPoint(a-b,a+b);
	CvPoint pDownRight = cvPoint(a+b,a+b);

	CvScalar color = CV_RGB(255,0,0);

	cvDrawCircle(BASE,pCenter,1000*ScaleFactor,color,1,8,0);
	cvDrawCircle(BASE,pCenter,2000*ScaleFactor,color,1,8,0);
	cvDrawCircle(BASE,pCenter,3000*ScaleFactor,color,1,8,0);
	cvDrawCircle(BASE,pCenter,4000*ScaleFactor,color,1,8,0);

	cvDrawLine(BASE,pLeft   ,pRight    ,color,1,8,0);
	cvDrawLine(BASE,pUp     ,pDown     ,color,1,8,0);
	cvDrawLine(BASE,pUpLeft ,pDownRight,color,1,8,0);
	cvDrawLine(BASE,pUpRight,pDownLeft ,color,1,8,0);
}

void DibujarLecturaSegmento(IplImage * Base,long * Data, int Size, PersonTracker* pt){
	std::vector<Segment> segments = pt->getSegments();
	int numSegments = (int)segments.size();
	int currSegNum=0;
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 1.0, 0, 1, CV_AA);
	double a = 9000*ScaleFactor/2;             // 9000 = alto y ancho del recuadro
	double b = 8000*ScaleFactor/2;             // 8000 = diametro del circulo
	//CvPoint iniPoint = cvPoint(int(a),int(a));
	//char strLabel1[100]="0";
	int pintado=0, color=0;

	//ROS_INFO_STREAM(Size);
	for (int i=0;i<Size;i++){
		double drawingAngle =Deg2Rad(150+RESOLUTION*i);
		if (Data[i]>20){
			// Angulo actual
			double angle = -N135+RESOLUTION*i;
			// Esta el angulo dentro de un segmento??
			bool angleInSegment = false;
			for(int j=0;j<numSegments;j++) {
				if(segments[j].hasAngle(angle)) {
					angleInSegment = true;
					currSegNum = j;
					break;
				}
			}
			// Elegir color dependiendo del tipo de objeto
			CvScalar lineColor;
			// Dibujar linea
			if(angleInSegment) {
				TrackingStatus st = pt->getTrackingStatus();
				int trackIndex = pt->getTrackedIndex();

				// Objeto bien trackeado en rojo
				if((st == TRACK_OK) && trackIndex == currSegNum) {
					lineColor = CV_RGB(255,0,0);
				}
				// Objeto trackeado bajo oclusion en rojo oscuro
				else if((st == TRACK_OCCLUSION_RISK || st == TRACK_OCCLUSION || st == TRACK_LOST || st == TRACK_RECOVER) && trackIndex == currSegNum) {
					lineColor = CV_RGB(120,0,0);
				}
				// Objeto cercano en lila
				else if ((st == TRACK_OCCLUSION_RISK || st == TRACK_OCCLUSION) && pt->getNearIndex() == currSegNum) {
					lineColor = CV_RGB(255,0,255);
				}
				// Otros segmentos en tonos de verde
				else {
					if(currSegNum%2 == 0)
						lineColor = CV_RGB(30,240,0);
					else
						lineColor = CV_RGB(15,120,0);
					color=1;
				}
			} else {
				// Cualquier otra cosa en azul
				lineColor = CV_RGB(0,0,255);
			}
			if (int(a+Data[i]*ScaleFactor*cos(drawingAngle))>a-100 &&
					int(a+Data[i]*ScaleFactor*cos(drawingAngle))<a+100 &&
					pintado==0 &&
					color==1){
				pintado=1;
				color=0;
			}

			cvDrawLine(Base,
					cvPoint(
							int(a+Data[i]*ScaleFactor*cos(drawingAngle)),
							int(a-Data[i]*ScaleFactor*sin(drawingAngle))),
							cvPoint(
									int(a+b*cos(drawingAngle)),
									int(a-b*sin(drawingAngle))),
									lineColor,2,8,0);
		}

	}

}

int j=0,inicializado = 0;
IplImage * LaserBase;
sensor_msgs::LaserScan laser_scan; 
int ingresa=1;
long * Mx1;
bool start_init=false;
//Mat im1;

void scanValues(const sensor_msgs::LaserScan laser)
{
	int data_max =laser.ranges.size();
	if (ingresa)
	{
		pt = new PersonTracker(data_max, RESOLUTION);
		LaserBase = cvCreateImage( cvSize(9000*ScaleFactor,9000*ScaleFactor), 8, 3 );
		ingresa=0;
		Mx1=new long[laser.ranges.size()];
	}

	for (unsigned int i=0; i<laser.ranges.size();i++)
	{
		Mx1[i]=round(laser.ranges[i]*1000);
	}

	pt->processNewReading(Mx1,laser.ranges.size());

	if(pt->isCalibrationOk() && inicializado == 0 && start_init)
	{
		printf("Calibration OK, tracking will start\n");
		pt->startTracking();
		inicializado = 1;
		init=true;
		ROS_ERROR("Inicializado\n");
	}
	if(pt->getTrackedIndex() != -1) {
		SegmentFeatures& ts = pt->getTrackedSegment();
		init=true;
		lost=false;
		currR     = ts.getMeanRadius();
		currTheta = ts.getMeanAngle();
	}else
	{
		lost=true;
	}
	DibujarBase(LaserBase);
	DibujarLecturaSegmento(LaserBase,Mx1,data_max,pt);
	cvShowImage("imagen", LaserBase);
	cv::moveWindow("imagen", 0, 0);
	cvWaitKey(1);

}

bool initTracker(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res)
{
	start_init=true;
	return true;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_tracker");
	ros::NodeHandle n("~");
	//int data_max=678;

	ingresa=1;
	ros::Subscriber hokuyoSubscriber = n.subscribe("/bender/sensors/laser_waist/scan", 1, scanValues);
	ros::ServiceServer initService = n.advertiseService("initialize_tracker",initTracker);

	ros::Publisher data_pub = n.advertise<bender_msgs::data_follow>("tracker_estimation", 50);

	bender_msgs::data_follow odom;

	while (ros::ok()) {

		ros::spinOnce();
		odom.init=init;
		odom.lost=lost;
		odom.dist = currR;
		odom.theta = currTheta;
		data_pub.publish(odom);


	}

	cvReleaseImage(&LaserBase);
	delete(Mx1);
	return 0;
}

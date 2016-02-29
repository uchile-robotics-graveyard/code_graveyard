#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>

//Opencv
#include "cv.h"
#include "highgui.h"


//C++
#include <iostream>
#include <math.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string.h>
#include <vector>
#include <cstdlib>
#include <unistd.h>

#include <bender_srvs/Onoff.h>
#include <bender_msgs/CartesianInfo.h>
#include <bender_srvs/GripperInfo.h>

#include <stdio.h>


using namespace std;
using namespace cv;


bool activo = false;
IplImage *imageRGB = NULL;
ros::Subscriber sub2, sub3;

ros::Publisher pub;

struct pos3d
{
  int xh;
  int ys;
  int z;
  int r;
};

struct grip
{
  Point cms;
  int id;
};


pos3d centerC1,centerC2,centerC3;
bender_srvs::GripperInfo::Response inf;
vector < grip >  gripimg;
pos3d posgripper;
bool valgrip=true;//TODO change a false cuando obtenga la informacion de los brazos

int dist(cv::Vec3b& pixel,pos3d center){
//      cout<<(int)pixel[0]<<" "<<(int)pixel[1]<<endl;

  int d = abs((int)pixel[0]-center.xh);
   d += abs((int)pixel[1]-center.ys);
   return d;
}


Mat processimg(Mat image, pos3d center){
    Mat fullImageHSV;
    Mat binar(image.rows,image.cols, CV_8UC1, Scalar(0));
    cvtColor(image, fullImageHSV, CV_BGR2HSV);  
  
    cv::MatIterator_<cv::Vec3b> it = image.begin<cv::Vec3b>(),
                    it_end = image.end<cv::Vec3b>();
    MatIterator_<uchar> it2 = binar.begin<uchar>();
    
    for(; it != it_end; ++it,++it2)
    {
	if(dist(*it,center) <= center.r)
	  *it2 = (uchar)255;
    }   
    
    
    return binar;

}


void findContour(Mat binar,Mat& img,int id){
   
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours( binar, contours, hierarchy,
    CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

	   
	       
    int cont = 0;
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Moments> mu(contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f> mc( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    { 
	mu[i] = moments( contours[i], false ); 
	mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
	approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
	boundRect[i] = boundingRect( Mat(contours_poly[i]) );
       
	if(mu[i].m00>60 &&mu[i].m00<500) cont++;
    }

     
    inf.cx.resize(cont);
    inf.cy.resize(cont); 
    inf.BBoxes.resize(cont); 
    inf.n = cont;
     
     Mat drawing = Mat::zeros( binar.size(), CV_8UC3 );
     cont=0;
      for(unsigned int i=0;i<contours.size();i++)
      {
	  if(mu[i].m00>60 &&mu[i].m00<500)
	  {
	    Scalar color = Scalar(255, 255, 255 );
	    drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
//  	    circle( img, mc[i], 3, color, -1, 8, 0 );
// 	    rectangle( img, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
	    inf.cx[cont] = mc[i].x;
	    inf.cy[cont] = mc[i].y;
	    grip temp;
	    temp.cms = mc[i];
	    temp.id = id;
	    gripimg.push_back(temp);
	    
	    inf.BBoxes[cont].x = boundRect[i].x;
	    inf.BBoxes[cont].y = boundRect[i].y;
	    inf.BBoxes[cont].width = boundRect[i].width;
	    inf.BBoxes[cont].height = boundRect[i].height;
	    cont++;
	    
	  }
      }
      
}



void processColors( Mat img){
  
    Mat binar = processimg(img, centerC1);   // imshow("binarN",binar);   cvWaitKey(10);
    Mat binar1 = processimg(img, centerC2);  //  imshow("binarA",binar1);   cvWaitKey(10);
    Mat binar2 = processimg(img, centerC3);  // imshow("binarF",binar2);   cvWaitKey(10);
    
    findContour(binar,img,1);
    findContour(binar1,img,2);  
    findContour(binar2,img,3);

    float val,val1,val2;
    for(int i=0;i<gripimg.size();i++)
    {
      bool close = true;
      for(unsigned int j=0;j<gripimg.size();j++)
	if(i!=j){
	  val = abs(gripimg[i].cms.x - gripimg[j].cms.x) + abs(gripimg[i].cms.y - gripimg[j].cms.y);
	  if(val < 40 && val>5 && gripimg[i].id !=gripimg[j].id)
	      close =false;
	}

      if(close){
	gripimg.erase(gripimg.begin()+i);
	i--;
      }
    }
    
//      for(unsigned int i=0;i<gripimg.size();i++)
//       {
// // 	if (gripimg[i].id == 1 ) circle( img, gripimg[i].cms, 2, Scalar(0, 255, 0 ), -1, 8, 0 );
// 	if (gripimg[i].id == 2 ) circle( img, gripimg[i].cms, 3, Scalar(255, 0, 0 ), -1, 8, 0 );
// // 	if (gripimg[i].id == 3 ) circle( img, gripimg[i].cms, 2, Scalar(0, 0, 255 ), -1, 8, 0 );
//       }
      
      
      vector < Point > finalP,finalPm;
    
      int v,v1,v2;
     
   for(int i=0;i<gripimg.size();i++)
      for(unsigned int j=0; gripimg[i].id ==1 && j<gripimg.size();j++){
	 for(unsigned int k=0; gripimg[j].id==2 &&k<gripimg.size();k++){
	      v=0;
	      v1=0;
	      v2=0;
	    
	      if(gripimg[k].id == 3){
		    val = abs(gripimg[i].cms.x - gripimg[j].cms.x) + abs(gripimg[i].cms.y - gripimg[j].cms.y);
		    val1 = abs(gripimg[i].cms.x - gripimg[k].cms.x) + abs(gripimg[i].cms.y - gripimg[k].cms.y);
 		    val2 = abs(gripimg[k].cms.x - gripimg[j].cms.x) + abs(gripimg[k].cms.y - gripimg[j].cms.y);
		    if(val < 40 ) v=1;
		    if(val1 < 40 ) v1=1;
 		    if(val2 > 10 ) v2=1;
		
		    if(v + v1 + v2 == 3 ){
		      Point f,fm;
		      f.x = min(min(gripimg[i].cms.x, gripimg[j].cms.x),gripimg[k].cms.x);
		      f.y = min(min(gripimg[i].cms.y, gripimg[j].cms.y), gripimg[k].cms.y);  
		      fm.x = max(max(gripimg[i].cms.x, gripimg[j].cms.x),gripimg[k].cms.x);
		      fm.y = max(max(gripimg[i].cms.y, gripimg[j].cms.y), gripimg[k].cms.y);  

		      finalP.push_back(f);
		      finalPm.push_back(fm);
		    }
		}
	    }
	}

      
     for(unsigned int i=0;i<finalP.size();i++)
      {
// 	circle( img, finalP[i], 4, Scalar(0, 255, 0 ), -1, 8, 0 );
	rectangle( img, finalP[i], finalPm[i],  Scalar(0, 255, 0 ), 2, 8, 0 );
      }
	    imshow("prueba",img);
	    cvWaitKey(10);
	    gripimg.clear();
}

void procesandoRGB(const sensor_msgs::ImageConstPtr& imgin){

    IplImage *cv_image=NULL;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {

	if (activo && valgrip)
	{
	    //valgrip = false;TODO
	    cv_ptr = cv_bridge::toCvCopy(imgin, "bgr8");
	    cv_image = cvCloneImage(&(IplImage)cv_ptr->image); 
	}
	
    }
    catch (cv_bridge::Exception error)
    {
	ROS_ERROR("error");
    }
    
    
    if (activo && valgrip)
    {

	Mat img (cv_image); 

	//Mat img_roi = img(cv::Rect(0,180,320,240)); TODO Pasar el 3d a 2d de imagen, erificar si esta en la iamgen

	processColors(img);
    }
}


void grasp_info(bender_msgs::CartesianInfo info){
    posgripper.xh = info.x;
    posgripper.ys = info.y;
    posgripper.z = info.z;
    valgrip = true;
}



bool visualState(bender_srvs::Onoff::Request  &req, bender_srvs::Onoff::Response &res) 
{
  
    ros::NodeHandle node2;
    if(req.select){
	if (activo) {
		    ROS_INFO_STREAM("Already turned on");
	} else {
		    activo = true;
		    sub2 = node2.subscribe("/camera/rgb/image_raw", 1, procesandoRGB);//Asus
		    sub3 = node2.subscribe("/bender/arm/right_arm/grasp_info", 1, grasp_info);
		    ROS_INFO_STREAM("Turning on . . . OK");
	}
    }
    else {
	  if (activo) {
		    sub2.shutdown();
		    activo = false;
		    ROS_INFO_STREAM(" Turning off . . . OK");
	} else {
		    ROS_INFO_STREAM("Already turned off");
	}
      
    }

    return true;
    
}

bool GripperInfo(bender_srvs::GripperInfo::Request  &req, bender_srvs::GripperInfo::Response &res) 
{
  
    res = inf;
    
    return true;
    
}


int main(int argc, char **argv) {
	ros::init(argc,argv,"visual_servoing");
	ros::NodeHandle node;
	    namedWindow("prueba", 1);
	//TODO hacerlo con barras y detectar los 3 colores separados
	centerC1.xh =141 ; 	centerC1.ys =197 ; centerC1.r=50;//Naranjo 
	centerC2.xh =247 ; 	centerC2.ys =162 ; centerC2.r=50;//Azul
	centerC3.xh =234 ; 	centerC3.ys =149 ; centerC3.r=50;//Fucsia
	
// 	createTrackbar("centerC3.xh", "prueba", &centerC3.xh, 255);
// 	createTrackbar("centerC3.ys", "prueba", &centerC3.ys, 255);
//   Mat input1 = imread("grip.png", 1);
// //     Mat binar = 
//     processColors(input1);
//   
	ros::ServiceServer service1 = node.advertiseService("visualState", visualState);
	ros::ServiceServer service2 = node.advertiseService("GripperInfo", GripperInfo);

	Mat back;
	
	int nfr = 0;
	while (ros::ok())
	{
		ros::spinOnce();
		usleep(10);
	}
	return 0;
}

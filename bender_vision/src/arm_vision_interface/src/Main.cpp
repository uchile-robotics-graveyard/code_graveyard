#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <string.h>
#include <std_srvs/Empty.h>
#include "bender_srvs/ObjectDetection.h"
#include "bender_srvs/PlaneHeight.h"
#include "bender_srvs/String.h"
#include "bender_msgs/PlaneDetection.h"
#include "bender_msgs/SiftDetectionFloor.h"
#include "bender_msgs/SiftDetectionDepth.h"

using namespace std;

vector<string> name;
vector<string> type;
vector<string> detector;
vector<double> x;
vector<double> y;
vector<double> z;

vector<string> name2;
vector<string> type2;
vector<string> detector2;
vector<double> x2;
vector<double> y2;
vector<double> z2;

double plane_height = 0;

string getObjectType(string object_name){

	ros::NodeHandle priv("~");
	ros::ServiceClient get_class_client;
	get_class_client = priv.serviceClient<bender_srvs::String>("/bender/utils/mapper/get_class");

	while ( ros::ok() && !get_class_client.waitForExistence(ros::Duration(3.0)) );

	bender_srvs::String string_srv;
	string_srv.request.data = object_name;

	get_class_client.call(string_srv);

	return string_srv.response.data;

	/*
	if( object_name.compare("brush")==0 ||
		object_name.compare("detergent")==0 ||
		object_name.compare("shampoo")==0 ||
		object_name.compare("sponge")==0 ||
		object_name.compare("tooth_paste")==0) {

		return "cleaning_stuff";
	if( name.compare("beer")==0 || name.compare("orange_juice")==0 ||
		name.compare("chocolate_milk")==0 || name.compare("grape_juice")==0 ||
		name.compare("energy_drink")==0 || name.compare("cola")==0 || 
		name.compare("water")==0 || name.compare("milk")==0 ||
		name.compare("lata")==0 || name.compare("botella")==0)
		return "drink";
	if( name.compare("chocolate_cookies")==0 || name.compare("biscuits")==0 ||
		name.compare("pringles")==0 || name.compare("crackers")==0 ||
		name.compare("bread")==0 || name.compare("chips")==0 || 
		name.compare("strawberry_coockies")==0 )
		return "snacks";
	if( name.compare("noodles")==0 || name.compare("beans")==0 ||
		name.compare("yeast")==0 || name.compare("coffee")==0 || 
		name.compare("baby_food")==0)

		return "food";
	}
	*/

	return "unknown";
}

void PlaneDetectionCallback(bender_msgs::PlaneDetection msg)
{
	plane_height = msg.heightOfPlane;
	for(int j=0;j<msg.nObjectFound;j++){
		for(int i=0;i<x.size();i++){
			if(msg.names[j].length()==0){
				if(sqrt(pow(msg.xObj[j]-x[i],2)+pow(msg.yObj[j]-y[i],2))<5){
					if(name[i].length()>0)
						return;
					else{
						x[i]=(msg.xObj[j]);
						y[i]=(msg.yObj[j]);
						z[i]=(msg.zObj[j]);
						detector[i]="Plane";
						return;
					}
				}
				/*
				name.push_back(msg.names[j]);
				type.push_back(getObjectType(msg.names[j]));
				x.push_back(msg.xObj[j]);
				y.push_back(msg.yObj[j]);
				z.push_back(msg.zObj[j]+4);
				return;*/
			}
			else{
				if(strcmp(msg.names[j].c_str(),name[i].c_str())==0){
					x[i]=(msg.xObj[j]);
					y[i]=(msg.yObj[j]);
					z[i]=(msg.zObj[j]);
					detector[i]="Plane";
					return;
				}
			}
		}
		name.push_back(msg.names[j]);
// 		type.push_back(getObjectType(msg.names[j]));
		type.push_back("");
		x.push_back(msg.xObj[j]);
		y.push_back(msg.yObj[j]);
		z.push_back(msg.zObj[j]);
		detector.push_back("Plane");
		return;
	}
}

void SiftDetectionCallback(bender_msgs::SiftDetectionFloor msg)
{
	for(int j=0;j<msg.xSift.size();j++){
		for(int i=0;i<x.size();i++){
			if(strcmp(msg.names[j].c_str(),name[i].c_str())==0)
			{
				if(detector[i].compare("Plane")==0){
				//	if(sqrt(pow(msg.xSiftBase[j]-x[i],2)+pow(msg.ySiftBase[j]-y[i],2)+pow(msg.heightOfPlane+7-z[i],2))<3)
					if(sqrt(pow(msg.xSift[j]-x[i],2)+pow(msg.ySift[j]-y[i],2))<2)				
					{
						x[i]=(msg.xSift[j]);
						y[i]=(msg.ySift[j]);
						z[i]=(msg.zSift[j]);
						detector[i]="Sift";
					}
					return;
				}
				else{
					if(detector[i].compare("Sift")==0){
						x[i]=(msg.xSift[j]);
						y[i]=(msg.ySift[j]);
						z[i]=(msg.zSift[j]);
						detector[i]="Sift";
						return;
					}
				}
			}
		}
		name.push_back(msg.names[j]);
		type.push_back(getObjectType(msg.names[j]));
//		x.push_back(msg.xSift[j]);
//		y.push_back(msg.ySiftBase[j]);
		x.push_back(msg.xSift[j]);
		y.push_back(msg.ySift[j]);
		z.push_back(msg.zSift[j]);
		detector.push_back("Sift");
		return;
	}
}

bool eval ( float min , float large, float eval){
	bool cons = 0.02;
	if(min - cons < eval && min + large + cons > eval)
		return true;
	return false;

}

void SiftDepthDetectionCallback(bender_msgs::SiftDetectionDepth msg)
{
	cout<<"detect"<<endl;
	for(int j=0;j<msg.x.size();j++){
		bool exist = false;
		for(int i=0;i<x2.size();i++){
			if(strcmp(msg.names[j].c_str(),name2[i].c_str())==0)
			{
				//	if(sqrt(pow(msg.xSiftBase[j]-x[i],2)+pow(msg.ySiftBase[j]-y[i],2)+pow(msg.heightOfPlane+7-z[i],2))<3)
					if(eval(msg.x[j],msg.width[j],x2[i]) && eval(msg.y[j],msg.height[j],y2[i]) && eval(msg.z[j],msg.length[j],z2[i]))				
					{
						// cout<<"asd"<<endl;
						// cout<<msg.x[j]<<" "<<msg.width[j]<<" "<<(float)(msg.width[j]/2)<<" "<<(msg.x[j]+ (float)(msg.width[j]/2))<<endl;

						x2[i]=(msg.x[j]+ (float)(msg.width[j]/2));
						y2[i]=(msg.y[j]+ (float)(msg.height[j]/2));
						z2[i]=(msg.z[j]+ (float)(msg.length[j]/2));
						detector2[i]="Sift";
						exist = true;
						// cout<<"asd1"<<endl;
					}
					

			}
		}
		if(!exist){
			// cout<<"asd3"<<endl;
			name2.push_back(msg.names[j]);
			type2.push_back("");
			x2.push_back(msg.x[j]+ msg.width[j] /2 );
			y2.push_back(msg.y[j]+ msg.height[j] /2);
			z2.push_back(msg.z[j]+ msg.length[j] /2);
			detector2.push_back("SiftDepth");
		}
	}
}

bool DetectedObj(bender_srvs::ObjectDetection::Request &req, bender_srvs::ObjectDetection::Response &res){
	res.name=name;
	res.type=type;
	res.detector=detector;
	res.x=x;
	res.y=y;
	res.z=z;
	return true;
}

bool OnlyDetectedObj(bender_srvs::ObjectDetection::Request &req, bender_srvs::ObjectDetection::Response &res){
	res.name=name2;
	res.type=type2;
	res.detector=detector2;
	res.x=x2;
	res.y=y2;
	res.z=z2;
	return true;
}


bool ResetObj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	name.clear();
	type.clear();
	detector.clear();
	x.clear();
	y.clear();
	z.clear();
	return true;
}
bool PlaneHeightForward(bender_srvs::PlaneHeight::Request &req, bender_srvs::PlaneHeight::Response &res){
	res.height = plane_height;
	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "arm_vision_interface");
	ros::NodeHandle n;
	
	ros::Subscriber sub_plane = n.subscribe("/PlaneDetection", 10, PlaneDetectionCallback);
	ros::Subscriber sub_sift = n.subscribe("/SiftDetectionFloor", 10, SiftDetectionCallback);
	ros::Subscriber sub_siftdepth = n.subscribe("/bender/pcl/SiftDetectionDepth", 10, SiftDepthDetectionCallback);
	ros::ServiceServer obj_server2 = n.advertiseService("/arm_vision_interface/onlydetect_obj", OnlyDetectedObj);
	ros::ServiceServer obj_server = n.advertiseService("/arm_vision_interface/detect_obj", DetectedObj);
	ros::ServiceServer reset_obj_server = n.advertiseService("/arm_vision_interface/reset_obj", ResetObj);
	ros::ServiceServer plane_height_server = n.advertiseService("/arm_vision_interface/plane_height",PlaneHeightForward);
	ros::spin();
	return 0;
}

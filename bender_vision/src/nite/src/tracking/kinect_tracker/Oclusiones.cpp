#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include "bender_msgs/KinectTrackerData.h"
#include "bender_msgs/OclusionData.h"
#include "bender_srvs/ID.h"
#include <vector>

bender_msgs::KinectTrackerData OperatorData;
bender_msgs::KinectTrackerData LastOperatorData;
std_msgs::Int8MultiArray LastOccluderDataID;

int operatorID;

ros::Publisher oclusion_pub;
ros::Publisher oclusionList_pub;

int FindIDPosition(bender_msgs::KinectTrackerData Data, int ID){

	for (int m=0; m<Data.Distance.size(); m++){

		if(ID==Data.ID[m]){

			return m;
		}
	}

	return -1;

}

void OclusionDetector(const bender_msgs::KinectTrackerData &Detections){

	if (LastOperatorData.ID.size()==0){

		for (int m=0; m<Detections.Distance.size(); m++){

			LastOperatorData.Distance.push_back(Detections.Distance[m]);
			LastOperatorData.ID.push_back(Detections.ID[m]);
			LastOperatorData.Theta.push_back(Detections.Theta[m]);

		}

		return;
	}

	OperatorData.ID.clear();
	OperatorData.Distance.clear();
	OperatorData.Theta.clear();

	if (operatorID==-1)
		return;

	for (int m=0; m<Detections.Distance.size(); m++){

		if(operatorID==Detections.ID[m]){
			OperatorData.Distance.push_back(Detections.Distance[m]);
			OperatorData.ID.push_back(Detections.ID[m]);
			OperatorData.Theta.push_back(Detections.Theta[m]);
			break;
		}
	}

	int dim = Detections.Distance.size();
	int dimOccluders = LastOccluderDataID.data.size();
	bender_msgs::OclusionData Occluder;


	//std::cout << "OperatorData.ID.size = " << OperatorData.ID.size() << "\n";

	if (OperatorData.ID.size()==0){

		oclusionList_pub.publish(LastOccluderDataID);

		//std::cout << "pub last occluder data" << "\n";

		for (int i=0; i<dimOccluders; i++){
			for (int j=0; j<dim; j++){

				if (LastOccluderDataID.data[i]==Detections.ID[j]){

					Occluder.ID.push_back(Detections.ID[j]);//id del objeto
					Occluder.dist.push_back(Detections.Distance[j]);//distancia del objeto
					Occluder.theta.push_back(Detections.Theta[j]);//angulo del objeto
					Occluder.position.push_back("LOST_TRACKER");
					Occluder.direction.push_back(" ");

				}
			}
		}

		if (Occluder.ID.size()>0)
			oclusion_pub.publish(Occluder);


		return;
	}

	//std::cout << "dim = " << dim << "\n";


	if (dim > 1){
		double auxTheta=20;
		for (int i=0; i<dim; i++){
			LastOccluderDataID.data.clear();
			if (OperatorData.ID[0]!=Detections.ID[i]){
				if (fabs(OperatorData.Theta[0] - Detections.Theta[i])<auxTheta){
					//printf("Angulo menor a auxtheta\n");
					if (Detections.Distance[i] < OperatorData.Distance[0]){
						//printf("La distancia es menor.........\n");
						Occluder.ID.push_back(Detections.ID[i]);//id del objeto
						LastOccluderDataID.data.push_back(Detections.ID[i]); // Guardo ID Oclusor
						Occluder.dist.push_back(Detections.Distance[i]);//distancia del objeto
						Occluder.theta.push_back(Detections.Theta[i]);//angulo del objeto

						if (Detections.Theta[i] > OperatorData.Theta[0]){
							Occluder.position.push_back("LEFT");
						}
						else{
							Occluder.position.push_back("RIGHT");
						}

						int pos = FindIDPosition(LastOperatorData,Detections.ID[i]);

						if (pos != -1 && LastOperatorData.Theta[pos] > Detections.Theta[i]){
							Occluder.direction.push_back("LEFT_TO_RIGHT");
						}
						else if (pos != -1 && LastOperatorData.Theta[pos] < Detections.Theta[i]){
							Occluder.direction.push_back("RIGHT_TO_LEFT");
						}

					}
				}
			}
		}

		//std::cout << "Occluder.Id.size = " << Occluder.ID.size() << "\n";

		//if (Occluder.ID.size()>0)
		oclusion_pub.publish(Occluder);

		LastOperatorData.ID = Detections.ID;
		LastOperatorData.Distance = Detections.Distance;
		LastOperatorData.Theta = Detections.Theta;

	}
}

//Funcion que me permite recuperar el ID del sujeto trackeado

bool SetID(bender_srvs::ID::Request  &req,	bender_srvs::ID::Response &res) 	{

	if (req.ID > 0){

		operatorID = req.ID;
		return true;
	}

	return false;
}

int main(int argc, char **argv){

	ros::init(argc,argv,"kinect_occlusion_detector");
	ros::NodeHandle node("~");
	ros::Subscriber sub = node.subscribe("/bender/vision/kinect_tracker/tracker_estimation", 1, OclusionDetector);
	ros::ServiceServer serv = node.advertiseService("setID", SetID);

	operatorID = -1;

	oclusion_pub = node.advertise<bender_msgs::OclusionData>("pos_oclusion", 50);
	oclusionList_pub = node.advertise<std_msgs::Int8MultiArray>("list_oclusion", 50);

	ros::spin();
	return 0;
}

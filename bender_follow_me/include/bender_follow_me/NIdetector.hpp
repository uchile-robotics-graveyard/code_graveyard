/*
 * NIdetector.h
 *
 *  Created on: 28-10-2014
 *      Author: matias
 */

#ifndef NIDETECTOR_HPP_
#define NIDETECTOR_HPP_

// C++
#include <string.h>
#include <vector>
#include <math.h>
#include <boost/scoped_ptr.hpp>

#include <opencv2/opencv.hpp>

// ROS
#include "ros/ros.h"
#include "ros/package.h"
#include <tf/transform_listener.h>
#include <bender_config/ParameterServerWrapper.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// ROS messages / services
#include <bender_srvs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <bender_follow_me/BodyDetections.h>
#include <bender_follow_me/SingleBodyDetection.h>
#include <bender_srvs/Transformer.h>

// OpenNI
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnHash.h>
#include <XnLog.h>
#include <XnPropNames.h>

// NITE
#include <XnVNite.h>
#include <XnVPointControl.h>


namespace bender_follow_me {

#define MAX_DEPTH 10000
XnBool g_bDrawDepthMap;
XnBool g_bDrawPixels;

/*
//Creando la clase del PointDrawer
static XnBool bShouldPrint = false;
class XnVPointDrawer : public XnVPointControl
{
public:
	XnVPointDrawer(XnUInt32 nHistory, xn::DepthGenerator depthGenerator, xn::UserGenerator userGenerator) :XnVPointControl("XnVPointDrawer"),m_nHistorySize(nHistory), m_DepthGenerator(depthGenerator),m_bDrawDM(false), m_bFrameID(false) {
		g_UserGenerator = userGenerator;
		m_pfPositionBuffer = new XnFloat[nHistory*3];
	}

	virtual ~XnVPointDrawer() {
		std::map<XnUInt32, std::list<XnPoint3D> >::iterator iter;
		for (iter = m_History.begin(); iter != m_History.end(); ++iter)	{
			iter->second.clear();
		}
		m_History.clear();

		delete []m_pfPositionBuffer;
	}
	XnPoint3D ptProjective ;

	// Handle creation of a new point
	void OnPointCreate(const XnVHandPointContext* cxt)
	{
		//printf("** %d\n", cxt->nID);
		// Create entry for the hand
		m_History[cxt->nID].clear();
		bShouldPrint = true;
		OnPointUpdate(cxt);
		bShouldPrint = true;
	}

	// Handle destruction of an existing hand
	void OnPointDestroy(XnUInt32 nID) {
		// No need for the history buffer
		m_History.erase(nID);
	}

	// Handle new position of an existing point
	void OnPointUpdate(const XnVHandPointContext* cxt) {
		// positions are kept in projective coordinates, since they are only used for drawing

		XnPoint3D UserCoM;
		XnPoint3D HandPosition = cxt->ptPosition;

		XnUserID aUsers[15];
		XnUInt16 nUsers = 15;
		XnUserID trackedUser = 0;

		XnPoint3D projectiveHandPoint;
		m_DepthGenerator.ConvertRealWorldToProjective(1, &HandPosition, &projectiveHandPoint);

		g_UserGenerator.GetUsers(aUsers, nUsers);

		float dist = 10000000;
		int closestUser = -1;

		for (int i = 0; i < nUsers; ++i) {

			xn::SceneMetaData sceneMetaData;
			g_UserGenerator.GetUserPixels(aUsers[i], sceneMetaData);

			g_UserGenerator.GetCoM(aUsers[i], UserCoM);

			float distL = sqrt((projectiveHandPoint.X-UserCoM.X)*(projectiveHandPoint.X-UserCoM.X)
					+(projectiveHandPoint.Y-UserCoM.Y)*(projectiveHandPoint.Y-UserCoM.Y)
					+(projectiveHandPoint.Z-UserCoM.Z)*(projectiveHandPoint.Z-UserCoM.Z));

			if (distL < dist){

				dist = distL;
				closestUser = aUsers[i];

			}

			const XnLabel *label = sceneMetaData.Data();

			int labelPoint = (int)HandPosition.X + HandPosition.Y * sceneMetaData.XRes();

			if (aUsers[i] == label[labelPoint]){
				trackedUser = aUsers[i];
				//printf("User Waving: %d\n",trackedUser);
			}
		}

		if (closestUser!=-1){
			//printf("Closest User Waving: %d\n",closestUser);
			trackedUser = closestUser;
		}

		if (bShouldPrint && trackedUser>=1){

			g_UserGenerator.GetCoM(trackedUser, UserCoM);

			char strLabel[100] = "";
			sprintf(strLabel, "Waving !!");
			cv::putText(image, strLabel, cv::Point(projectiveHandPoint.X,projectiveHandPoint.Y), 0, 0.5, cvScalar(0, 0, 255), 1, 8);

			bender_msgs::WaveData UserWaveData;

			UserWaveData.X = UserCoM.X;
			UserWaveData.Y = UserCoM.Y;
			UserWaveData.Z = UserCoM.Z;
			UserWaveData.Distance = sqrt(pow(UserCoM.X,2)+pow(UserCoM.Z,2));
			UserWaveData.Theta = atan(UserCoM.X/UserCoM.Z)*180/PI;
			UserWaveData.ID = trackedUser;

			if (UserWaveData.Distance > 0)
				WaveUserData_publisher.publish(UserWaveData);
		}
		// Add new position to the history buffer
		m_History[cxt->nID].push_front(ptProjective);
		// Keep size of history buffer
		if (m_History[cxt->nID].size() > m_nHistorySize)
			m_History[cxt->nID].pop_back();
		bShouldPrint = true; //switch

	}


	// Handle a new message.
	// Calls other callbacks for each point, then draw the depth map (if needed) and the points
	void Update(XnVMessage* pMessage) {
		XnVPointControl::Update(pMessage);
		if (m_bDrawDM)
		{
			xn::DepthMetaData depthMD;
			m_DepthGenerator.GetMetaData(depthMD);
		}



	}
	// Change whether or not to draw the depth map
	void SetDepthMap(XnBool bDrawDM)
	{
		m_bDrawDM = bDrawDM;
	}

protected:
	// Source of the depth map
	xn::DepthGenerator m_DepthGenerator;

	xn::UserGenerator g_UserGenerator;


	// position per hand
	XnUInt32 m_nHistorySize;
	// previous positions per hand
	std::map<XnUInt32, std::list<XnPoint3D> > m_History;

	XnBool m_bDrawDM;
	XnBool m_bFrameID;
	XnFloat* m_pfPositionBuffer;
};
*/

class NIdetector {

public:

	NIdetector(std::string name);
	virtual ~NIdetector();

	void getUserCoMs(std::vector<geometry_msgs::PointStamped>& users);
	void publishUsers();
	void computeDepthHistogram(xn::DepthMetaData &depth_metadata, float *depth_histogram, int hist_size);
	void equalizeDepthHistogram(float *histogram, float *equalized, int hist_size);
	void drawTrackedUsers(xn::SceneMetaData &scene_metadata, xn::DepthMetaData &depth_metadata, float *histogram);
	void drawUserIds();
	void spinOnce();
	void niSetup();
	void shutdown();
	bool saveImageService(bender_srvs::String::Request &req, bender_srvs::String::Response &res);

	static XnUInt32 n_colors;
	static XnFloat user_colors[][3];

	// images
	cv::Mat image_users;
	//cv::Mat image;

	// generators
	xn::UserGenerator    _ni_user_generator;
	xn::DepthGenerator   _ni_depth_generator;
	xn::HandsGenerator   _ni_hands_generator;
	xn::GestureGenerator _ni_gesture_generator;
	xn::ImageGenerator   _ni_image_generator;

	// user
	bool need_user_pose;
	XnChar user_str_pose[20];

private:

	std::string _name;
	bool draw_background;
	bool _publish_images;
	float _max_person_CoM_height;
	float _min_person_CoM_height;
	std::string _ni_config_filename;

	// user tracking
	float _depth_histogram[MAX_DEPTH];
	float _depth_histogram_equalized[MAX_DEPTH];

	// - - - - - ROS - - - - -

	// node handler
	ros::NodeHandle priv;

	// publishers
	ros::Publisher _detection_pub;
	ros::Publisher _detection_markers_pub;
	image_transport::Publisher _image_pub;

	// servers
	ros::ServiceServer _save_image_server;

	// subscribers (listeners)
	std::string _kinect_frame_id;
	std::string _frame_out;
	tf::TransformListener _tf_listener;


	// - - - - OpenNI - - - - -

	// holds NI nodes graph
	xn::Context _ni_context;

	// generator data
	xn::DepthMetaData depth_metadata;
	xn::ImageMetaData image_metadata;
	xn::SceneMetaData scene_metadata;

	// hands
	/*
	XnVSessionManager* _ni_hands_session_manager;
	XnVFlowRouter* _ni_hands_flow_router;
	XnVPointDrawer* g_pDrawer;*/

	// - - - - - - - - - - - - - - NI CALLBACKS - - - - - - - - - - - - - - - -

	static void XN_CALLBACK_TYPE niUser_newUser (xn::UserGenerator& generator, XnUserID id, void* pCookie) {
		ROS_INFO_STREAM("New User, with id:" << id);

		NIdetector* node = (NIdetector*)pCookie;
		if (node->need_user_pose) {
			node->_ni_user_generator.GetPoseDetectionCap().StartPoseDetection(node->user_str_pose, id);
		} else {
			node->_ni_user_generator.GetSkeletonCap().RequestCalibration(id, TRUE);
		}
	}

	static void XN_CALLBACK_TYPE niUser_lostUser(xn::UserGenerator& generator, XnUserID id, void* pCookie) {
		ROS_INFO_STREAM("Lost User, id:" << id);
	}

	static void XN_CALLBACK_TYPE niHands_sessionStarting(const XnPoint3D& ptPosition, void* UserCxt) {
		ROS_INFO_STREAM("New Hands Session");
		//printf("Session start: (%f,%f,%f)\n", ptPosition.X, ptPosition.Y,ptPosition.Z);
		//g_SessionState = IN_SESSION;
	}

	static void XN_CALLBACK_TYPE niHands_sessionEnding(void* UserCxt) {
		ROS_INFO_STREAM("Hands Session Ending");
		//g_SessionState = NOT_IN_SESSION;
	}

	static void XN_CALLBACK_TYPE niHands_focusProgress(const XnChar* strFocus, const XnPoint3D &ptPosition, XnFloat fProgress, void* UserCxt) {
		ROS_INFO_STREAM("Hand waving in progress: (x,y,z)=(" << ptPosition.X << ", " << ptPosition.Y << ", "<<ptPosition.Z << ")");
	}

	static void XN_CALLBACK_TYPE niHands_noHands(void* UserCxt) {
		ROS_INFO_STREAM("No hands");
	}
	//void XN_CALLBACK_TYPE niGesture_progressHandler(xn::GestureGenerator &generator, const XnChar* strGesture, const XnPoint3D* pPosition, XnFloat fProgress, void* pCookie);
	//printf("Gesture %s progress: %f (%f,%f,%f)\n", strGesture, fProgress, pPosition->X, pPosition->Y, pPosition->Z);
};

XnUInt32 NIdetector::n_colors = 11;
XnFloat NIdetector::user_colors[][3] = {
	{0.0, 1.0, 1.0}, { 0.0, 0.0, 1.0}, { 0.0, 1.0, 0.0},
	{1.0, 1.0, 0.0}, { 1.0, 0.0, 0.0}, { 1.0, 0.5, 0.0},
	{0.5, 1.0, 0.0}, { 0.0, 0.5, 1.0}, { 0.5, 0.0, 1.0},
	{1.0, 1.0, 0.5}, { 1.0, 1.0, 1.0}
};


} /* namespace bender_follow_me */

#endif /* NIDETECTOR_HPP_ */


/*
 * SearchFace.h
 *
 *  Created on: Jan 20, 2014
 *      Author: matias.pavez.b@gmail.com
 */

#ifndef SEARCHFACE_H_
#define SEARCHFACE_H_

// C, C++
#include <vector>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// Bender
#include <bender_utils/ParameterServerWrapper.h>
#include <bender_msgs/Emotion.h>
#include <bender_srvs/SearchPerson.h>
#include <bender_srvs/FaceInfo.h>
#include <bender_srvs/FaceRecognition.h>

namespace bender_macros {

class SearchFace {

private:

	std::string _name;

	unsigned int _current_head_pos_index;
	unsigned int _current_face_index;
	int _last_head_angle;

	// - - - - - P a r a m e t e r s - - - - - - -
	int _camera_width;
	int _reco_attempts;
	double _reco_distance_th;
	double _reco_th;
	double _fov_h;
	std::string _camera_tf;
	std::vector<int> _head_angles;
	std::vector<double> _camera_model;


	ros::Publisher _face_pub;

	// - - - - - - - - S e r v i c e   C l i e n t s  - - - - - - - - -
	ros::ServiceClient _face_detection_client;
  	ros::ServiceClient _face_recognize_client;


  	// - - - - - - - - - - -  S e r v i c e s  - - - - - - - - - - - -
	ros::ServiceServer _search_service;




public:
	SearchFace(std::string name);
	virtual ~SearchFace();

private:

	void extractPose(int face_width, int face_offset, geometry_msgs::PoseStamped & person);
	bool getNextFaceIndex(unsigned int total_faces, unsigned int & index);
	bool getNextHeadPosition(unsigned int & angle);
	void rotateHead(int angle);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// - - - - - - - - - - - - - - - - - - - - -  S e r v i c e s  - - - - - - - - - - - - - - - - - - - - - - - - - -

	bool search(bender_srvs::SearchPerson::Request &req, bender_srvs::SearchPerson::Response &res);

};

} /* namespace bender_macros */
#endif /* SEARCHFACE_H_ */

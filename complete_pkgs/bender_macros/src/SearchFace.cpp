/*
 * SearchFace.cpp
 *
 *  Created on: Jan 20, 2014
 *      Author: matias.pavez.b@gmail.com
 */

#include "SearchFace.h"

// TODO: use a tf listener to transform pose to /map frame

namespace bender_macros {

SearchFace::SearchFace(std::string name):_name(name) {

	ros::NodeHandle priv("~");

	// control variables
	_current_head_pos_index = 0;
	_current_face_index = 0;
	_last_head_angle = 0;


    // - - - - - - - P A R A M E T E R   S E R V E R - - - - - - - - -
	bender_utils::ParameterServerWrapper psw;
    psw.getParameter("reco_attempts",_reco_attempts,10);
    psw.getParameter("reco_th",_reco_th,70.0);
    psw.getParameter("reco_distance_th",_reco_distance_th,110.0);
    psw.getParameter("fov_h",_fov_h,60.0);
    psw.getParameter("camera_width",_camera_width,640);
    psw.getParameter("camera_tf",_camera_tf,"/bender/sensors/camera_right_eye_link");
    _fov_h = _fov_h*M_PI/180.0;

    // camera model: y = a1*x^n + a2*x^(n-1) + ... + an
    std::vector<double> default_model;
	default_model.push_back(2.1964e-09);
	default_model.push_back(-2.1624e-06);
	default_model.push_back(7.5421e-04);
	default_model.push_back(-0.11278);
	default_model.push_back(6.9305);
    psw.getParameter("camera_model",_camera_model,default_model);

	// head positions [degrees]
    std::vector<int> default_positions;
	default_positions.push_back(35);
	default_positions.push_back(0);
	default_positions.push_back(-35);
	psw.getParameter("head_angles",_head_angles,default_positions);

	// - - - - - - - - P u b l i s h e r s - - - - - - - - - - - - - - -
	_face_pub = priv.advertise<bender_msgs::Emotion>("/bender/face/head",1, this);


	// - - - - - - - - S e r v i c e   C l i e n t s  - - - - - - - - -
	// vision
	_face_detection_client = priv.serviceClient<bender_srvs::FaceInfo>("/bender/vision/face_detector/detect_face");
	_face_recognize_client = priv.serviceClient<bender_srvs::FaceRecognition>("/bender/vision/face_recognizer/recognize_img");


	// - - - - - - - - w a i t   f o r   s e r v i c e s  - - - - - - - - -
	// vision
	while ( ros::ok() && !_face_detection_client.waitForExistence(ros::Duration(3.0)) );
	while ( ros::ok() && !_face_recognize_client.waitForExistence(ros::Duration(3.0)) );

	// - - - - - - - - - - - - - - S E R V I C E S - - - - - - - - - - - - - -
	_search_service = priv.advertiseService("/bender/macros/search_face", &SearchFace::search,this);
}

SearchFace::~SearchFace() {
}

bool SearchFace::search(bender_srvs::SearchPerson::Request &req, bender_srvs::SearchPerson::Response &res) {

	unsigned int angle = 0;
	unsigned int face_index = 0;
	float reco_rate;
	bender_srvs::FaceRecognition _reco_srv;
	bender_srvs::FaceInfo detection_srv;

	while ( getNextHeadPosition(angle) ) {

		rotateHead(angle);

		detection_srv = bender_srvs::FaceInfo();
		detection_srv.request.return_images = true;
		_face_detection_client.call(detection_srv);

		if (detection_srv.response.n_faces == 0) {
			continue;
		}
		ROS_INFO_STREAM("[" << _name << "]: Number of faces detected: " << detection_srv.response.n_faces);


		while( getNextFaceIndex(detection_srv.response.faces.size(),face_index) ) {

			ROS_INFO_STREAM("[" << _name << "]: Working on face: " << face_index);

			_reco_srv.request.face = detection_srv.response.faces[face_index];
			_face_recognize_client.call(_reco_srv);

			if (!_reco_srv.response.face_index == req.id) {
				continue;
			}
			reco_rate = 100.0/_reco_attempts;

			for (int k=0; k<_reco_attempts-1; k++) {

				_face_recognize_client.call(_reco_srv);

				if (_reco_srv.response.face_index == req.id && _reco_srv.response.distance < _reco_distance_th) {
					reco_rate += 100.0/_reco_attempts;
				}
			}

			if (reco_rate >= _reco_th) {

				ROS_INFO_STREAM("[" << _name << "]: " << " face " << req.id << " found, with accuracy: " << reco_rate << "%");

				extractPose(
						detection_srv.response.BBoxes[face_index].width,
						detection_srv.response.BBoxes[face_index].x,
						res.person
				);

				rotateHead(0);
				res.found = true;
				return true;
			}
		}
	}

	ROS_INFO_STREAM("[" << _name << "]: " << " face not found");
	rotateHead(0);
	res.found = false;

	return true;
}

void SearchFace::extractPose(int face_width, int face_offset, geometry_msgs::PoseStamped & person) {

	std::vector<double>::iterator it;
	double depth = 0;

	// model: y = a*x^n + b*x^(n-1) + ... + e
	int power = _camera_model.size()-1;
	for ( it = _camera_model.begin(); it < _camera_model.end(); it++,power--) {
		depth += (*it)*pow(face_width,power);
	}

	double resolution = 2.0*depth*tan(_fov_h/2.0)/_camera_width;

	person.header.stamp = ros::Time::now();
	person.header.frame_id = _camera_tf;
	person.pose.position.x = depth;
	person.pose.position.y = - resolution*(
			face_offset + face_width - _camera_width/2.0
	);
	person.pose.position.z = 0.0;
	person.pose.orientation.w = 1.0;

}

bool SearchFace::getNextFaceIndex(unsigned int total_faces, unsigned int & index) {

	if (_current_face_index >= total_faces ) {
		_current_face_index = 0;
		return false;
	}

	index = _current_face_index;
	_current_face_index++;

	return true;

}

bool SearchFace::getNextHeadPosition(unsigned int &angle) {

	if (_current_head_pos_index >= _head_angles.size() ) {
		_current_head_pos_index = 0;
		return false;
	}

	angle = _head_angles[_current_head_pos_index];
	_current_head_pos_index++;

	return true;
}

void SearchFace::rotateHead(int angle) {

	bender_msgs::Emotion emotion;
	int delta;
    emotion.Order = "MoveX";
    emotion.X = angle;

    delta = fabsf(angle - _last_head_angle);
    _last_head_angle = angle;

    ROS_INFO_STREAM("[" << _name << "]: Rotating Head to " << angle << " degrees");

	_face_pub.publish(emotion);
	ros::Duration(delta*3.0/40.0).sleep();
}

} /* namespace bender_macros */


int main(int argc, char** argv) {

	ros::init(argc, argv, "search_face");

	boost::scoped_ptr<bender_macros::SearchFace> node(
			new bender_macros::SearchFace(ros::this_node::getName())
	);

	ros::spin();

	ROS_INFO("Quitting ... \n");
	return 0;
}

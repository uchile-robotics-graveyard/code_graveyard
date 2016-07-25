/*
 * EnrollMacros.cpp
 *
 *  Created on: January 2014
 *      Author: matias.pavez.b@gmail.com
 */

#include "EnrollFace.h"

namespace bender_macros {

EnrollFace::EnrollFace(std::string name): _name(name) {

	// - - - v a r i a b l e s - - -
	priv = ros::NodeHandle("~");
	_is_talking = false;

	// - - - - - - - - S e r v i c e   C l i e n t s  - - - - - - - - -

    // speech
    _speech_synthesizer_client = priv.serviceClient<bender_srvs::synthesize>("/speech_synthesizer/synthesize");

    // vision
    _face_detection_client = priv.serviceClient<bender_srvs::FaceInfo>("/bender_vision/face_detection/detect");
    _face_add_client = priv.serviceClient<bender_srvs::FaceRecognition>("/bender_vision/face_recognition/add");
    _face_load_images_client = priv.serviceClient<bender_srvs::FaceRecognition>("/bender_vision/face_recognition/read_im_db");


	// - - - - - - - - w a i t   f o r   s e r v i c e s  - - - - - - - - -

    // speech
    while ( ros::ok() && !_speech_synthesizer_client.waitForExistence(ros::Duration(3.0)) );

    // vision
    while ( ros::ok() && !_face_detection_client.waitForExistence(ros::Duration(3.0)) );
    while ( ros::ok() && !_face_add_client.waitForExistence(ros::Duration(3.0)) );
    while ( ros::ok() && !_face_load_images_client.waitForExistence(ros::Duration(3.0)) );

    // load image database
    _face_load_images_client.call(_reco_srv);

	// - - - - - - - - - - -  S e r v i c e s  - - - - - - - - - - - -
    _enroll_face_server = priv.advertiseService("enroll",&EnrollFace::enrollFace,this);


    ROS_INFO_STREAM("[" << _name << "]: Running . . .");
}

EnrollFace::~EnrollFace() {
}

bool EnrollFace::enrollFace(bender_srvs::ID::Request &req, bender_srvs::ID::Response &res) {

	int subject_index = req.ID; // TODO: use getNextFreeID() service !!
	int total_images = 3; // TODO: service parameter
	int correct_images = 0;

	_detection_srv.request.return_images = false;
	_reco_srv.request.add_face_index = subject_index;

	talk("Please, look at my right eye");

	_face_detection_client.call(_detection_srv);
	while (_detection_srv.response.n_faces == 0) {

		talk("I dont' see anyone");
		ros::Duration(1).sleep();
		_face_detection_client.call(_detection_srv);
	}

	while(correct_images < total_images) {

		if (!_face_add_client.call(_reco_srv)) {
			ROS_WARN_STREAM("[" << _name << "]: Failed to save image, will retry");
		}
		correct_images++;
		ROS_INFO_STREAM("[" << _name << "] Enroll Faces: image " << correct_images << " from " << total_images << " ok");
	}

	talk("okey, i am ready");

	res.ID = subject_index;

	return true;
}

void EnrollFace::talk(std::string sentence) {

	ROS_INFO_STREAM("[" << _name << "]: Talking:" << sentence);

	_synt_srv.request.text = sentence;
	_speech_synthesizer_client.call(_synt_srv);

	double timeout = 2.0;
	double elapsed_time = 0.0;
	_is_talking = false;
	while (!_is_talking) {
		ros::Duration(0.5).sleep();
		priv.getParam("/bender/talking",_is_talking);
		elapsed_time+=1.0;

		if (elapsed_time >= timeout) {
			elapsed_time = 0;
			break;
		}
	}
	while(_is_talking) {
		ros::Duration(0.5).sleep();
		priv.getParam("/bender/talking",_is_talking);
		elapsed_time +=1.0;

		if (elapsed_time >= timeout) {
			elapsed_time = 0;
			break;
		}
	}
}

} /* namespace bender_macros */

int main(int argc, char** argv) {

	ros::init(argc, argv, "enroll_macros");

	boost::scoped_ptr<bender_macros::EnrollFace> node(
			new bender_macros::EnrollFace(ros::this_node::getName())
	);

	ros::spin();

	ROS_INFO("Quitting ... \n");
	return 0;
}

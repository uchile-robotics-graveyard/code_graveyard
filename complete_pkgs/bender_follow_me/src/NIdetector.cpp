/*
 * NIdetector.cpp
 *
 *  Created on: 28-10-2014
 *      Author: matias
 */

#include <bender_follow_me/NIdetector.hpp>

/*
 * TODO: demasiados falsos positivos!, especialmente muebles/cosas planas
 */

namespace bender_follow_me {

NIdetector::NIdetector(std::string name): _name(name) {

	priv = ros::NodeHandle("~");

	// parameters
	bender_config::ParameterServerWrapper psw;
	psw.getParameter("draw_background", draw_background, true);
	psw.getParameter("publish_images" , _publish_images , true);
	psw.getParameter("max_person_CoM_height", _max_person_CoM_height, 1.4);
	psw.getParameter("min_person_CoM_height", _min_person_CoM_height, 0.8);
	psw.getParameter("tf_frame_out", _frame_out, "/bender/base_link");
	psw.getParameter("kinect_frame_id", _kinect_frame_id, "/bender/sensors/kinect_waist_link");
	psw.getParameter("ni_config_filename", _ni_config_filename, "SamplesConfig.xml");

	need_user_pose = false;
	// TODO: MAX_DEPTH as parameter

	// initialize images
	image_users = cv::Mat(cv::Size(640,480), CV_8UC3, cv::Scalar(0,0,0));

	// - - -  ROS  - - -

	// publishers
	_detection_pub = priv.advertise<bender_follow_me::BodyDetections>("ni_detections",1, this);
	_detection_markers_pub = priv.advertise<visualization_msgs::MarkerArray>("detection_markers",1, this);

	// advertise topic if necessary
	if (_publish_images) {
		image_transport::ImageTransport it(priv);
		_image_pub = it.advertise("depth_detections_image", 1);
	}

	// servers
	_save_image_server = priv.advertiseService("save_image", &NIdetector::saveImageService, this);


	// setup NI nodes
	niSetup();

	// [important!] wait for timer initialization!
	while (ros::ok() && ros::Time::now().isZero());

	ros::Duration(1.0).sleep();

	ROS_INFO("Working");
}

NIdetector::~NIdetector() {
}


bool NIdetector::saveImageService(bender_srvs::String::Request &req, bender_srvs::String::Response &res) {

	if (!_ni_image_generator.IsValid()) {
		return false;
	}

	std::string filename = req.data;
	try {
		_ni_context.WaitOneUpdateAll(_ni_image_generator);
		_ni_image_generator.GetMetaData(image_metadata);
		const XnRGB24Pixel* ni_image_ptr = _ni_image_generator.GetRGB24ImageMap();
		cv::Mat rgb_image = cv::Mat(cv::Size(640,480), CV_8UC3);
		XnUInt32 image_cols = image_metadata.XRes();
		XnUInt32 image_rows = image_metadata.YRes();
		for (uint i=0; i<image_rows; ++i) {
			cv::Vec3b* cv_image_ptr = rgb_image.ptr<cv::Vec3b>(i);
			for (uint j=0; j<image_cols; ++j) {
				cv_image_ptr[j][0] = (*ni_image_ptr).nBlue;
				cv_image_ptr[j][1] = (*ni_image_ptr).nGreen;
				cv_image_ptr[j][2] = (*ni_image_ptr).nRed;
				ni_image_ptr++;
			}
		}
		//cv::imshow("rgb", rgb_image);
		//cv::waitKey(10);

		// prepare filename
		std::string pkg_path = ros::package::getPath("bender_follow_me");
		std::string ext(".jpg");
		if ( filename.find(ext) == std::string::npos) {
			filename += ext;
		}
		filename = pkg_path + "/" + filename;

		// write parameters
		std::vector<int> write_params;
		write_params.push_back(CV_IMWRITE_JPEG_QUALITY);
		write_params.push_back(100);

		// write
		cv::imwrite(filename, rgb_image, write_params);

	} catch (std::exception &e) {
		ROS_WARN_STREAM("There was a problem while trying to retrieve the RGB image:"
				<< e.what()
		);
		return false;
	}

	res.data = filename;
	ROS_INFO_STREAM("Saved RGB image to: " << filename);
	return true;
}

void NIdetector::niSetup() {

	ROS_INFO("configuring openni . . .");

	std::string pkg_path = ros::package::getPath("bender_follow_me");
	std::string file_path = pkg_path + "/config/openni/" + _ni_config_filename;

	// - - - Initialize from XML - - -

	// check file existence
	XnBool file_exist;
	xnOSDoesFileExist(file_path.c_str(), &file_exist);
	if (!file_exist) {
		ROS_ERROR_STREAM("Could not find OpenNI configuration file: '" << file_path << "'.  Aborting.\n");
		exit(-1);
	}
	ROS_INFO_STREAM("Reading OpenNI configuration from: '" << file_path << "'");

	// initialization
	XnStatus ni_ret_val = _ni_context.InitFromXmlFile(file_path.c_str(), NULL);
	if (ni_ret_val != XN_STATUS_OK) {
		ROS_ERROR_STREAM("Failed to initialize OpenNI: " << xnGetStatusString(ni_ret_val));
	    exit(-1);
	}

	// retrieve required nodes
	xn::NodeInfoList node_list;
	ni_ret_val = _ni_context.EnumerateExistingNodes(node_list);
	if (ni_ret_val != XN_STATUS_OK) {
		ROS_ERROR_STREAM("Bad configuration syntax.\nFailed to initialize OpenNI: " << xnGetStatusString(ni_ret_val));
		exit(-1);
	}

	// TODO: puede fallar al inicio... esperar a que se conecte el dispositivo
	ROS_INFO_STREAM("Analyzing nodes:");
	int cnt = 1;
	for (xn::NodeInfoList::Iterator it = node_list.Begin(); it != node_list.End(); ++it) {
		ROS_INFO_STREAM("node (" << cnt << ") name: " << (*it).GetInstanceName());
		cnt++;

		switch ((*it).GetDescription().Type) {

			case XN_NODE_TYPE_DEPTH: {

				// get generator
				(*it).GetInstance(_ni_depth_generator);
				break;
			}

			case XN_NODE_TYPE_HANDS: {

				/*
				// get generator
				(*it).GetInstance(_ni_hands_generator);

				// configure generator callbacks
				_ni_hands_session_manager = new XnVSessionManager;
				_ni_hands_session_manager->Initialize(&_ni_context, "Wave", "RaiseHand");
				_ni_hands_session_manager->RegisterSession(NULL, niHands_sessionStarting, niHands_sessionEnding, niHands_focusProgress);

				g_pDrawer = new XnVPointDrawer(20, _ni_depth_generator, _ni_user_generator);
				_ni_hands_flow_router = new XnVFlowRouter;
				_ni_hands_flow_router->SetActive(g_pDrawer);

				_ni_hands_session_manager->AddListener(_ni_hands_flow_router);

				g_pDrawer->RegisterNoPoints(NULL, niHands_noHands);
				g_pDrawer->SetDepthMap(g_bDrawDepthMap);
				*/
				break;
			}

			case XN_NODE_TYPE_IMAGE: {
				// get generator
				(*it).GetInstance(_ni_image_generator);
				break;
			}
			//case XN_NODE_TYPE_GESTURE: {
			//	break;
			//}
			case XN_NODE_TYPE_USER: {

				XnCallbackHandle user_callbacks_handler;
				(*it).GetInstance(_ni_user_generator);
				_ni_user_generator.RegisterUserCallbacks(niUser_newUser, niUser_lostUser, this, user_callbacks_handler);


				/*
				// User leaves the scene (but is still being tracked)

				XnCallbackHandle user_cb_exit_handle;

				user_generator.RegisterToUserExit(
				User_ExitUser
				,this
				,user_cb_exit_handle
				);*/

				break;
			}
		}

	}

	// Make it start generating data
	ni_ret_val = _ni_context.StartGeneratingAll();
	if (ni_ret_val != XN_STATUS_OK) {
		ROS_ERROR_STREAM("Failed to start generating NI data: " << xnGetStatusString(ni_ret_val));
		exit(-1);
	}

	ROS_INFO("configuration OK.");

}

void NIdetector::shutdown() {

	_ni_context.Release();
}

void NIdetector::computeDepthHistogram(xn::DepthMetaData &depth_metadata, float *depth_histogram, int hist_size) {

	const XnDepthPixel* depth_map_ptr = depth_metadata.Data();
	XnUInt16 depth_cols = depth_metadata.XRes();
	XnUInt16 depth_rows = depth_metadata.YRes();

	uint curr_depth;
	memset(depth_histogram, 0, hist_size*sizeof(float));
	for ( uint i = 0; i < depth_rows; ++i ) {
		for ( uint j = 0; j < depth_cols; ++j ) {

			curr_depth = *depth_map_ptr;
			if ( 0 < curr_depth && curr_depth < hist_size) {

				depth_histogram[curr_depth]++;
			}
			depth_map_ptr++;
		}
	}

}

void NIdetector::equalizeDepthHistogram(float *histogram, float *equalized, int hist_size) {

	uint n_valid_depth_points;

	// prepare memory
	//memset(equalized, 0, hist_size*sizeof(float));

	// compute accumulative histogram
	equalized[0] = histogram[0];
	for ( uint idx = 1; idx < hist_size; ++idx ) {
		equalized[idx] = equalized[idx-1] + histogram[idx];
	}
	n_valid_depth_points = equalized[hist_size-1];

	// normalize & invert colors
	if ( n_valid_depth_points > 0) {
		for ( uint idx = 1; idx < hist_size; ++idx ) {
			equalized[idx] = (float)(256 * (1.0f - (equalized[idx] / n_valid_depth_points)));
		}
	}

}

void NIdetector::drawTrackedUsers(xn::SceneMetaData &scene_metadata, xn::DepthMetaData &depth_metadata, float *histogram) {

	// get initial depth pointers again
	const XnLabel* scene_map_ptr = scene_metadata.Data();
	const XnDepthPixel* depth_map_ptr = depth_metadata.Data();

	XnUInt16 depth_cols = depth_metadata.XRes();
	XnUInt16 depth_rows = depth_metadata.YRes();
	uint curr_depth;
	uint curr_hist_value;
	for (int i=0; i<depth_rows; ++i) {

		cv::Vec3b* cv_image_ptr = image_users.ptr<cv::Vec3b>(i);
		for (int j=0; j<depth_cols; ++j) {

			// prepare cv image
			cv_image_ptr[j][0] = 0;
			cv_image_ptr[j][1] = 0;
			cv_image_ptr[j][2] = 0;

			// draw background or users
			if ( draw_background || (*scene_map_ptr) != 0) {

				curr_depth = *depth_map_ptr;
				XnLabel label = *scene_map_ptr;
				XnUInt32 color_idx = label % (NIdetector::n_colors-1);

				// select fixed color for background
				if (label == 0) { color_idx = NIdetector::n_colors-1; }

				// draw user
				if (curr_depth > 0) {
					curr_hist_value = (uint) histogram[curr_depth];
					cv_image_ptr[j][0] = curr_hist_value * NIdetector::user_colors[color_idx][0];
					cv_image_ptr[j][1] = curr_hist_value * NIdetector::user_colors[color_idx][1];
					cv_image_ptr[j][2] = curr_hist_value * NIdetector::user_colors[color_idx][2];
				}
			}

			// update positions
			scene_map_ptr++;
			depth_map_ptr++;
		}
	}
}

void NIdetector::drawUserIds() {

	// - - - get user info - - -
	XnUserID user_ids[10];
	XnUInt16 n_users = 10;
	_ni_user_generator.GetUsers(user_ids, n_users);

	double user_d;
	double user_theta;
	XnPoint3D point3D, point2D;
	for( int i=0; i<n_users; ++i ) {

		// get CoM: Center of Mass
		_ni_user_generator.GetCoM(user_ids[i], point3D);

		// transform to image coordinates
		_ni_depth_generator.ConvertRealWorldToProjective(1, &point3D, &point2D);

		// compute position
		user_d = sqrt(point3D.X*point3D.X + point3D.Z*point3D.Z)/1000.0;
		user_theta = -atan(point3D.X/point3D.Z)*180.0/M_PI;

		std::stringstream info_1_ss, info_2_ss;
		info_1_ss << "user id:" << user_ids[i];
		info_2_ss << (float)((int)(user_d*100.0))/100.0 << "[m], " << (int)user_theta << "[deg]";

		cv::putText(image_users, info_1_ss.str(), cv::Point(point2D.X,point2D.Y)   , 0, 0.8, cv::Scalar(255, 255, 0), 1, 8);
		cv::putText(image_users, info_2_ss.str(), cv::Point(point2D.X,point2D.Y+20), 0, 0.5, cv::Scalar(255, 255, 0), 1, 8);
	}
}

void NIdetector::getUserCoMs(std::vector<geometry_msgs::PointStamped>& users) {

	ros::Time detection_time = ros::Time::now();
	std::vector<geometry_msgs::PointStamped> ni_CoMs;
	geometry_msgs::PointStamped user;
	geometry_msgs::PointStamped user_tf;

	// - - - get NI user info - - -
	XnUserID user_ids[10];
	XnUInt16 n_users = 10;
	_ni_user_generator.GetUsers(user_ids, n_users);

	XnPoint3D point3D;
	for( int i=0; i<n_users; ++i ) {

		// get CoM: Center of Mass
		_ni_user_generator.GetCoM(user_ids[i], point3D);

		if ( point3D.X == point3D.Y && point3D.Y == point3D.Z && point3D.Z == 0 ) {
			// user not in scene
			continue;
		}


		user.header.seq = user_ids[i];    // user id saved as header seq!!!

		user.header.stamp = detection_time;
		user.header.frame_id = _kinect_frame_id;
		user.point.x =  point3D.Z/1000.0;
		user.point.y = -point3D.X/1000.0;
		user.point.z =  point3D.Y/1000.0;
		ni_CoMs.push_back(user);
	}

	// - - - transform user poses - - -
	users.clear();

	// transform isn't needed
	if (_kinect_frame_id == _frame_out) {

		std::vector<geometry_msgs::PointStamped>::const_iterator it;
		for (it = ni_CoMs.begin(); it != ni_CoMs.end(); ++it) {

			// invalid users! ... too high or too low
			if ( it->point.z < _min_person_CoM_height
				|| _max_person_CoM_height < it->point.z) {
				continue;
			}

			// add valid user
			users.push_back(*it);
		}
		return;
	}

	// transform each center of mass
	try {
		_tf_listener.waitForTransform(_frame_out, _kinect_frame_id, detection_time, ros::Duration(2.0));

		std::vector<geometry_msgs::PointStamped>::const_iterator it;
		for (it = ni_CoMs.begin(); it != ni_CoMs.end(); ++it) {

			user = *it;
			_tf_listener.transformPoint(_frame_out, user, user_tf);


			// invalid users! ... too high or too low
			if ( user_tf.point.z < _min_person_CoM_height
				|| _max_person_CoM_height < user_tf.point.z) {
				continue;
			}

			// add valid user
			user_tf.header.seq = it->header.seq; // copy user id (previously saved on this field)!!!
			users.push_back(user_tf);
		}

	} catch (tf::ConnectivityException &e) {
		ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
		users.clear();
		return;

	} catch (tf::TransformException &e) {
		ROS_WARN_STREAM("Transform Exception: " << e.what());
		users.clear();
		return;
	}
}

void NIdetector::publishUsers() {

	std::vector<geometry_msgs::PointStamped> users;

	// - - get center of mass on output frame - -
	getUserCoMs(users);
	if ( users.empty() ) {
		return;
	}

	// - - publish detection data             - -
	BodyDetections ni_msg;
	ni_msg.header.frame_id = _frame_out;
	ni_msg.header.stamp = users[0].header.stamp;
	ni_msg.source = "NI"; // XXX: todo

	std::vector<geometry_msgs::PointStamped>::const_iterator it;
	for (it = users.begin(); it != users.end(); ++it) {

		SingleBodyDetection single_detection;
		single_detection.id = it->header.seq;	// user id was saved on this field!
		single_detection.x = it->point.x;
		single_detection.y = it->point.y;
		single_detection.z = it->point.z;
		single_detection.heading = 0; 			// cannot determine from ni detection

		ni_msg.detections.push_back(single_detection);
	}
	_detection_pub.publish(ni_msg);

	// - - publish visualization messages     - -
	if ( _detection_markers_pub.getNumSubscribers() > 0 ) {

		// visualization step
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		visualization_msgs::MarkerArray marker_array;
		visualization_msgs::Marker marker;
		visualization_msgs::Marker marker_text;

		marker.ns = "ni_detection";
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::MODIFY;
		marker.lifetime = ros::Duration(0.2);
		marker.header = ni_msg.header;

		marker_text.ns = marker.ns + "_text";
		marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker_text.action = visualization_msgs::Marker::MODIFY;
		marker_text.lifetime = ros::Duration(0.2);
		marker_text.header = ni_msg.header;

		// size
		marker.scale.x = 0.30;
		marker.scale.y = 0.30;
		marker.scale.z = 0.30;

		XnUInt32 color_idx;
		for( int i=0; i<ni_msg.detections.size(); ++i ) {

			// color
			color_idx = ni_msg.detections[i].id % (NIdetector::n_colors-1);
			marker.color.b = NIdetector::user_colors[color_idx][0];
			marker.color.g = NIdetector::user_colors[color_idx][1];
			marker.color.r = NIdetector::user_colors[color_idx][2];
			marker.color.a = 1.0;

			// position
			marker.pose.position.x = ni_msg.detections[i].x;
			marker.pose.position.y = ni_msg.detections[i].y;
			marker.pose.position.z = ni_msg.detections[i].z;
			marker.pose.orientation.w = 1.0;
			marker.id = ni_msg.detections[i].id;
			marker_array.markers.push_back(marker);

			// text id
			marker_text.scale.z = 0.3;
			marker_text.color.a = 1.0;
			marker_text.color.b = 1.0;
			marker_text.color.g = 1.0;
			marker_text.color.r = 0.0;
			marker_text.pose = marker.pose;
			marker_text.pose.position.x = marker_text.pose.position.x + 0.2;
			marker_text.pose.position.y = marker_text.pose.position.y + 0.2;
			marker_text.pose.position.z = marker_text.pose.position.z + 0.2;
			std::stringstream ss; ss << "id:" << (uint)ni_msg.detections[i].id;
			marker_text.text = ss.str();
			marker_text.id = ni_msg.detections[i].id;
			marker_array.markers.push_back(marker_text);
		}

		// publish
		_detection_markers_pub.publish(marker_array);
	}

}

void NIdetector::spinOnce() {

	// Wait for new data to be available
	XnStatus ni_ret_val = _ni_context.WaitAnyUpdateAll();
	if (ni_ret_val != XN_STATUS_OK) {
		ROS_WARN_STREAM("Failed updating data: " << xnGetStatusString(ni_ret_val));
		return;
	}

	// get depth/scene data
	if (_ni_depth_generator.IsValid() ) {
		_ni_depth_generator.GetMetaData(depth_metadata);
	}
	_ni_user_generator.GetUserPixels(0, scene_metadata);

	if (_publish_images && _image_pub.getNumSubscribers() > 0 ) {

		// Histogram Equalization
		// its required in order to display the depth map in a human "readable" way
		computeDepthHistogram(depth_metadata, _depth_histogram, MAX_DEPTH);
		equalizeDepthHistogram(_depth_histogram, _depth_histogram_equalized, MAX_DEPTH);

		// draw users
		drawTrackedUsers(scene_metadata, depth_metadata, _depth_histogram_equalized);
		drawUserIds();

		// display stuff
		sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_users).toImageMsg();
		_image_pub.publish(image_msg);
		cv::waitKey(30);
	}

	// publish ROS messages
	publishUsers();


	// ??? tiene que ir ahi, no se pk
	_ni_context.WaitOneUpdateAll(_ni_user_generator);
	//_ni_hands_session_manager->Update(&_ni_context);
}

} /* namespace bender_follow_me */


int main(int argc, char **argv){

	ros::init(argc, argv, "ni_detector");

	boost::scoped_ptr<bender_follow_me::NIdetector> node(
		new bender_follow_me::NIdetector(ros::this_node::getName())
	);

	ros::Rate rate(20);
	while (ros::ok()) {

		node->spinOnce();

		ros::spinOnce();
		rate.sleep();
	}

	printf("\nQuitting... \n\n");

	// Clean-up
	node->shutdown();

	return 0;
}



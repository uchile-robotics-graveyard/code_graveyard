/*
 * BayesianFilter.hpp
 *
 *  Created on: 16-10-2014
 *      Author: matias
 */

#ifndef BAYESIANFILTER_HPP_
#define BAYESIANFILTER_HPP_

// C++
#include <string.h>
#include <vector>
#include <math.h>
#include <boost/scoped_ptr.hpp>
#include <pthread.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigenvalues>

// ROS
#include "ros/ros.h"

// ROS messages
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <bender_srvs/String.h>
#include <bender_follow_me/BodyDetections.h>
#include <bender_follow_me/SingleBodyDetection.h>
#include <bender_follow_me/TrackingState.h>
#include <bender_srvs/ID.h>

// models
#include <bender_follow_me/MotionModel.hpp>
#include <bender_follow_me/LaserModel.hpp>
//#include <bender_laser/LegDetector.hpp>

namespace bender_follow_me {

struct TrackNewData {
	ros::Time observation_time;
	geometry_msgs::Point32 position;
	float heading;
};

struct Track {

	uint id;
	MotionModel model;
	ros::Time last_update_time;

	bool has_new_data;
	TrackNewData new_data;

	Track() {
		// model =
		last_update_time = ros::Time::now();
		id = -1;
		has_new_data = false;
	};

	void initModel(float x, float y) {
		model.initializeState(x, y, ros::Time::now());
	};

	void reinitModel() {
		model.resetUncertainty();
	};

	float getTrackUncertainty() {
		//return model._Q(0,0) + model._Q(1,1);
		return 0;
	};
};

class BayesianFilter {
public:
	BayesianFilter();
	virtual ~BayesianFilter();
	void callback_detections(const bender_follow_me::BodyDetections &msg);
	void kalman_spinOnce();
	void MHT_spinOnce();

	void initializeFilter();
	bool initialization_triggered;

private:
	void associateObservations(const bender_follow_me::BodyDetections &msg, std::vector<bool> &used_observations);
	uint getEmptyId();
	static void unscentedTransform(const State &x, State &y, const state_cov_t &Q, std::vector<state_t> &Y_points, std::vector<state_t> &X_points);
	static void generateExpectedObservation(const std::vector<state_t> &Y_points, std::vector<observation_t> &Z_points, observation_t &z, observation_cov_t &Pzz, const observation_cov_t &R);
	static void kalmanCorrection(State &x, const State &y, const std::vector<state_t> &Y_points, const std::vector<observation_t> &Z_points, const observation_t z, const observation_cov_t Pzz, const observation_t z_real);
	static float normalizeRadian(float angle);
	static void createMarkerSigmaPoints(const std::vector<state_t> &S_points, const std_msgs::Header header, visualization_msgs::MarkerArray &marker_array, std::string &ns, std_msgs::ColorRGBA color0,  std_msgs::ColorRGBA color);
	bool initializeFilterService(bender_srvs::String::Request &req, bender_srvs::String::Response &res);
	bool getInitIdService(bender_srvs::ID::Request &req, bender_srvs::ID::Response &res);

private:

	std::string _name;
	std::string _frame_id;

	// publishers
	ros::Publisher _tracking_markers_pub;
	ros::Publisher _tracking_pub;

	// subscribers
	ros::Subscriber _leg_detection_sub;
	ros::Subscriber _ni_detection_sub;

	// servers
	ros::ServiceServer _initialize_server;
	ros::ServiceServer _get_init_id_server;

	ros::NodeHandle priv;

	// mutex for observation update
	pthread_mutex_t _observation_mutex;


	// - - - variables - - -
	float init_desired_distance; // init_desired_distance = 1.5; // [m] no se ve mi cabeza
	float init_min_distance;     // init_min_distance = 1.0; // [m] apenas alcanza a reconocer a persona
	float init_max_distance;     // init_min_distance = 2.2; // [m] 2 mts es demasiado lejos
	float init_max_width;        // init_max_width    = 1.2; // [m] ancho de la ventana de análisis
	float init_max_body_parts_distance; // [m]

	std::string SOURCE_NI_DETECTOR;
	std::string SOURCE_LEG_DETECTOR;
	bender_follow_me::SingleBodyDetection init_leg_observation;
	bender_follow_me::SingleBodyDetection init_ni_observation;
	ros::Time init_leg_observation_time;
	ros::Time init_ni_observation_time;
	bool _initialized;
	ros::Duration _max_non_updated_track_reinit_time; // [s]
	ros::Duration _max_non_updated_track_lifetime; // [s]
	float _max_track_uncertainty; // [m]
	float _max_observation_dist;  // [m]
	uint init_id;

	// common
	LaserModel _lmodel;

	// for each track:
	//BodyDetection _last_observations;
	std::list<Track> tracks;
	//ros::Time _last_update_time;
	//MotionModel _model;

	static const int _p;
	static const int _n;
	static const float _W0;
	static const float _Wi;

	void publishTracking(const std::vector<geometry_msgs::PoseStamped> &hypotheses, const std::vector<state_t> &X_points, const std::vector<state_t> &Y_points, const std::vector<observation_t> &Z_points) {

		// tracking status
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		bender_follow_me::TrackingState msg;

		for (int i=0; i<hypotheses.size(); ++i) {

			geometry_msgs::PoseStamped h = hypotheses[i];
			msg.header = h.header;
			msg.ids.push_back(i);
			msg.x_a.push_back(h.pose.position.x);
			msg.y_a.push_back(h.pose.position.y);
			msg.theta_a.push_back(180.0*2.0*acosf(h.pose.orientation.w)/M_PI);
			msg.vel_a.push_back(-99999);

			_tracking_pub.publish(msg);
		}

		// visualization step
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		if (_tracking_markers_pub.getNumSubscribers() > 0) {

			visualization_msgs::MarkerArray marker_array;
			visualization_msgs::Marker marker;
			visualization_msgs::Marker marker_text;

			marker.ns = "leg_tracking";
			marker.type = visualization_msgs::Marker::CYLINDER;
			marker.action = visualization_msgs::Marker::MODIFY;
			marker.lifetime = ros::Duration(0.2);

			marker_text.ns = marker.ns + "_text";
			marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker_text.action = visualization_msgs::Marker::MODIFY;
			marker_text.lifetime = ros::Duration(0.2);

			// size
			marker.scale.x = 0.20;
			marker.scale.y = 0.20;
			marker.scale.z = 0.60;

			// colors
			std_msgs::ColorRGBA color;
			color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
			marker.color = color;

			for (int i=0; i<hypotheses.size(); ++i) {

				geometry_msgs::PoseStamped h = hypotheses[i];
				marker.header = h.header;
				marker.pose = h.pose;

				// add hypothesis
				marker.id = i;
				marker_array.markers.push_back(marker);

				// text id
				marker_text.header = h.header;
				marker_text.scale.z = 0.3;
				marker_text.color.a = 1.0;
				marker_text.color.b = 1.0;
				marker_text.color.g = 1.0;
				marker_text.color.r = 0.0;
				marker_text.pose = marker.pose;
				marker_text.pose.position.x = marker_text.pose.position.x + 0.2;
				marker_text.pose.position.y = marker_text.pose.position.y + 0.2;
				marker_text.pose.position.z = marker_text.pose.position.z + 0.2;
				std::stringstream ss; ss << "id:" << (uint)i;
				marker_text.text = ss.str();
				marker_text.id = i;
				marker_array.markers.push_back(marker_text);

			}

			marker.ns = "leg_tracking-pose";
			marker.type = visualization_msgs::Marker::ARROW;

			// size
			marker.scale.x = 0.50;
			marker.scale.y = 0.10;
			marker.scale.z = 0.10;

			for (int i=0; i<hypotheses.size(); ++i) {

				geometry_msgs::PoseStamped h = hypotheses[i];
				marker.header = h.header;
				marker.pose = h.pose;

				// add hypothesis
				marker.id = i;
				marker_array.markers.push_back(marker);
			}

			std_msgs::Header header = marker.header;

			// - - - - add X_points - - - -
			std_msgs::ColorRGBA colorX0;
			colorX0.r = 1.0; colorX0.g = 1.0; colorX0.b = 0.0; colorX0.a = 1.0;
			std_msgs::ColorRGBA colorX;
			colorX.r = 0.0; colorX.g = 1.0; colorX.b = 0.0; colorX.a = 1.0;
			std::string nsX = "X-points";
			createMarkerSigmaPoints(X_points, header, marker_array, nsX, colorX0, colorX);

			// - - - - add Y_points - - - -
			std_msgs::ColorRGBA colorY0;
			colorY0.r = 1.0; colorY0.g = 0.0; colorY0.b = 1.0; colorY0.a = 1.0;
			std_msgs::ColorRGBA colorY;
			colorY.r = 0.0; colorY.g = 0.0; colorY.b = 1.0; colorY.a = 1.0;
			std::string nsY = "Y-points";
			createMarkerSigmaPoints(Y_points, header, marker_array, nsY,colorY0, colorY);

			// - - - - add Z_points - - - -
			std_msgs::ColorRGBA colorZ0;
			colorZ0.r = 1; colorZ0.g = 1.0; colorZ0.b = 0.5; colorZ0.a = 1.0;
			std_msgs::ColorRGBA colorZ;
			colorZ.r = 0.5; colorZ.g = 1.0; colorZ.b = 1.0; colorZ.a = 1.0;
			std::string nsZ = "Z-points";
			std::vector<state_t> Z_points_;
			for (int i=0; i<Z_points.size(); ++i) {
				observation_t Zi = Z_points[i];
				state_t Zi_;
				LaserModel::observationToState(Zi, Zi_);
				Z_points_.push_back(Zi_);
			}
			createMarkerSigmaPoints(Z_points_, header, marker_array, nsZ,colorZ0, colorZ);

			// publish
			_tracking_markers_pub.publish(marker_array);
		}

	};
};

float BayesianFilter::normalizeRadian(float angle) {

	float sinx, cosx;
	sincosf(angle, &sinx, &cosx);

	return atan2f(sinx, cosx);
}

void BayesianFilter::createMarkerSigmaPoints(const std::vector<state_t> &S_points, const std_msgs::Header header, visualization_msgs::MarkerArray &marker_array, std::string &ns, std_msgs::ColorRGBA color0,  std_msgs::ColorRGBA color) {

	if (S_points.empty()) {
		return;
	}

	visualization_msgs::Marker marker;

	// prepare
	marker.header = header;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::MODIFY;
	marker.ns = ns;


	// add S0
	marker.id = 0;
	marker.scale.x = 0.15;
	marker.scale.y = 0.15;
	marker.color = color0;

	state_t S0 = S_points[0];
	geometry_msgs::Point p;
	MotionModel::toPoint(S0, p);
	marker.points.push_back(p);
	marker_array.markers.push_back(marker);

	// add Si
	marker.id = 1;
	marker.color = color;
	marker.scale.x = 0.10;
	marker.scale.y = 0.10;
	for (int i=1; i<S_points.size(); ++i) {

		state_t Si = S_points[i];
		MotionModel::toPoint(Si, p);
		marker.points.push_back(p);
	}
	marker_array.markers.push_back(marker);
}


// - - - compute weights - - -
const int BayesianFilter::_n = State::length();
const int BayesianFilter::_p = 3 - BayesianFilter::_n;
const float BayesianFilter::_W0 = (BayesianFilter::_p+0.0)/(BayesianFilter::_n + BayesianFilter::_p);
const float BayesianFilter::_Wi = 1.0/(2.0*(BayesianFilter::_n+BayesianFilter::_p));


// (Static) Unscented Transform
void BayesianFilter::unscentedTransform(const State &x, State &y, const state_cov_t &Q, std::vector<state_t> &Y_points, std::vector<state_t> &X_points) {

	//bool debug = false;//__DEBUG_UT__;
	//bool cond2 = false;

	//ROS_WARN_COND(cond2,">> A");

	// preparation
	Y_points.clear();
	X_points.clear();
	y.clean();

	//ROS_WARN_COND(cond2,">> B");

	y.time = ros::Time::now();
	//ROS_WARN_STREAM_COND(debug,"y.time=" << y.time);
	double dt = (y.time - x.time).toSec();
	//ROS_WARN_STREAM_COND(debug,"dt=" << dt);

	// - - - compute sigma points & a-priori estimate - - -

	// X0 --> X0_
	state_t X0 = x.mean;
	X_points.push_back(X0);
	//ROS_WARN_STREAM_COND(debug,"X0 = [" << X0.transpose() << "]");
	state_t Y0;
	MotionModel::predict(X0,Y0,dt);
	Y_points.push_back(Y0);
	//ROS_WARN_STREAM_COND(debug,"i:0, Y0 = [" << Y0.transpose() << "]");

	//ROS_WARN_COND(cond2,">> C");

	y.mean += _W0*Y0;
	//ROS_WARN_STREAM_COND(debug,"i:0, y  = [" << y.mean.transpose() << "]");

	// X1,...,X2n ---> X_1,...,X_2n
	state_cov_t pre_root = (_n+_p)*x.cov;
	Eigen::SelfAdjointEigenSolver<state_cov_t> solver(pre_root);
	state_cov_t Pxx_np = solver.operatorSqrt();
	//ROS_WARN_STREAM_COND(debug,"Pxx_np=[\n" << Pxx_np << "]");
	for (int i=0; i<_n; ++i) {

		state_t Xi1 = x.mean + Pxx_np.col(i);
		state_t Xi2 = x.mean - Pxx_np.col(i);
		state_t Yi1;
		state_t Yi2;
		MotionModel::predict(Xi1,Yi1,dt);
		MotionModel::predict(Xi2,Yi2,dt);
		X_points.push_back(Xi1);
		X_points.push_back(Xi2);
		Y_points.push_back(Yi1);
		Y_points.push_back(Yi2);
		//ROS_WARN_STREAM_COND(debug,"i:"<< i+1 << ", Y+ = [" << Yi1.transpose() << "], Y- = [" << Yi2.transpose() << "]");

		y.mean += _Wi*Yi1 + _Wi*Yi2;

	}
	//ROS_WARN_STREAM_COND(debug,"y  = [" << y.mean.transpose() << "]");

	//ROS_WARN_COND(cond2,">> D");

	// compute Pyy
	y.cov += (_W0+1.0)*(Y0 - y.mean)*(Y0 - y.mean).transpose();
	for(int i=1; i<Y_points.size(); ++i) {
		state_t Yi = Y_points[i];
		y.cov += _Wi*(Yi - y.mean)*(Yi - y.mean).transpose();
	}
	y.cov += Q;
	//ROS_WARN_STREAM_COND(debug,"Pyy=[\n" << y.cov << "]");

	//ROS_WARN_COND(cond2,">> E");
}


} /* namespace bender_follow_me */

#endif /* BAYESIANFILTER_HPP_ */

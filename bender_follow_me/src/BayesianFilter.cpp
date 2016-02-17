/*
 * BayesianFilter.cpp
 *
 *  Created on: 16-10-2014
 *      Author: matias
 */

/**
 * TODO: este nodo debe estar preocupado de trackear solamente a todo lo que se mueva y venga de los detectores... nada de dar preferencia a cierta hipótesis
 * TODO: alineamiento de matrices de tamaño fijo: http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
 * TODO: a veces (rara vez) tira un bad_alloc
 * TODO: se pueden generar nans! (no ha sucedido ultimamente)
 * TODO: por alguna razón, el filtro se inicializa con las orientaciones
 *   giradas en 180 grados :(, pero converge rápidamente.. cáda vez que
 *   se pierden las detecciones, también se rota la orientación al recuperlas. :(
 * TODO: ha tirado 1 vez el error: terminate called after throwing an instance of 'ros::serialization::StreamOverrunException'   what():  Buffer Overrun
 * TODO: De repente muere sin sentido alguno :(
 * TODO: Error in `/home/matias/catkin_ws/devel/lib/bender_follow_me/bayesian_filter': double free or corruption (top): 0x00007f4714000e70 ***
 * TODO: *** Error in `/home/matias/catkin_ws/devel/lib/bender_follow_me/bayesian_filter': corrupted double-linked list: 0x00007f5ba4000d20 ***
 * TODO: velocidad mostrarla con flecha de distinto tamaño
 * TODO: visualizar certezas del tracking, en algún lado.
 */

#include <bender_follow_me/BayesianFilter.hpp>

namespace bender_follow_me {

static bool __DEBUG__ = false;
//static bool __DEBUG_UT__ = __DEBUG__;
static bool __DEBUG_GZ__ = __DEBUG__;
//static bool __DEBUG_CB__ = __DEBUG__;
static bool __DEBUG_KC__ = __DEBUG__;
static bool __DEBUG_KSTEP__ = false;


BayesianFilter::BayesianFilter() {

	priv = ros::NodeHandle("~");

	// [important!] wait for timer initialization!
	while (ros::ok() && ros::Time::now().isZero()) ;

	if (pthread_mutex_init(&_observation_mutex, NULL) != 0)
	{
		printf("\n mutex init failed\n");
		exit(1);
	}

	// - - - - - - - - - - - - - - - -  P U B L I S H E R S - - - - - - - - - - - - - - - -
	_tracking_pub = priv.advertise<bender_follow_me::TrackingState>("tracking",1, this);
	_tracking_markers_pub = priv.advertise<visualization_msgs::MarkerArray>("tracking_markers",1, this);

	// - - - - - - - - - - - - - - - - S U B S C R I B E R S - - - - - - - - - - - - - - -
	//_leg_detection_sub = priv.subscribe("leg_detections", 1, &BayesianFilter::callback_detections,this);
	//_ni_detection_sub = priv.subscribe("ni_detections", 1, &BayesianFilter::callback_detections,this);

	// servers
	_initialize_server = priv.advertiseService("initialize_tracker", &BayesianFilter::initializeFilterService, this);
	_get_init_id_server = priv.advertiseService("get_init_id", &BayesianFilter::getInitIdService, this);

	// initialize data
	//_last_update_time = ros::Time::now();
	SOURCE_NI_DETECTOR = "NI";
	SOURCE_LEG_DETECTOR = "LEG";
	_frame_id = "/bender/base_link";
	_max_non_updated_track_lifetime = ros::Duration(6); // [s]
	_max_track_uncertainty = 2;  // [m]
	_max_observation_dist = 1.5; // [m]
	_initialized = false;
	init_desired_distance = 1.5; // [m] no se ve mi cabeza
	init_min_distance = 1.0;     // [m] apenas alcanza a reconocer a persona
	init_max_distance = 2.0;     // [m] 2 mts es demasiado lejos
	init_max_width    = 1.2;     // [m] ancho de la ventana de análisis
	init_max_body_parts_distance = 0.25; // [m] ... 30[cm]
	init_leg_observation_time = ros::Time::now();
	init_ni_observation_time = ros::Time::now();
	initialization_triggered = false;
	init_id = 0;
	_max_non_updated_track_reinit_time = ros::Duration(1); // reinit when acquired again

	ROS_DEBUG_STREAM("Using W0=" << _W0 << ", Wi=" << _Wi);
	ROS_INFO("Working");
}

BayesianFilter::~BayesianFilter() {
	pthread_mutex_destroy(&_observation_mutex);
}


// (Static)
void BayesianFilter::generateExpectedObservation(const std::vector<state_t> &Y_points, std::vector<observation_t> &Z_points, observation_t &z, observation_cov_t &Pzz, const observation_cov_t &R) {

	bool debug = __DEBUG_GZ__;

	// preparation
	z.fill(0);
	Pzz.fill(0);
	Z_points.clear();

	// we are ready
	if (Y_points.empty()) return;

	int n = State::length();

	// Y0 --> Z0
	state_t Y0 = Y_points[0];
	//ROS_WARN_STREAM_COND(debug,"Y0=[" << Y0.transpose() << "]");
	observation_t Z0;
	LaserModel::generateExpectedObservation(Y0,Z0);
	Z_points.push_back(Z0);
	ROS_WARN_STREAM_COND(debug,"i:0, Z0 = [" << Z0.transpose() << "]");

	z += _W0*Z0;
	//ROS_WARN_STREAM_COND(debug,"i:0, z  = [" << z.transpose() << "]");

	// Y1,...,Y2n ---> Z1,...,Z2n
	for (int i=1; i<=n; ++i) {

		state_t Yi1 = Y_points[2*i-1];
		state_t Yi2 = Y_points[2*i];
		observation_t Zi1;
		observation_t Zi2;
		LaserModel::generateExpectedObservation(Yi1,Zi1);
		LaserModel::generateExpectedObservation(Yi2,Zi2);
		Z_points.push_back(Zi1);
		Z_points.push_back(Zi2);
		ROS_WARN_STREAM_COND(debug,"i:"<< i << ", Z+ = [" << Zi1.transpose() << "], Z- = [" << Zi2.transpose() << "]");

		z += _Wi*Zi1 + _Wi*Zi2;
	}
	ROS_WARN_STREAM_COND(debug,"z  = [" << z.transpose() << "]");


	// compute Pzz
	Pzz += (_W0+1.0)*(Z0 - z)*(Z0 - z).transpose();
	for(int i=1; i<Z_points.size(); ++i) {
		observation_t Zi = Z_points[i];
		Pzz += _Wi*(Zi - z)*(Zi - z).transpose();
	}
	Pzz += R;
	ROS_WARN_STREAM_COND(debug,"Pzz = [\n" << Pzz << "]");
}

// (Static)
void BayesianFilter::kalmanCorrection(State &x, const State &y, const std::vector<state_t> &Y_points, const std::vector<observation_t> &Z_points, const observation_t z, const observation_cov_t Pzz, const observation_t z_real) {

	float debug = __DEBUG_KC__;

	// useful values
	int x_size = State::length();
	int z_size = z.size();

	// prepare matrix Pxz
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Pxz;
	//Eigen::Matrix<float, bender_follow_me::state_length, bender_follow_me::state_length> Pxz;
	Pxz.resize(x_size,z_size);
	Pxz.fill(0);

	// prepare matrix K
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> K;
	K.resize(x_size,z_size);
	K.fill(0);

	// compute Pxz
	state_t Y0 = Y_points[0];
	observation_t Z0 = Z_points[0];
	Pxz += _W0*(Y0-y.mean)*(Z0-z).transpose();
	for (int i=1; i<=x_size; ++i) {
		state_t Yi1 = Y_points[2*i-1];
		state_t Yi2 = Y_points[2*i];
		observation_t Zi1 = Z_points[2*i-1];
		observation_t Zi2 = Z_points[2*i];
		ROS_WARN_STREAM_COND(debug,"i:"<< i << ", Y+ = [" << Yi1.transpose() << "], Y- = [" << Yi2.transpose() << "]");
		ROS_WARN_STREAM_COND(debug,"i:"<< i << ", Z+ = [" << Zi1.transpose() << "], Z- = [" << Zi2.transpose() << "]");

		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Ptmp;
		Ptmp.resize(x_size,z_size);
		Ptmp.fill(0);
		Ptmp = _Wi*((Yi1-y.mean)*(Zi1-z).transpose() + (Yi2-y.mean)*(Zi2-z).transpose());
		ROS_WARN_STREAM_COND(debug,"Ptmp = [\n" << Ptmp << "]");
		Pxz += Ptmp;
	}
	ROS_WARN_STREAM_COND(debug,"Pxz = [\n" << Pxz << "]");

	// compute kalman Gain K
	K = Pxz * Pzz.inverse();
	ROS_WARN_STREAM_COND(debug,"K = [\n" << K << "]");


	observation_t v = z_real - z;
	ROS_WARN_STREAM_COND(debug,"z   = [" << z.transpose() << "]");
	ROS_WARN_STREAM_COND(debug,"z_r = [" << z_real.transpose() << "]");
	ROS_WARN_STREAM_COND(debug,"v = z_r - z = [" << v.transpose() << "]");

	// finally
	x.mean = y.mean + K * v;
	x.mean(2) = normalizeRadian((float)x.mean(2)); // TODO: generalizar esto... es pega del modelo!, no del tracker
	x.cov  = y.cov  - K * Pzz * K.transpose();
	x.time = y.time;
}

void BayesianFilter::MHT_spinOnce() {

	// there are no tracks
	if (tracks.size() == 0) {
		return;
	}

	// list for publishing tracking results
	std::vector<geometry_msgs::PoseStamped> hypotheses;
	std::vector<state_t> all_X_points, all_Y_points;
	std::vector<observation_t> all_Z_points;

	// analize each track
	std::list<Track>::iterator track_it;
	for ( track_it = tracks.begin(); track_it != tracks.end(); ++track_it ) {

		//kalman_spinOnce();

		// - - - - - - predict - - - - - - - -
		// compute unscented Transform
		std::vector<state_t> X_points, Y_points;
		State y;
		unscentedTransform(track_it->model._x, y, track_it->model._Q, Y_points, X_points);


		// - - - - - - update - - - - - - -
		std::vector<observation_t> Z_points;
		// update if there exists any new data for this track
		if ( track_it->has_new_data ) {

			// predict observation
			observation_t z;
			observation_cov_t Pzz;
			generateExpectedObservation(Y_points, Z_points, z, Pzz, _lmodel._R);

			// kalman correction
			observation_t z_real;
			LaserModel::detectionToObservation(track_it->new_data.position, track_it->new_data.heading, z_real);
			kalmanCorrection(track_it->model._x, y, Y_points, Z_points, z, Pzz, z_real);

			// update time counter
			track_it->last_update_time = track_it->new_data.observation_time;

			// mark 'new_data' as used
			track_it->has_new_data = false;

		} else {
			// there are no new data

			// leave track state as filter prediction
			track_it->model._x = y;
		}

		geometry_msgs::PoseStamped h;
		h.header.stamp = track_it->model._x.time;
		h.header.frame_id = _frame_id;
		MotionModel::toPoint(track_it->model._x.mean, h.pose.position);
		h.pose.orientation.z = 1.0*sinf(0.5*(float)track_it->model._x.mean(2));
		h.pose.orientation.w = cosf(0.5*(float)track_it->model._x.mean(2));
		hypotheses.push_back(h);

		all_X_points.insert(all_X_points.end(), X_points.begin(), X_points.end() );
		all_Y_points.insert(all_Y_points.end(), Y_points.begin(), Y_points.end() );
		all_Z_points.insert(all_Z_points.end(), Z_points.begin(), Z_points.end() );

	}

	// - - - publish current track results - - -
	publishTracking(hypotheses, all_X_points, all_Y_points, all_Z_points);
}

void BayesianFilter::kalman_spinOnce() {

	float debug = __DEBUG_KSTEP__;
}

uint BayesianFilter::getEmptyId() {

	uint id = 0;

	// get current id's
	std::vector<uint> ids;
	std::list<Track>::iterator track_it;
	for ( track_it = tracks.begin(); track_it != tracks.end(); ++track_it ) {
		ids.push_back(track_it->id);
	}

	// order ids
	std::sort(ids.begin(), ids.end());

	std::vector<uint>::iterator ids_it;
	for (ids_it = ids.begin(); ids_it != ids.end(); ++ids_it) {

		if (id != (*ids_it)) {
			return id;
		} else {
			id++;
		}
	}
	return id;
}

void BayesianFilter::associateObservations(const bender_follow_me::BodyDetections &msg, std::vector<bool> &used_observations) {

	float max_float = std::numeric_limits<float>::max();
	uint n_observations = msg.detections.size();

	// association matrix
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic > S;
	S.resize(tracks.size(), n_observations);
	S.fill(0);
	//printf("C> B\n");

	// Compute association matrix
	std::list<Track>::iterator track_it;
	int i=0;
	for ( track_it = tracks.begin(); track_it != tracks.end(); ++track_it, ++i) {
		//printf("C> C, i=%d\n",i);

		// predict next state
		std::vector<state_t> X_points, Y_points;
		State next_state;
		unscentedTransform(track_it->model._x, next_state, track_it->model._Q, Y_points, X_points);

		float x = next_state.mean(0);
		float y = next_state.mean(1);
		//float heading = next_state.mean(2);

		uint track_id = track_it->id;
		for (int j=0; j < msg.detections.size(); ++j) {

			//printf("C> C, i=%d, j=%d\n",i,j);
			float obs_x = msg.detections[j].x;
			float obs_y = msg.detections[j].y;

			// euclidean distance // TODO: mahalanobis distance
			float dx = x - obs_x;
			float dy = y - obs_y;
			S(i,j) = sqrtf( dx*dx + dy*dy );
		}
	}

	// update tracks
	//printf("C> D\n");
	int max_associations = std::min(S.cols(), S.rows());
	for (int k=0; k < max_associations; ++k) {

		int i_idx, j_idx;
		float min_val;

		// get min association distance
		min_val = S.minCoeff(&i_idx, &j_idx);

		if (min_val < _max_observation_dist) {
			// associate

			track_it = tracks.begin();
			std::advance(track_it, i_idx);

			// re-initialyze if neccesary
			if (msg.header.stamp - track_it->last_update_time > _max_non_updated_track_reinit_time) {
				track_it->reinitModel();
			}

			// queue new data
			track_it->new_data.position.x = msg.detections[j_idx].x;
			track_it->new_data.position.y = msg.detections[j_idx].y;
			//track_it->new_data.heading    = msg.detections[j_idx].heading;
			track_it->new_data.observation_time = msg.header.stamp;
			track_it->has_new_data = true;
			used_observations[j_idx] = true;
		}

		// mark rows/cols as used
		S.row(i_idx).fill(max_float); // invalidate track
		S.col(j_idx).fill(max_float); // invalidate observation
	}
}

void BayesianFilter::initializeFilter() {

	if (_initialized == true) {
		return;
	}

	ROS_INFO("Initializing tracker . . .");

	// add message callbacks
	ROS_INFO(". . . subscribing to detection callbacks");
	_leg_detection_sub = priv.subscribe("leg_detections", 1, &BayesianFilter::callback_detections,this);
	_ni_detection_sub = priv.subscribe("ni_detections", 1, &BayesianFilter::callback_detections,this);

	ROS_INFO(". . . analyzing incoming data");
	ros::Duration dt(1); // 1 [s]
	bool time_ready;
	bool ready = false;
	float ni_x, ni_y;
	while (!ready && ros::ok()) {

		time_ready = ros::Time::now() - init_leg_observation_time < dt
				  && ros::Time::now() - init_ni_observation_time  < dt;
		if (time_ready ) {

			ni_x = init_ni_observation.x;
			ni_y = init_ni_observation.y;
			float dx = init_leg_observation.x - ni_x;
			float dy = init_leg_observation.y - ni_y;
			float dist = sqrtf(dx*dx + dy*dy);

			if (dist <= init_max_body_parts_distance) {
				ready = true;
			}

		} else {
			ros::Duration(0.1).sleep();
		}
	}

	ROS_INFO(". . . creating track");

	Track track;
	float obs_x = ni_x;
	float obs_y = ni_y;
	float obs_heading = 0;
	// initialize track
	track.id = getEmptyId();
	track.has_new_data = false;
	track.last_update_time = ros::Time::now();
	track.initModel(obs_x, obs_y);

	// append new track
	init_id = track.id;
	tracks.push_back(track);
	_initialized = true;

	/*while (tracks.size() == 0) {
		ros::Duration(0.1).sleep();
	}*/
	ROS_INFO("Tracker initialized OK");
}

bool BayesianFilter::getInitIdService(bender_srvs::ID::Request &req, bender_srvs::ID::Response &res) {

	if (!_initialized) {
		res.ID = -1;
	} else {
		res.ID = init_id;
	}

	return true;
}

bool BayesianFilter::initializeFilterService(bender_srvs::String::Request &req, bender_srvs::String::Response &res) {

	/*
	if (!_initialized) {
		initialization_triggered = true;
	}*/
	return true;
}

void BayesianFilter::callback_detections(const bender_follow_me::BodyDetections &msg) {

	//printf(". . .callback\n");
	/*
	if (!_initialized) {

		//printf(". . . not initialized\n");

		// find the nearest person to the desired position
		float min_distance = 9999;
		bool found_desired = false;
		bender_follow_me::SingleBodyDetection nearest;
		std::vector<bender_follow_me::SingleBodyDetection>::const_iterator body;
		for (body = msg.detections.begin(); body != msg.detections.end(); ++body) {

			// position constraints
			if ( fabsf(body->y) > init_max_width/2.0	// width constraint
				|| body->x < init_min_distance			// min distance constraint
				|| body->x > init_max_distance			// max distance constraint
				) {
				continue;
			}

			// legs constraints: Single legs has low reliability
			if (msg.source == SOURCE_LEG_DETECTOR
				&& body->leg_type == LegsPattern::SL) {
				continue;
			}

			float dx = body->x - init_desired_distance;
			float dy = body->y - 0.0;
			float dist = sqrtf(dx*dx + dy*dy);
			if (dist < min_distance) {
				min_distance = dist;
				found_desired = true;
				nearest = (*body);
			}
		}

		// save it
		if (found_desired) {

			if (msg.source == SOURCE_LEG_DETECTOR) {
				init_leg_observation = nearest;
				init_leg_observation_time = msg.header.stamp;

			} else if ( msg.source == SOURCE_NI_DETECTOR ) {
				init_ni_observation = nearest;
				init_ni_observation_time = msg.header.stamp;
			}
		}
		return;
	}*/

	// Data association, tracks creation, tracks deletion
	//printf(". . .initialized\n");
	//printf(". . .waiting lock\n");
	pthread_mutex_lock(&_observation_mutex);
	//printf(". . . lock granted\n");
	printf("C> tracks:%lu, observations:%lu\n", tracks.size(), msg.detections.size());
	uint n_observations = msg.detections.size();
	if (n_observations == 0) {
		//printf(". . . unlocking\n");
		pthread_mutex_unlock(&_observation_mutex);
		//printf(". . . unlocked\n");
		return;
	}
	//printf(". . .new data\n");

	//printf(">>> Z\n");

	// associate new data
	std::vector<bool> used_observations(n_observations, false);
	associateObservations(msg, used_observations);

	//printf(">>> A\n");

	// analyze track that weren't updated
	std::list<Track>::iterator track_it;
	for (track_it = tracks.begin(); track_it != tracks.end();) {

		//printf(">>> B\n");

		ros::Time now = msg.header.stamp;

		// max lifetime reached
		if (track_it->has_new_data == false) {

			if ( now - track_it->last_update_time  > _max_non_updated_track_lifetime ) {
				// deletion by max lifetime
				track_it = tracks.erase(track_it);

			} else if (track_it->getTrackUncertainty() > _max_track_uncertainty) {
				// deletion by great uncertainty
				track_it = tracks.erase(track_it);

			} else {
			++track_it;
			}
		}
	}

	// analyze unused observations
	for (int i=0; i<n_observations; ++i) {

		//printf(">>> C\n");

		// create tracks
		if ( used_observations[i] == false ) {

			if (msg.source == SOURCE_LEG_DETECTOR) {
				// legs detector

				if ( false /*msg.detections[i].leg_type == bender_laser::LegsPattern::SL */) {
					// not really important leg pattern

				} else {
					// LegsPattern::FS && LegsPattern::LA
					Track track;
					float obs_x = msg.detections[i].x;
					float obs_y = msg.detections[i].y;
					float obs_heading = msg.detections[i].heading;

					// initialize track
					track.id = getEmptyId();
					track.has_new_data = false;
					track.last_update_time = ros::Time::now();
					track.initModel(obs_x, obs_y);

					// append new track
					tracks.push_back(track);
					//printf(". . .new track\n");
				}

			} else if (msg.source == SOURCE_NI_DETECTOR) {
				// NI detector

				Track track;
				float obs_x = msg.detections[i].x;
				float obs_y = msg.detections[i].y;
				//float obs_heading = msg.detections[i].heading;

				// initialize track
				track.id = getEmptyId();
				track.has_new_data = false;
				track.last_update_time = ros::Time::now();
				track.initModel(obs_x, obs_y);

				// append new track
				tracks.push_back(track);
				//printf(". . .new track\n");

			} else {

			}
		}

	}

	//printf(". . . unlocking\n");
	pthread_mutex_unlock(&_observation_mutex);
	//printf(". . . unlocked\n");
}

} /* namespace bender_follow_me */


int main(int argc, char **argv){

	ros::init(argc, argv, "bayesian_filter");

	bender_follow_me::BayesianFilter filter;

	ros::AsyncSpinner spinner(2);

	spinner.start();
	ros::Rate r(20);
	while (ros::ok()) {

		/*if ( filter.initialization_triggered ) {
			filter.initializeFilter();
			filter.initialization_triggered = false;
		}*/

		try{
			//printf("ZERO> A\n");
			filter.MHT_spinOnce();
			//printf("ZERO> B\n");
		} catch (std::runtime_error &e) {
			ROS_ERROR_STREAM(e.what());
		} catch (std::exception &e) {
			ROS_ERROR_STREAM(e.what());
		}

		r.sleep();
	}

	printf("\nQuitting... \n\n");

	return 0;
}

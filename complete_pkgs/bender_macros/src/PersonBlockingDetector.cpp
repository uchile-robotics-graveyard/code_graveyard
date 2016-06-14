/*
 * PersonBlockingDetector.cpp
 *
 *      Author: matias.pavez.b@gmail.com
 */

#include <bender_macros/PersonBlockingDetector.hpp>


namespace bender_macros {

PersonBlockingDetector::PersonBlockingDetector() {

    ros::NodeHandle priv("~");

    _received_first_goal = false;
    _received_first_plan = false;

    // - - - - - - - P A R A M E T E R   S E R V E R - - - - - - - - -
    bender_utils::ParameterServerWrapper psw;
    psw.getParameter("map_frame",_map_frame, "/map");
    psw.getParameter("robot_frame",_robot_frame, "/bender/base_link");
    psw.getParameter("detection_lifetime",_detection_lifetime, 1.0);
    psw.getParameter("plan_tolerance",_plan_tolerance, 0.1);
    psw.getParameter("person_radius",_person_radius, 0.25);
    psw.getParameter("blocker_to_path_radius",_blocker_to_path_radius, 1.0);
    psw.getParameter("path_length_interact_th",_path_length_interact_th, 20.0);


    // - - - - - - - S E R V I C E   C L I E N T S - - - - - - - - - - - - - -
	_get_plan_client = priv.serviceClient<nav_msgs::GetPlan>("make_plan");
	while ( ros::ok() && !_get_plan_client.waitForExistence(ros::Duration(3.0)) ) ;

    // - - - - - - - S U B S C R I B E R S - - - - - - - - - - - - - - -
    // person detector data
    _leg_detections_sub  = priv.subscribe("people_legs", 1, &PersonBlockingDetector::callback_leg_detections,this);
    _person_detections_sub  = priv.subscribe("people_body", 1, &PersonBlockingDetector::callback_person_detections,this);

    // navigation data
    _new_goal_sub  = priv.subscribe("new_goal", 1, &PersonBlockingDetector::callback_newGoal,this);
    _new_plan_sub  = priv.subscribe("new_plan", 1, &PersonBlockingDetector::callback_newPlan,this);

    // - - - - - - - P U B L I S H E R S - - - - - - - - - - - - - - -
    _blocking_pub = priv.advertise<bender_msgs::PathBlockingDetections>("blocking_detections", 1);
    _blocking_markers_pub = priv.advertise<visualization_msgs::MarkerArray>("blocking_markers", 1);


    // [important!] wait for timer initialization!
	while (ros::ok() && ros::Time::now().isZero());

	ros::Duration(1.0).sleep();

    ROS_INFO("Ready to work");
}

PersonBlockingDetector::~PersonBlockingDetector() {

}

void PersonBlockingDetector::deleteOldDetections() {

	ros::Time now = ros::Time::now();
	ros::Duration max_dt(_detection_lifetime);

	std::vector<std::string> sources;
	std::vector<geometry_msgs::PoseStamped> detections;
	for (int i=0; i<_detections.size(); ++i) {

		if ( (now - _detections[i].header.stamp) < max_dt ) {
			detections.push_back(_detections[i]);
			sources.push_back(_sources[i]);
		}
	}

	_detections = detections;
	_sources = sources;
}

bool PersonBlockingDetector::request_plan(nav_msgs::Path& plan, const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, const float tolerance) {

	if ( !_get_plan_client.waitForExistence(ros::Duration(0.5)) ) {
		ROS_WARN_STREAM("cannot request navigation plan... server unavailable: " << _get_plan_client.getService() );
		return false;
	}

	nav_msgs::GetPlan srv;
	srv.request.start = start;
	srv.request.goal = goal;
	srv.request.tolerance = tolerance;

	try {
		_get_plan_client.call(srv);
		plan = srv.response.plan;

	} catch (std::exception &e) {
		ROS_WARN_STREAM("Failed to get plan from server: " << _get_plan_client.getService());
		return false;
	}

	return true;
}

float PersonBlockingDetector::distancePointPoint(const float &Ax, const float &Ay, const float &Bx, const float &By) {
	return sqrtf((Bx-Ax)*(Bx-Ax) + (By-Ay)*(By-Ay));
}

float PersonBlockingDetector::distancePointSegment(const geometry_msgs::PoseStamped &C, const geometry_msgs::PoseStamped &A, const geometry_msgs::PoseStamped &B) {

	float Ax = A.pose.position.x;
	float Ay = A.pose.position.y;
	float Bx = B.pose.position.x;
	float By = B.pose.position.y;
	float Cx = C.pose.position.x;
	float Cy = C.pose.position.y;

	const float L2 = (Bx-Ax)*(Bx-Ax) + (By-Ay)*(By-Ay);
	if (L2 == 0.0) {
		// the two points are the same
		return distancePointPoint(Ax, Ay, Cx, Cy);
	}


	const float r  = ((Cx-Ax)*(Bx-Ax) + (Cy-Ay)*(By-Ay))/L2;
	if (r < 0.0) {

		// P is on the backward extension of AB
		// distance Point - A
		return distancePointPoint(Ax, Ay, Cx, Cy);

	} else if (r>1.0) {

		// P is on the forward extension of AB
		// distance Point - B
		return distancePointPoint(Bx, By, Cx, Cy);
	}

	// P is interior to AB
	// distance Point - Projection
	const float Px = Ax + r*(Bx-Ax);
	const float Py = Ay + r*(By-Ay);
	return distancePointPoint(Cx, Cy, Px, Py);
}

bool PersonBlockingDetector::getCurrentPoseInMap(geometry_msgs::PoseStamped& robot) {

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.frame_id = _robot_frame;
	pose_stamped.header.stamp = ros::Time::now();
	pose_stamped.pose.orientation.w = 1;

	// transform pose
	try {
		_tf_listener.waitForTransform(_map_frame, _robot_frame, pose_stamped.header.stamp, ros::Duration(2.0));
		_tf_listener.transformPose(_map_frame, pose_stamped, robot);

	} catch (tf::ConnectivityException &e) {
		ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
		return false;

	} catch (tf::TransformException &e) {
		ROS_WARN_STREAM("Transform Exception: " << e.what());
		return false;
	}

	return true;
}

void PersonBlockingDetector::publish_blockers(const std::vector<geometry_msgs::PoseStamped>& blockers, const std::vector<PersonBlockingDetector::obstacle_t> &states, bool should_interact) {

	if (_blocking_pub.getNumSubscribers() > 0) {

		bender_msgs::PathBlockingDetections msg;
		msg.header.frame_id = _map_frame;
		msg.header.stamp = ros::Time::now();
		msg.should_take_action = should_interact;

		for (int i=0; i<blockers.size(); ++i) {

			msg.detections.push_back(blockers[i].pose);

			if ( states[i] == PersonBlockingDetector::OBSTACLE ) {
				msg.info.push_back("obstacle");
			} else {
				msg.info.push_back("candidate");
			}

		}
		_blocking_pub.publish(msg);
	}

	if (_blocking_markers_pub.getNumSubscribers() > 0) {

		// visualization step
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		visualization_msgs::MarkerArray marker_array;
		visualization_msgs::Marker marker;

		marker.ns = "path_blocker_people";
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::MODIFY;
		marker.lifetime = ros::Duration(0.1);

		// size
		marker.scale.x = 0.30;
		marker.scale.y = 0.30;
		marker.scale.z = 1.60;

		// colors
		std_msgs::ColorRGBA yellow, red, black;
		yellow.r = 1.0; yellow.g = 1.0; yellow.b = 0.0; yellow.a = 1.0;
		red.r    = 1.0; red.g    = 0.0; red.b    = 0.0; red.a    = 1.0;
		black.r  = 1.0; black.g  = 1.0; black.b  = 1.0; black.a  = 1.0;

		for (int k=0; k<blockers.size(); ++k) {

			geometry_msgs::PoseStamped person = blockers[k];
			marker.header = person.header;
			marker.pose = person.pose;

			// coloring
			if (states[k] == PersonBlockingDetector::CANDIDATE) {
				marker.color = yellow;
			} else if ( states[k] == PersonBlockingDetector::OBSTACLE ) {
				marker.color = red;
			} else {
				marker.color = black;
			}

			// add person
			marker.id = k;
			marker_array.markers.push_back(marker);
		}

		// publish
		_blocking_markers_pub.publish(marker_array);
	}

}

void PersonBlockingDetector::getRobotPositionOnPath(int &segment_index, const geometry_msgs::PoseStamped &robot) {

	segment_index = 0;
	float min_dist = 1000;
	const int plan_size = _current_plan.poses.size();
	for (int j=1; j<plan_size; ++j) {

		float d = distancePointSegment(robot, _current_plan.poses[j-1], _current_plan.poses[j]);
		if (d < min_dist) {
			min_dist = d;
			segment_index = j;
		}
	}

}

float PersonBlockingDetector::getPathLength(const nav_msgs::Path& path, const int& starting_idx) {

	float length = 0;
	const int plan_size = path.poses.size();
	for (int j=starting_idx; j<plan_size; ++j) {

		length += distancePointPoint(
				path.poses[j-1].pose.position.x, path.poses[j-1].pose.position.y,
				path.poses[ j ].pose.position.x, path.poses[ j ].pose.position.y
		);
	}
	return length;
}

void PersonBlockingDetector::spinOnce() {

	//ready to work
	if (!_received_first_plan || !_received_first_goal) {
		return;
	}

	// -- check robot position in the global path --

	// get robot position
	geometry_msgs::PoseStamped robot;
	if ( !getCurrentPoseInMap(robot) ) {
		return;
	}

	// get nearest path segment index
	int segment_index;
	getRobotPositionOnPath(segment_index, robot);

	// -- look for people near the global plan --

	// delete old detections
	deleteOldDetections();

	std::vector<geometry_msgs::PoseStamped> blocking_guys;
	std::vector<PersonBlockingDetector::obstacle_t> blocking_states;
	for (int i=0; i<_detections.size(); ++i) {

		geometry_msgs::PoseStamped person = _detections[i];

		// find distance from person to the path
		// look on the remaining path points
		float min_dist = 1000;
		for (int j=segment_index; j<_current_plan.poses.size(); ++j) {

			float d = distancePointSegment(person, _current_plan.poses[j-1], _current_plan.poses[j]);
			if (d < min_dist) {
				min_dist = d;
			}
		}

		// append potential blocker
		if (min_dist <= _person_radius) {
			blocking_guys.push_back(person);
			blocking_states.push_back(PersonBlockingDetector::CANDIDATE);
		}
	}


	// -- check real blockers --
	if (blocking_guys.size() == 0) {
		return;
	}

	// make plan
	nav_msgs::Path new_plan;
	if ( !request_plan(new_plan, robot, _current_goal, _plan_tolerance) ) {
		return;
	}

	// failed to generate a plan
	if (new_plan.poses.size() == 0) {

		// everyone is an obstacle!
		for (int i=0; i<blocking_states.size(); ++i) {
			blocking_states[i] = PersonBlockingDetector::OBSTACLE;
		}

		publish_blockers(blocking_guys, blocking_states, true);

	}
	// at this point, the new plan shouldn't collide with anyone!,
	// otherwise it would have failed.

	// almost reaching the goal AND with no obstacles
	if (new_plan.poses.size() == 1) {
		return;
	}

	// re-check distance between potential obstacles and the new path:
	// if somebody is near the new path, then he isn't an obstacle.
	// notice that the planner will try the shortest path, so there is
	// a high probability that the new_plan will be near the person.
	// otherwise: the path changed a lot, so he IS an obstacle for
	// us (but now he isn't, because the path changed).
	bool path_changed = false;
	for (int i=0; i<blocking_guys.size(); ++i) {

		geometry_msgs::PoseStamped blocker = blocking_guys[i];

		// distance person-new_path
		float min_dist = 1000;
		for (int j=1; j<new_plan.poses.size(); ++j) {

			float d = distancePointSegment(blocker, new_plan.poses[j-1], new_plan.poses[j]);
			if (d < min_dist) {
				min_dist = d;
			}
		}

		// mark real blocker
		if (min_dist > _blocker_to_path_radius) {
			blocking_states[i] = PersonBlockingDetector::OBSTACLE;
			path_changed = true;
		}
	}

	// TODO: lo que realmente se debería hacer, es chequear que tanto cambió un path con el otro
	// lo que se podría hacer con algo como una correlación.
	// compute path lengths
	bool should_interact;
	if (path_changed) {

		// compute path lengths
		float old_plan_length = getPathLength(_current_plan, segment_index);
		float new_plan_length = getPathLength(new_plan, 1);

		float diff = 100.0*(new_plan_length - old_plan_length)/old_plan_length;
		diff -= 100;

		if (diff < 0) {

			// new path is shorter
			should_interact = false;

		} else if ( fabsf(diff) < _path_length_interact_th ) {

			// old path isn't shorter enough
			should_interact = false;

		} else {

			// new path isn't shorter enough to be a good choice
			should_interact = true;
		}

	} else {
		should_interact = false;
	}

	// publish results
	publish_blockers(blocking_guys, blocking_states, should_interact);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - S u b s c r i b e r   C a l l b a c k s  -  - - - - - - - - - -

void PersonBlockingDetector::callback_leg_detections(const bender_msgs::PoseDetections &msg) {

	// don't waste time
	if (msg.detections.size() == 0) {
		return;
	}

	// delete old data from the leg detector
    std::vector<std::string> sources;
    std::vector<geometry_msgs::PoseStamped> detections;
	for (int i=0; i<_sources.size(); ++i) {

		if (_sources[i] != "legs") {
			detections.push_back(_detections[i]);
			sources.push_back(_sources[i]);
		}
	}
	_sources = sources;
	_detections = detections;

	// append new detections
	try {
		_tf_listener.waitForTransform(_map_frame, msg.header.frame_id, msg.header.stamp, ros::Duration(2.0));

		geometry_msgs::PoseStamped legs;
		geometry_msgs::PoseStamped tf_legs;
		for (int i=0; i<msg.detections.size(); ++i) {

			// SL detection represents a person, but reports many False Positives
			if ( msg.info[i] == bender_laser::LegsPattern::toString(bender_laser::LegsPattern::SL) ) {
				continue;
			}
			legs.header = msg.header;
			legs.pose.position = msg.detections[i].position;
			legs.pose.orientation = msg.detections[i].orientation;
			_tf_listener.transformPose(_map_frame, legs, tf_legs);
			_detections.push_back(tf_legs);
			_sources.push_back("legs");
		}

	} catch (tf::ConnectivityException &e) {
		ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
		return;

	} catch (tf::TransformException &e) {
		ROS_WARN_STREAM("Transform Exception: " << e.what());
		return;
	}
}

void PersonBlockingDetector::callback_person_detections(const bender_msgs::PoseDetections &msg) {

	// don't waste time
	if (msg.detections.size() == 0) {
		return;
	}

	// delete old data from the person detector
    std::vector<std::string> sources;
    std::vector<geometry_msgs::PoseStamped> detections;
	for (int i=0; i<_sources.size(); ++i) {

		if (_sources[i] != "hog") {
			detections.push_back(_detections[i]);
			sources.push_back(_sources[i]);
		}
	}
	_sources = sources;
	_detections = detections;

	// append new detections
	try {
		_tf_listener.waitForTransform(_map_frame, msg.header.frame_id, msg.header.stamp, ros::Duration(2.0));

		geometry_msgs::PoseStamped person;
		geometry_msgs::PoseStamped tf_person;
		for (int i=0; i<msg.detections.size(); ++i) {


			person.header = msg.header;
			person.pose.position = msg.detections[i].position;
			person.pose.orientation = msg.detections[i].orientation;
			_tf_listener.transformPose(_map_frame, person, tf_person);
			tf_person.pose.orientation.w = 1.0;
			tf_person.pose.orientation.x = 0.0;
			tf_person.pose.orientation.y = 0.0;
			tf_person.pose.orientation.z = 0.0;
			_detections.push_back(tf_person);
			_sources.push_back("hog");
		}

	} catch (tf::ConnectivityException &e) {
		ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
		return;

	} catch (tf::TransformException &e) {
		ROS_WARN_STREAM("Transform Exception: " << e.what());
		return;
	}

}

void PersonBlockingDetector::callback_newGoal(const geometry_msgs::PoseStamped new_goal) {

	// transform goal
	geometry_msgs::PoseStamped map_goal;
	if (new_goal.header.frame_id != _map_frame) {

		try {
			_tf_listener.waitForTransform(_map_frame, new_goal.header.frame_id, new_goal.header.stamp, ros::Duration(2.0));
			_tf_listener.transformPose(_map_frame, new_goal, map_goal);

		} catch (tf::ConnectivityException &e) {
			ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
			return;

		} catch (tf::TransformException &e) {
			ROS_WARN_STREAM("Transform Exception: " << e.what());
			return;
		}

	} else {
		map_goal = new_goal;
	}

	// save goal
	_current_goal = map_goal;
	_received_first_goal = true;
	ROS_INFO_STREAM("Received new goal: frame='" << _current_goal.header.frame_id << "', (x,y)=(" << std::setprecision(2) << std::fixed
			<< _current_goal.pose.position.x << "," << _current_goal.pose.position.y << ")");
}

void PersonBlockingDetector::callback_newPlan(const nav_msgs::Path &path) {

	if (path.poses.size() <= 1) {
		return;
	}

	// transform path
	nav_msgs::Path map_path;
	if (path.header.frame_id != _map_frame) {

		try {
			_tf_listener.waitForTransform(_map_frame, path.header.frame_id, path.header.stamp, ros::Duration(2.0));

			map_path.header.frame_id = _map_frame;
			map_path.header.stamp = path.header.stamp;

			geometry_msgs::PoseStamped tf_pose;
			for (int i=0; i<path.poses.size(); ++i) {

				_tf_listener.transformPose(_map_frame, path.poses[i], tf_pose);
				map_path.poses.push_back(tf_pose);
			}

		} catch (tf::ConnectivityException &e) {
			ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
			return;

		} catch (tf::TransformException &e) {
			ROS_WARN_STREAM("Transform Exception: " << e.what());
			return;
		}

		_current_plan = map_path;

	} else {
		_current_plan = path;
	}

	_received_first_plan = true;
	geometry_msgs::PoseStamped start = _current_plan.poses[0];
	geometry_msgs::PoseStamped goal  = _current_plan.poses[_current_plan.poses.size()-1];
	ROS_INFO_STREAM("Received new plan: frame='" << _current_plan.header.frame_id << "', from (x,y)=(" << std::setprecision(1) << std::fixed
			<< start.pose.position.x << "," << start.pose.position.y
			<< "), to (x,y)=(" << goal.pose.position.x << "," << goal.pose.position.y << ").");
}

} /* namespace bender_macros */

int main(int argc, char **argv){

	ros::init(argc, argv, "person_blocking_detector");

	boost::scoped_ptr<bender_macros::PersonBlockingDetector> node(
				new bender_macros::PersonBlockingDetector()
	);


	ros::Rate rate(20);
	while (ros::ok()) {

		node->spinOnce();

		ros::spinOnce();
		rate.sleep();
	}

	printf("\nQuitting... \n\n");

	return 0;
}

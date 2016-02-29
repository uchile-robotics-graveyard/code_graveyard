/*
 * BaseController.cpp
 *
 *  Created on: 16-12-2014
 *      Author: matias
 */

#include <bender_follow_me/BaseController.hpp>


namespace bender_follow_me {

BaseController::BaseController(std::string name): _name(name) {

	ros::NodeHandle priv("~");

	// parameters
	alpha_1 = 0.0;
	alpha_2 = 0.0;
	alpha_3 = 0.0;
	alpha_4 = 0.0;
	alpha_5 = 0.0;
	alpha_6 = 0.0;
	dt = 2*1.0/10; // control at 10[Hz]
	footprint_radius_squared = 0.4 * 0.4; // [m2]
	_frame_in = "/bender/base_link";


	// - - - - - - - - - - - - - - - - - - C L I E N T S - - - - - - - - - - - - - - - - -
	std::string tf_server_name = "/bender/tf/simple_pose_transformer/transform";
	_transformer_client = priv.serviceClient<bender_srvs::Transformer>(tf_server_name);

	// - - - - - - - - - - - - - - - -  P U B L I S H E R S - - - - - - - - - - - - - - - -
	_collision_pub = priv.advertise<visualization_msgs::Marker>("predicted_collision",1, this);
	_footprint_pub = priv.advertise<geometry_msgs::PolygonStamped>("footprint",1, this);
	_motion_footprint_pub = priv.advertise<geometry_msgs::PolygonStamped>("predicted_footprint",1, this);
	_cmd_pub = priv.advertise<geometry_msgs::Twist>("cmd_vel_out",1, this);

	// - - - - - - - - - - - - - - - - S U B S C R I B E R S - - - - - - - - - - - - - - -
	_laser_scan_sub = priv.subscribe("scan", 1, &BaseController::callback_laser_scan,this);
	_cmd_sub = priv.subscribe("cmd_vel_in", 1, &BaseController::callback_in_commands,this);

	// [important!] wait for timer initialization!
	while (ros::ok() && ros::Time::now().isZero());

	last_sent_command_time = ros::Time::now();
	last_cmd_time = ros::Time::now();

	// wait clients
	while ( ros::ok() && !_transformer_client.waitForExistence(ros::Duration(3.0)) );

	ROS_INFO("Ready to Work");
}

BaseController::~BaseController() {}

float random_1_1() {
	return -1+2*((float)rand())/RAND_MAX;
}

float sample_gaussian(float in) {

	float out = 0;
	for (int i=0; i<12; ++i) {
		out += random_1_1();
	}
	return out*in/6.0;
}

float sample_triangular(float in) {

	return in*random_1_1()*random_1_1();
}

void BaseController::sample_motion_model_velocity(const geometry_msgs::Twist &cmd, float &x, float &y) {

	float x0 = 0;
	float y0 = 0;
	float theta0 = 0;

	float v = cmd.linear.x;
	float w = cmd.angular.z;
	w = -w;	// formula correction

	float _v_ =  v + sample_gaussian(alpha_1*fabsf(v) + alpha_2*fabsf(w));
	float _w_ = w + sample_gaussian(alpha_3*fabsf(v) + alpha_4*fabsf(w));
	//float _gamma_ = sample_gaussian(alpha_5*fabsf(v) + alpha_6*fabsf(w));

	if (_w_ == 0) { _w_ = 0.00001; }
	float d = (_v_/_w_);
	x = x0 - d*sinf(theta0) + d*sinf(theta0 + _w_*dt);
	y = y0 - d*cosf(theta0) + d*cosf(theta0 + _w_*dt);
	//float theta = theta0 + _w_*dt + _gamma_*dt;
}

void BaseController::callback_in_commands(const geometry_msgs::Twist &cmd) {

	last_cmd = cmd;
	last_cmd_time = ros::Time::now();

	// Dummy
	//_cmd_pub.publish(cmd);
}


void BaseController::makePolygonCircle(std::vector<geometry_msgs::Point32>& points, float x_center, float y_center, float radius) {

	int n_points = 30;
	float delta_theta = 2.0*M_PI/((float)n_points);

	float theta = 0;
	geometry_msgs::Point32 p;
	for (int i=0; i<n_points; ++i) {

		p.x = x_center + radius*cosf(theta);
		p.y = y_center + radius*sinf(theta);
		p.z = 0;
		points.push_back(p);

		theta += delta_theta;
	}
}

void BaseController::publishVisualizationMessages(float motion_x, float motion_y, std::string scan_frame, float collision_x, float collision_y) {

	// robot footprint
	geometry_msgs::PolygonStamped footprint;
	footprint.header.frame_id = _frame_in;
	footprint.header.stamp = ros::Time::now();
	makePolygonCircle(footprint.polygon.points, 0, 0, sqrtf(footprint_radius_squared));
	_footprint_pub.publish(footprint);

	// robot footprint after motion
	geometry_msgs::PolygonStamped motion_footprint;
	motion_footprint.header.frame_id = _frame_in;
	motion_footprint.header.stamp = ros::Time::now();
	makePolygonCircle(motion_footprint.polygon.points, motion_x, motion_y, sqrtf(footprint_radius_squared));
	_motion_footprint_pub.publish(motion_footprint);

	// predicted collision
	if (collision_x != 0 && collision_y != 0) {

		visualization_msgs::Marker collision;
		collision.ns = "predicted_collision";
		collision.id = 1;
		collision.type = visualization_msgs::Marker::SPHERE;
		collision.action = visualization_msgs::Marker::MODIFY;
		collision.lifetime = ros::Duration(2);

		// size
		collision.scale.x = 0.2;
		collision.scale.y = 0.2;
		collision.scale.z = 0.2;

		// color: RED
		collision.color.a = 1.0;
		collision.color.g = 0.0;
		collision.color.b = 0.0;
		collision.color.r = 1.0;

		// header
		collision.header.frame_id = scan_frame;
		collision.header.stamp = ros::Time::now();

		collision.pose.orientation.w = 1.0;
		collision.pose.position.x = collision_x;
		collision.pose.position.y = collision_y;
		collision.pose.position.z = 0.0;
		_collision_pub.publish(collision);
	}
}

bool BaseController::robot_collides(float x, float y, const sensor_msgs::LaserScan& scan, float &collision_x, float &collision_y) {

	std::vector<float>::const_iterator range;
	float theta = scan.angle_min;
	for (range = scan.ranges.begin(); range != scan.ranges.end(); ++range ) {

		float dx = x - (*range)*cosf(theta);
		float dy = y - (*range)*sinf(theta);
		float d2 = dx*dx + dy*dy;

		if (d2 <= footprint_radius_squared) {
			collision_x = (*range)*cosf(theta);
			collision_y = (*range)*sinf(theta);
			return true;
		}

		// update angle
		theta += scan.angle_increment;
	}
	return false;
}

void BaseController::callback_laser_scan(const sensor_msgs::LaserScan& scan) {

	if (last_sent_command_time == last_cmd_time) {
		return;
	}

	// predict motion result
	float x_base, y_base;
	sample_motion_model_velocity(last_cmd, x_base, y_base);

	// change in coordinates from base_frame to scan_frame
	/*bender_srvs::Transformer srv;
	srv.request.pose_in.header.stamp    = ros::Time::now();
	srv.request.pose_in.header.frame_id = _frame_in;
	srv.request.frame_out = scan.header.frame_id;
	srv.request.pose_in.pose.position.x = x_base;
	srv.request.pose_in.pose.position.x = y_base;
	srv.request.pose_in.pose.orientation.w = 1.0;
	try {
		_transformer_client.call(srv);
	} catch (std::exception &e) {
		ROS_WARN_STREAM("Cannot transform base position from frame "
			<< _frame_in << ", to frame " << scan.header.frame_id);
		return;
	}
	float x_scan = srv.response.pose_out.pose.position.x;
	float y_scan = srv.response.pose_out.pose.position.y;
	*/
	float x_scan = x_base - 0.245;
	float y_scan = y_base - 0.245;

	// check collision
	float collision_x = 0, collision_y = 0;
	if (robot_collides(x_scan, y_scan, scan, collision_x, collision_y)) {
		geometry_msgs::Twist stop_cmd;
		stop_cmd.linear.x = 0;
		stop_cmd.angular.z = 0;
		_cmd_pub.publish(stop_cmd);
	} else {
		_cmd_pub.publish(last_cmd);
	}
	last_sent_command_time = last_cmd_time;

	publishVisualizationMessages(x_base, y_base, scan.header.frame_id, collision_x, collision_y);
}

} /* namespace bender_follow_me */

int main(int argc, char **argv){

	ros::init(argc, argv, "base_controller");

	boost::scoped_ptr<bender_follow_me::BaseController> node(
				new bender_follow_me::BaseController(ros::this_node::getName())
	);

	// run at 15 Hz
	ros::Rate rate(15);
	while (ros::ok()) {

		ros::spinOnce();
		rate.sleep();
	}
	printf("\nQuitting... \n\n");

	return 0;
}

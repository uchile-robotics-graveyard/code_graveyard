/*
 * CocktailParty.cpp
 *
 *  Created on: January 2014
 *      Author: matias.pavez.b@gmail.com
 */

#include "Stage2/CocktailParty/CocktailParty.h"

namespace bender_behaviors {

CocktailParty::CocktailParty(std::string name): _name(name) {

	// - - - v a r i a b l e s - - -
	priv = ros::NodeHandle("~");
	std::string semantic_map = "amtc.sem_map";
	ros::ServiceClient nav_sem_map_load_client;
	bender_srvs::SemMap load_srv;

	
	// - - - - - - - - S e r v i c e   C l i e n t s  - - - - - - - - -

	// arm
	_grasp_object_client = priv.serviceClient<bender_srvs::armGoal>("/bender_macros/grasp_object");

	// navigation
	//nav_sem_map_load_client = priv.serviceClient<bender_srvs::SemMap>("/semantic_map_server/load");
	//_nav_sem_map_get_client = priv.serviceClient<bender_srvs::SemMap>("/semantic_map_server/get");
	//_nav_go_to_client = priv.serviceClient<bender_srvs::NavGoal>("/goalServer/go");
	//_nav_approach_client = priv.serviceClient<bender_srvs::NavGoal>("/goalServer/approach");
	//_nav_arrived_client = priv.serviceClient<bender_srvs::NavGoal>("/goalServer/has_arrived");

	// speech
   // _speech_synthesizer_client = priv.serviceClient<bender_srvs::synthesize>("/speech_synthesizer/synthesize");
//	_request_drink_client = priv.serviceClient<bender_srvs::stringReturn>("/interaction_speech/request_drink");


    // vision
	//_door_open_detector_client = priv.serviceClient<bender_srvs::DoorDetector>("/door_open_detector/isopen");
	_find_order_client = priv.serviceClient<bender_srvs::objrecService>("/bender_macros/find_object");


	// macros
	//_search_waving_client = priv.serviceClient<bender_srvs::SearchPerson>("/search_waving/search");
//	_enroll_person_client = priv.serviceClient<bender_srvs::stringReturn>("/enroll_person/enroll");


	// - - - - - - - - w a i t   f o r   s e r v i c e s  - - - - - - - - -

	// arm
	while ( ros::ok() && !_grasp_object_client.waitForExistence(ros::Duration(3.0)) );

	// navigation
	/*while ( ros::ok() && !_nav_approach_client.waitForExistence(ros::Duration(3.0)) );
	while ( ros::ok() && !_nav_sem_map_get_client.waitForExistence(ros::Duration(3.0)) );
	while ( ros::ok() && !_nav_arrived_client.waitForExistence(ros::Duration(3.0)) );
	while ( ros::ok() && !_nav_go_to_client.waitForExistence(ros::Duration(3.0)) );
	while ( ros::ok() && !nav_sem_map_load_client.waitForExistence(ros::Duration(3.0)) );*/

	// speech
	while ( ros::ok() && !_speech_synthesizer_client.waitForExistence(ros::Duration(3.0)) );
	//while ( ros::ok() && !_request_drink_client.waitForExistence(ros::Duration(3.0)) );

	// vision
	//while ( ros::ok() && !_door_open_detector_client.waitForExistence(ros::Duration(3.0)) );
	while ( ros::ok() && !_find_order_client.waitForExistence(ros::Duration(3.0)) );

	// macros
	//while ( ros::ok() && !_search_waving_client.waitForExistence(ros::Duration(3.0)) );
	//while ( ros::ok() && !_enroll_person_client.waitForExistence(ros::Duration(3.0)) );
	

	// Initialize stuff
	ROS_INFO_STREAM("[" << _name << "]: Initializing semantic map: " << semantic_map);
	load_srv.request.id = semantic_map;
//	nav_sem_map_load_client.call(load_srv);

    ROS_INFO_STREAM("[" << _name << "]: Running . . .");
}

CocktailParty::~CocktailParty() {
}

// TODO: mejorar talk
void CocktailParty::talk(std::string sentence) {

	ROS_INFO_STREAM("[" << _name << "]: Talking:" << sentence);
	bender_srvs::synthesize _synt_srv;
	_synt_srv.request.text = sentence;
	_speech_synthesizer_client.call(_synt_srv);

	double timeout = 2.0;
	double elapsed_time = 0.0;
	bool _is_talking = false;
	while (!_is_talking) {
		ros::Duration(1.0).sleep();
		priv.getParam("/bender/talking",_is_talking);
		elapsed_time+=1.0;

		if (elapsed_time >= timeout) {
			elapsed_time = 0;
			break;
		}

	}
	while(_is_talking) {
		ros::Duration(1.0).sleep();
		priv.getParam("/bender/talking",_is_talking);
		elapsed_time +=1.0;

		if (elapsed_time >= timeout) {
			elapsed_time = 0;
			break;
		}
	}
}

void CocktailParty::waitForOpenDoor(int door_distance) {

	bender_srvs::DoorDetector srv;
	srv.request.distance = door_distance;

	_door_open_detector_client.call(srv);
	while (srv.response.door_opened == false) {
		_door_open_detector_client.call(srv);
		ros::Duration(0.5).sleep();
	}

	// wait for a fully opened door
	ros::Duration(2.0).sleep();
}

void CocktailParty::arenaEntrance(double walk_velocity, double walk_time) {

	ros::Publisher base = priv.advertise<geometry_msgs::Twist>("/cmd_vel",1, this);
	geometry_msgs::Twist order;

	// go ahead
	order.linear.x = walk_velocity;
	base.publish(order);

	// walk for some time
	ros::Duration(walk_time).sleep();

	// then stop
	order.linear.x = 0;
	base.publish(order);
	base.publish(order);
	base.publish(order);
	base.publish(order);
	base.publish(order);
}

void CocktailParty::setInitialPose(std::string place) {

	ros::Publisher initial_pose_pub;
	bender_srvs::SemMap get_srv;
	geometry_msgs::PoseWithCovarianceStamped initial_pose;

	initial_pose_pub = priv.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10,this);

	// get required pose from semantic map server
	get_srv.request.id = place;
	_nav_sem_map_get_client.call(get_srv);

	// publish initial pose
	initial_pose.header.stamp = ros::Time::now();
	initial_pose.header.frame_id = get_srv.response.data[0].frame_id;
	initial_pose.pose.pose = get_srv.response.data[0].pose;

	ROS_INFO_STREAM("initialpose: (x,y):" << get_srv.response.data[0].pose.position.x << ", " << get_srv.response.data[0].pose.position.y);

	initial_pose.pose.covariance[0] = 0.25;
	initial_pose.pose.covariance[7] = 0.25;
	initial_pose.pose.covariance[35] = 0.06853891945200942;

	initial_pose_pub.publish(initial_pose);
}

void CocktailParty::goToPose(std::string place) {

	bender_srvs::SemMap get_srv;
	bender_srvs::NavGoal go_srv;
	bender_srvs::NavGoal arrived_srv;
	double timeout = 7.0;  // TODO: parameter server
	double elapsed_time = 0;

	// get required pose from semantic map server
	get_srv.request.id = place;
	_nav_sem_map_get_client.call(get_srv);

	// send pose to navigation server
	go_srv.request.goal.header.stamp = ros::Time::now();
	go_srv.request.goal.header.frame_id = get_srv.response.data[0].frame_id;
	go_srv.request.goal.pose = get_srv.response.data[0].pose;
	_nav_go_to_client.call(go_srv);


	_nav_arrived_client.call(arrived_srv);
	while (arrived_srv.response.state != 3) {
		ros::Duration(0.5).sleep();
		elapsed_time += 0.5;
		if (elapsed_time >= timeout) {
			_nav_go_to_client.call(go_srv);
			ROS_WARN_STREAM("[" << _name << "]: Resseting goal '" << place <<
			 "' after timeout of " << timeout << " seconds");
			elapsed_time = 0;
		}
		_nav_arrived_client.call(arrived_srv);
	}
}

void CocktailParty::approachTo(bender_macros::MacroPerson person) {

	bender_srvs::NavGoal approach_srv;
	bender_srvs::NavGoal arrived_srv;
	double timeout = 40.0;  // TODO: parameter server
	double elapsed_time = 0;

	ROS_INFO_STREAM("approach: person pose: (x,y):" << person.getPose().pose.position.x << ", " << person.getPose().pose.position.y);

	approach_srv.request.goal = person.getPose();
	_nav_approach_client.call(approach_srv);

	_nav_arrived_client.call(arrived_srv);
	while (arrived_srv.response.state != 3) {
		ros::Duration(0.5).sleep();
		elapsed_time += 0.5;
		if (elapsed_time >= timeout) {
			_nav_approach_client.call(approach_srv);
			ROS_WARN_STREAM("[" << _name << "]: Reseting approach goal after timeout of "
			 << timeout << " seconds");
			elapsed_time = 0;
		}
		_nav_arrived_client.call(arrived_srv);
	}
}

void CocktailParty::searchWaving(bender_macros::MacroPerson& person) {

	bender_srvs::SearchPerson search_srv;

	_search_waving_client.call(search_srv);

	while (search_srv.response.found == false) {

		_search_waving_client.call(search_srv);
	}
	ROS_INFO_STREAM("person pose: (x,y):" << search_srv.response.person.pose.position.x << ", " << search_srv.response.person.pose.position.y);

	person.setPose(search_srv.response.person);
}

void CocktailParty::enroll(bender_macros::MacroPerson& person) {

	bender_srvs::stringReturn enroll_srv;
	//TODO: enroll with desired ID or get ID from enroll Service

	_enroll_person_client.call(enroll_srv);
	person.setName(enroll_srv.response.data);

	ROS_INFO_STREAM("[" << _name << "]: New Person: (id,name) =  (" << person.getId() << "," << person.getName() << ")");
}

void CocktailParty::askOrder(bender_macros::MacroPerson &person) {

	bender_srvs::stringReturn order_srv;

	_request_drink_client.call(order_srv);
	person.setOrder(order_srv.response.data);

	ROS_INFO_STREAM("[" << _name << "]: New order '" << person.getOrder() << "' for person: " << person.getName());
}

void CocktailParty::searchOrder(std::string order, geometry_msgs::Point &object_position) {

	bender_srvs::objrecService object_request;

	object_request.request.object = order;
	_find_order_client.call(object_request);
	while( object_request.response.find == false) {
		_find_order_client.call(object_request);
	}

	object_position.x = object_request.response.object.pose.position.x;
	object_position.y = object_request.response.object.pose.position.y;
	object_position.z = object_request.response.object.pose.position.z;

	ROS_INFO_STREAM("[" << _name << "]: Found '" << order << "' at (x,y,z):("
		<< object_position.x << "," << object_position.y << "," <<
		object_position.z << ")"
	);
}

void CocktailParty::graspObject(geometry_msgs::Point object_position) {

	bender_srvs::armGoal grasp_srv;
	grasp_srv.request.object.pose.position = object_position;

	_grasp_object_client.call(grasp_srv);
}




} /* namespace bender_behaviors */

int main(int argc, char** argv) {

	ros::init(argc, argv, "CocktailParty");

	boost::scoped_ptr<bender_behaviors::CocktailParty> behavior(
			new bender_behaviors::CocktailParty(ros::this_node::getName())
	);

	std::map<int,bender_macros::MacroPerson> person_map;
	int current_id = 100;
	bender_macros::MacroPerson person;
	geometry_msgs::Point object_position;
	person.setId(current_id);

	// TODO
	//behavior->setInitialPose("start");
/*
	behavior->talk("waiting the door signal");
	behavior->waitForOpenDoor(100);

	behavior->talk("Entering the arena");
	behavior->arenaEntrance(0.3, 6);
	
	behavior->talk("i am going to the search position");
	behavior->goToPose("living_room");

	// TODO: keep looking toward person
	behavior->talk("Looking for people calling me");
	behavior->searchWaving(person);

	behavior->talk("Person found, i am going to you");
	behavior->approachTo(person);
*/
	//behavior->talk("Enrolling");
	//behavior->enroll(person);

	//behavior->talk("Requesting order");
	//behavior->askOrder(person);

	//person_map[current_id] = person;

	//behavior->talk("i am going to the kitchen");
	//behavior->goToPose("kitchen");

	//ros::Duration(5.0).sleep();
	//behavior->talk("i reach the kitchen");



	//behavior->talk("approaching to table");
	// TODO: approach to table

	person.setOrder("sprite");
	behavior->talk("searching order");
	behavior->searchOrder(person.getOrder(),object_position);

	behavior->talk("grasping drink");
	behavior->graspObject(object_position);

	//behavior->talk("returning to person");
	//behavior->goToPose("living_room");
	//behavior->approachTo(person);

	behavior->talk("say your name motherfucker");

	ROS_INFO("Quitting ... \n");
	return 0;
}

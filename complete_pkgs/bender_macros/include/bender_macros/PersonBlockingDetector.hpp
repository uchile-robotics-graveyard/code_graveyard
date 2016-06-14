/*
 * PersonBlockingDetector.h
 *
 *      Author: matias.pavez.b@gmail.com
 */

#ifndef PERSONBLOCKINGDETECTOR_H_
#define PERSONBLOCKINGDETECTOR_H_

// C, C++
#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <vector>

// ROS
#include <ros/ros.h>
#include <bender_utils/ParameterServerWrapper.h>
#include <tf/transform_listener.h>
#include <bender_laser/LegDetector.hpp>

// ROS services / messages
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include <bender_srvs/PoseStamped.h>
#include <bender_msgs/PoseDetections.h>
#include <bender_msgs/PathBlockingDetections.h>

namespace bender_macros {

class PersonBlockingDetector {

public:

    std::string server_name;

private:

    geometry_msgs::PoseStamped _current_goal;
    nav_msgs::Path _current_plan;
    std::vector<std::string> _sources;
    std::vector<geometry_msgs::PoseStamped> _detections;
    bool _received_first_plan;
    bool _received_first_goal;

    // Parameters
    std::string _map_frame;
    std::string _robot_frame;
    float _detection_lifetime;
    float _plan_tolerance;
	float _person_radius;
	float _blocker_to_path_radius;
	float _path_length_interact_th;

    // Service Clients
    ros::ServiceClient _get_plan_client;

    // Listeners
    ros::Subscriber _new_goal_sub;
    ros::Subscriber _new_plan_sub;
    ros::Subscriber _leg_detections_sub;
    ros::Subscriber _person_detections_sub;
    tf::TransformListener _tf_listener;

    // Publishers
    ros::Publisher _blocking_pub;
    ros::Publisher _blocking_markers_pub;

public:
    PersonBlockingDetector();
    virtual ~PersonBlockingDetector();

    void spinOnce();

private:

    enum obstacle_t { CANDIDATE, OBSTACLE, OTHER };

    // main algorithm methods
    /**
     * deletes old cached detections
     */
    void deleteOldDetections();

    /**
     * returns the index of nearest plan segment to the robot (the index of the second point)
     */
    void getRobotPositionOnPath(int &segment_index, const geometry_msgs::PoseStamped &robot);

    /**
     * returns the path length, starting from the provided segment index (the index of it's second point)
     */
    static float getPathLength(const nav_msgs::Path& path, const int& starting_idx);

    // ROS interaction
    /**
     * returns the robot pose in the frame map
     */
    bool getCurrentPoseInMap(geometry_msgs::PoseStamped& robot);

    /**
     * publishes detection results
     */
    void publish_blockers(const std::vector<geometry_msgs::PoseStamped>& blockers, const std::vector<PersonBlockingDetector::obstacle_t>& blocking_states, bool should_interact);

    /**
     * request move_base to generate(and execute) a new plan for navigation
     */
    bool request_plan(nav_msgs::Path& plan, const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, const float tolerance);

    // distance metrics
    /**
     * returns the euclidean distance between 2 points
     */
    static float distancePointPoint(const float &Ax, const float &Ay, const float &Bx, const float &By);

    /**
     * returns the euclidean distance between a point C and a line segment AB
     *
     * if P is the projection of point C on the segment AB, and:
     * - P is interior to AB                 , then returns distance(C,P)
     * - P is on the backward extension of AB, then returns distance(A,C)
     * - P is on the forward  extension of AB, then returns distance(B,C)
     */
    static float distancePointSegment(const geometry_msgs::PoseStamped &C, const geometry_msgs::PoseStamped &A, const geometry_msgs::PoseStamped &B);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // - - - - - - - S u b s c r i b e r   C a l l b a c k s  -  - - - - - - - - - -
    void callback_newGoal(const geometry_msgs::PoseStamped new_goal);

    void callback_newPlan(const nav_msgs::Path &path);

    void callback_leg_detections(const bender_msgs::PoseDetections &msg);

    void callback_person_detections(const bender_msgs::PoseDetections &msg);

};

} /* namespace bender_macros */
#endif /* PERSONBLOCKINGDETECTOR_H_ */

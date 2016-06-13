#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from std_srvs.srv import EmptyRequest
from std_srvs.srv import EmptyResponse

from nav_msgs.msg import * 
from sensor_msgs.msg import *
from bender_srvs.srv import *
from math import sqrt, atan2, sin, cos

path = [Path]
points = PointCloud().points

def approachCallback(pointcloud):
    global points, mean_x, mean_y
    mean_x = 0
    mean_y = 0
    points = pointcloud.points
    for p in points:
        mean_x = mean_x + p.x
        mean_y = mean_y + p.y
    
    mean_x = mean_x/len(points)
    mean_y = mean_y/len(points)
    print "new points, centro en ("+ str(mean_x) + ", " + str(mean_y) + ")" 
        
def planCallback(pointpath):
    global path    
    path = pointpath.poses
    #print "new path"

def recalculateGoalCallback(request):
    global path, points, mean_x, mean_y
    
    for path_point in path:
        for cloud_point in points:
            
            cloud_x = cloud_point.x
            path_x = path_point.pose.position.x
            cloud_y = cloud_point.y
            path_y = path_point.pose.position.y
           
            _dx = abs(cloud_x - path_x)
            _dy = abs(cloud_y - path_y)
            _min_dist = sqrt(pow(_dx, 2)+pow(_dy, 2))
                   
            if _min_dist < 0.15:
                
                theta = atan2(mean_x - path_x,mean_y - path_y)
    
                path_point.pose.orientation.z = sin(theta);
                path_point.pose.orientation.w = cos(theta);
                                
                print _min_dist
                try:
                    go = rospy.ServiceProxy('/bender/nav/goal_server/go', NavGoal)
                    resp = go(path_point, 0)
                    return EmptyResponse()
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
                
    return EmptyResponse()

def recalculateGoal():
    
    rospy.init_node('recalculate_goal')
    
    s = rospy.Service('recalculateGoal', Empty, recalculateGoalCallback)
    
    rospy.Subscriber('/bender/nav/move_base/NavfnROS/plan', Path, planCallback)
    rospy.Subscriber('/bender/nav/goal_server/points_approach', PointCloud, approachCallback)
    
    rospy.spin()
   
    
# main
if __name__ == '__main__':
    recalculateGoal()
    
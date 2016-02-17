#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_follow_me')
import rospy
import smach
import smach_ros
import math

from bender_follow_me.msg import TrackingState
from bender_msgs.msg import *
from bender_srvs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
import bender_srvs


# behavior parameters
#debug = True
track_id = -1
track_x = 0
track_y = 0
track_heading = 0
track_vel = 0
initilized = False

# controller parameters
# - - - - - - - - - - - - - - - - - - - - - - 

# gains
Kp_linear  = 0.8 # linear  gain
Kp_angular = 0.5 # angular gain

# goal
goal_linear  = 1.2 # [m]
goal_angular = 0.0 # [deg]

# saturation
max_linear_speed  = 0.25 #[m/s]
max_angular_speed = 0.35 #[rad/s]

# histeresis
histeresis_threshold_linear  = 0.2 # [m]
histeresis_threshold_angular = 10  # [deg]


#funciones utiles
#def talk(text):
#    talker = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
#    talker(text)
#    debug_print(text)
#talk("Ok, I will start following you now.")
#rospy.sleep(4.0)



# ROS
# - - - - - - - - - - - - - - - - - - - - - - 

nav_velocity_pub = rospy.Publisher('/bender/follow_me/behavior_cmd', Twist, queue_size=1)


def get_control_cmd(person_x, person_y):

    # compute error
    d_linear  = math.sqrt(person_x*person_x + person_y*person_y) - goal_linear
    d_angular = math.atan2(person_y, person_x)*180.0/math.pi     - goal_angular
    rospy.loginfo('d_linear=' + str(d_linear) + ', d_angular=' + str(d_angular))

    # histeresis: linear
    if abs(d_linear) < histeresis_threshold_linear:
        d_linear = 0

    # histeresis: angular
    if abs(d_angular) < histeresis_threshold_angular:
        d_angular = 0

    # control command
    cmd_linear  = Kp_linear*d_linear
    cmd_angular = Kp_angular*d_angular

    # saturate
    cmd_linear  = min(cmd_linear , max_linear_speed )
    cmd_angular = min(cmd_angular, max_angular_speed)
    cmd_linear  = max(cmd_linear , -max_linear_speed )
    cmd_angular = max(cmd_angular, -max_angular_speed)

    # final control command
    control_cmd = Twist()
    control_cmd.linear.x  = cmd_linear
    control_cmd.angular.z = cmd_angular
    
    return control_cmd

def move(vel_cmd):

    rospy.loginfo('command: (linear,angular)=(' + str(vel_cmd.linear.x) + ', ' + str(vel_cmd.angular.z) + ')')
    nav_velocity_pub.publish(vel_cmd)
    return

def tracking_callback(msg):

    vel_cmd = Twist()
    if len(msg.ids) > 0:

        track_id = msg.ids[0]
        track_x = msg.x_a[0]
        track_y = msg.y_a[0]
        track_theta = msg.theta_a[0]
        track_vel = msg.vel_a[0]

        vel_cmd = get_control_cmd(track_x, track_y)

    else:
        rospy.loginfo('Lost everyone')
        

    move(vel_cmd)
    return



def init_follow():
    
    rospy.loginfo('Initializing tracker . . .')
    init_tracker_client = rospy.ServiceProxy('/bender/follow_me/bayesian_filter/initialize_tracker', String)
    init_tracker_client.wait_for_service()
    init_tracker_client.call()
    
    get_init_id_client = rospy.ServiceProxy('/bender/follow_me/bayesian_filter/get_init_id', ID)
    init_tracker_client.wait_for_service()
    res = get_init_id_client.call()
    while (res.ID == -1):
        rospy.sleep(0.2); 
        
    track_id = res.ID
    
    rospy.loginfo(' . . . OK ')
    
    return

#def follow():
#    return

def main():
    
    rospy.init_node('follow_me')
    
    rospy.loginfo('Setting up node')
    
    # tracking
    rospy.Subscriber('/bender/follow_me/bayesian_filter/tracking', TrackingState, tracking_callback)
    
    
    rospy.loginfo('Started')
    init_follow()
    
    
    
 #   follow()
    rospy.spin()

    
if __name__ == '__main__':
    main()

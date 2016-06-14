#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_srvs.srv import TableDetector
from bender_srvs.srv import TableDetectorResponse
from bender_srvs.srv import Onoff
from geometry_msgs.msg import Twist
from bender_utils.ros import benpy


# --- macros ---
from bender_macros.nav import GoToPoseStamped
from bender_macros.nav import GoalFromPlace
from bender_macros.head import MoveAsus

# TODO: tune controller parameters: gains, saturation, thresholds, references
# TODO: use saturation  for accelerations ('continuous' velocity steps)
# TODO: improve 'not detection' behavior

class GetCloser(smach.State):
        
    def __init__(self):
        
        smach.State.__init__(self, 
            outcomes=['succeeded','aborted','preempted'],
            input_keys=['distance'])
        
        
        # - - - - - Controller data - - - - - - -
        #
        # A.- references
        # - table distance
        # - table orientation        
        # 
        # B.- sensors (see rostopic hz /topic)
        # - sensor: rgbd_head works at 29.5 [Hz]
        # - sensor: table detector works at 29 [Hz]
        # - controller: 25 [Hz]
        #
        # C.- Controller
        # - P controller
        
        # references
        self.min_dref = 0.40 # very important!.. we dont want to collide!
                             # this should be the robot radius

        self.angle_ref = 0.0
        
        # controller gains
        self.angular_kp = 1
        self.linear_kp = 0.6

        # saturation parameters
        self.max_linear_vel = 0.25
        self.min_linear_vel = -0.25
        self.max_angular_vel = 0.3
        self.min_angular_vel = -0.3
        
        # convergence checking
        self.linear_th = 0.03                 # 3 [cm]
        self.angular_th = 2.5*3.1415/180.0    # 2.5 [degrees]
        self.req_convergence_times = 5
        
        # detection parameters
        self.min_detection_rate = 70;
        
        # publishers
        self.cmd_vel_pub = rospy.Publisher('/bender/nav/cmd_vel', Twist)

        # clients        
        self.table_detector_enable_client = benpy.ServiceProxy('/bender/pcl/table_detector/enable', Onoff)
        self.table_detector_client = benpy.ServiceProxy('/bender/pcl/table_detector/detection_state', TableDetector)
        
    def execute(self,ud):
        
        distance_ref = ud.distance
        
        if ud.distance < self.min_dref:
            rospy.logwarn("Invalid reference distance. Distance too small. Setting up to: " + str(self.min_dref) + "[m]")
            distance_ref = max(distance_ref, self.min_dref)
        
        rospy.loginfo('Executing state Approach to plane')
        
        # enable table detector
        ''' OBS: first N detections will have lower 
        detection rate than 'min_detection_rate'! '''
        self.table_detector_enable_client.wait_for_service()
        self.table_detector_enable_client(True)
        #self.table_detector_client.wait_for_service()
        #print "WAITING OK!!!!!!!!!!!!!!!!"
        
        # convergence counter
        convergence_times = 0
        
        #a#bad_cnt = 0
        
        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            
            try:
                detect_res = self.table_detector_client()
            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))

            # manage convergence counter 
            convergence_times = max(convergence_times - 1,0)

            if (detect_res.rate < self.min_detection_rate):
                
                #a#bad_cnt += 1
                #a#print 'BAD DETECTION #', str(bad_cnt)
                
                # send stop command
                self.cmd_vel_pub.publish(Twist())
                # print "stop"
                
            else:
                # errors      
                linear_error = distance_ref - detect_res.distance
                angular_error = self.angle_ref - detect_res.angle

                # check convergence         
                if (abs(linear_error) < self.linear_th and abs(angular_error) < self.angular_th):
                    
                    convergence_times += 2
                    print '[state GET_CLOSER]: In range . . . %3d/%3d' % (convergence_times, self.req_convergence_times)
                    
                    # we are OK
                    if (convergence_times >= self.req_convergence_times):
                        print '[state GET_CLOSER]: Done :)'
                        break

                # controller
                cmd = Twist()
                cmd.linear.x  = -self.linear_kp*(linear_error) 
                cmd.angular.z = -self.angular_kp*(angular_error)
                
                # saturation
                cmd.linear.x = min(cmd.linear.x,self.max_linear_vel)
                cmd.linear.x = max(cmd.linear.x,self.min_linear_vel)
                cmd.angular.z = min(cmd.angular.z,self.max_angular_vel)
                cmd.angular.z = max(cmd.angular.z,self.min_angular_vel)

                # send control action
                self.cmd_vel_pub.publish(cmd)
                #print "[state GET_CLOSER]: Sending control action"
                
            # send control commands at the required frequency
            rate.sleep()

        # send stop command
        self.cmd_vel_pub.publish(Twist())

        # disable table detector
       # self.table_detector_enable_client.wait_for_service()
        self.table_detector_enable_client(False)
        
        return 'succeeded'
    
    
def getInstance():
    
    sm = smach.StateMachine(
            outcomes = ['succeeded','aborted','preempted'],
            input_keys = ['distance']
    )

    with sm:
        
        smach.StateMachine.add('SETUP', MoveAsus.getTableApproachInstance(),
            transitions = {'succeeded':'GET_CLOSER',
                           'aborted':'GET_CLOSER'}
        )
        
        smach.StateMachine.add('GET_CLOSER', GetCloser(),
            remapping={'distance':'distance'})
       
    return sm
 
# main
if __name__ == '__main__':

    rospy.init_node('approach_to_plane')

    sm = getInstance()

    # generate dummy userdata
    ud = smach.UserData()
    ud.distance = 0.4
    ud.map_name = 'map.sem_map'

    # introspection server
    sis = smach_ros.IntrospectionServer('approach_to_plane', sm, '/APPROACH_TO_PLANE_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
 
    

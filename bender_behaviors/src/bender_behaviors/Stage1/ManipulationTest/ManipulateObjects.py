#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import time
import smach
import smach_ros
import cv2
import rospy, rospkg
import yaml, genpy
from bender_srvs.srv import *
from bender_msgs.msg import *
from geometry_msgs.msg import *
from bender_srvs.srv import String as string_srv

from bender_macros.speech import Talk
from bender_macros.skills import OctomapShelf

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# from bender_macros.arm import GraspObject_old
# from bender_macros.arm import LeaveObject_old
from bender_macros.arm import GraspCartesian_shelf
from bender_macros.skills import PlaceObjectShelf


class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                    io_keys = ['leave_obj1','leave_obj2','leave_obj3','leave_obj4','leave_obj5','selected_pose'])
        self.x = [0.45]
        self.y = [0.0, 0.2,-0.2,-0.4,-0.4]
        self.z = [1]

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')

        userdata.leave_obj1.pose.position.x = self.x[0] 
        userdata.leave_obj2.pose.position.x = self.x[0] 
        userdata.leave_obj3.pose.position.x = self.x[0] 
        userdata.leave_obj4.pose.position.x = self.x[0] 
        userdata.leave_obj5.pose.position.x = self.x[0] 

        userdata.leave_obj1.pose.position.y = self.y[0] 
        userdata.leave_obj2.pose.position.y = self.y[1] 
        userdata.leave_obj3.pose.position.y = self.y[2] 
        userdata.leave_obj4.pose.position.y = self.y[3] 
        userdata.leave_obj5.pose.position.y = self.y[4] 

        userdata.leave_obj1.pose.position.z = self.z[0] 
        userdata.leave_obj2.pose.position.z = self.z[0] 
        userdata.leave_obj3.pose.position.z = self.z[0] 
        userdata.leave_obj4.pose.position.z = self.z[0] 
        userdata.leave_obj5.pose.position.z = self.z[0] 

        # p = PoseStamped()
        # p.header.frame_id = 'bender/base_link' #'bender/sensors/rgbd_head_rgb_optical_frame'
        # p.pose.position.x,p.pose.position.y,p.pose.position.z = 0.60, 0.25, 0.75
        # p.pose.orientation.w = 1

        # userdata.selected_pose = p

        return 'succeeded'


class Selection(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['finish','succeeded','aborted','preempted'],
        input_keys = ['selected_objects_name','selected_objects_pose','selected_id','leave_obj1','leave_obj2','leave_obj3','leave_obj4','leave_obj5'],
        output_keys = ['selected_object','selected_pose','selected_id','selected_leavepose'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')

        sid = userdata.selected_id
        print sid
        try:
            if userdata.selected_objects_name[sid+1] == "":   
                return 'finish'
        except rospy.ServiceException, e:
            return 'finish'

        userdata.selected_id += 1
        userdata.selected_object = userdata.selected_objects_name[userdata.selected_id]
        userdata.selected_pose = userdata.selected_objects_pose[userdata.selected_id]
        print userdata.selected_objects_name[userdata.selected_id]
        print userdata.selected_objects_pose[userdata.selected_id]

        i = userdata.selected_id%5
        if i == 0: 
            userdata.selected_leavepose = userdata.leave_obj1
        if i == 1: 
            userdata.selected_leavepose = userdata.leave_obj2
        if i == 2: 
            userdata.selected_leavepose = userdata.leave_obj3
        if i == 3: 
            userdata.selected_leavepose = userdata.leave_obj4
        if i == 4: 
            userdata.selected_leavepose = userdata.leave_obj5

        return 'succeeded'

class AnnounceObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                            input_keys = ['selected_object'],
                            io_keys = ['h','r'])

        self.cont = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')

        sp = "I will try to manipulate the "+userdata.selected_object
        Talk.getInstance(sp,len(sp)/9)

        userdata.h, userdata.r = self.extract_inf_object(userdata.selected_object)

        return 'succeeded'

    def extract_inf_object(self, name):
        rospack = rospkg.RosPack()
        obj_path = rospack.get_path('bender_utils')+'/config/mapper/manipulation.yaml'
        obj_data = [0.04,0.1]
        try:
            with open(obj_path, 'r') as f:
                # load all documents
                obj_data = yaml.load(f)
                if obj_data is None:
                    raise yaml.YAMLError("Empty files not allowed")

                # rellena los checkpoints con las trajectorias predefinidas
                #print traj_data['header']
                print name+" found in yaml file"

        except yaml.YAMLError as e:
            rospy.logerr('Invalid YAML file: %s' % (str(e)))

        return obj_data[name][0], obj_data[name][1]

class ApologizeForGrasp(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
        input_keys = ['selected_object'])
        self.cont = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')

        sp = "I'm sorry, but I could not manipulate the "+userdata.selected_object
        Talk.getInstance(sp,len(sp)/9)

        return 'succeeded'

class ApologizeForGrasp(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
        input_keys = ['selected_object'])
        self.cont = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')

        sp = "I'm sorry, but I could not manipulate the "+userdata.selected_object
        Talk.getInstance(sp,len(sp)/9)

        return 'succeeded'

def getInstance():

    sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'],
        input_keys = ['selected_objects_name','selected_objects_pose'],
        output_keys = ['selected_manipulate'])
    
    
    sm.userdata.selected_object = "none"
    sm.userdata.selected_pose = PoseStamped()
    sm.userdata.selected_id = -1
    sm.userdata.selected_manipulate =[]

    sm.userdata.leave_obj1 = PoseStamped()    
    sm.userdata.leave_obj2 = PoseStamped()  
    sm.userdata.leave_obj3 = PoseStamped()  
    sm.userdata.leave_obj4 = PoseStamped()  
    sm.userdata.leave_obj5 = PoseStamped()  
    sm.userdata.selected_leavepose = PoseStamped()
    sm.userdata.lr_arm = 'r'
    sm.userdata.h = 0.1
    sm.userdata.r = 0.04

    with sm:
        smach.StateMachine.add  ('SETUP', Setup(),
          transitions  =  {'succeeded':'ACTUALIZE_OCTOMAP'}
        )
        smach.StateMachine.add  ('ACTUALIZE_OCTOMAP', OctomapShelf.getInstance(),
          transitions  =  {'succeeded':'SELECTION'}
        )      
        smach.StateMachine.add  ('SELECTION', Selection(),
          transitions  =  {'succeeded':'ANNOUNCE_OBJECT',
                            'finish':'succeeded'}
        )
        smach.StateMachine.add  ('ANNOUNCE_OBJECT' , AnnounceObject(),
          transitions  =  {'succeeded':'GRASP_OBJECT'}
                  )
        smach.StateMachine.add  ('GRASP_OBJECT' , GraspCartesian_shelf.getInstance(),
              transitions  =  {'succeeded':'LEAVE_OBJECT',
                           'notgrabbing':'APOLOGIZE_FOR_GRASP'},
            remapping   =   {'pose':'selected_pose',
                            'height':'h',
                            'radius':'r',
                            'lr_arm':'lr_arm'}
                  )
        smach.StateMachine.add  ('APOLOGIZE_FOR_GRASP' , ApologizeForGrasp(),
          transitions  =  {'succeeded':'preempted'}
                  )
        smach.StateMachine.add  ('LEAVE_OBJECT' , PlaceObjectShelf.getInstance(),
          transitions  =  {'succeeded':'SELECTION'},
            remapping   =   {'place_pose':'selected_leavepose',
                             'lr_arm':'lr_arm'}
                  )

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('manipulateobjects')

    sm = getInstance()
    ud = smach.UserData()

    # genera userdata de prueba
    p = PoseStamped()
    p.header.frame_id = 'bender/base_link' #'bender/sensors/rgbd_head_rgb_optical_frame'
    p.pose.position.x,p.pose.position.y,p.pose.position.z = 0.58, 0.15, 0.7
    p.pose.orientation.w = 1
    
    ud.selected_objects_pose = [p]
    ud.selected_objects_name = ["musculo"]

    sis = smach_ros.IntrospectionServer('manipulateobjects', sm, '/MANIPULATEOBJECTS_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()

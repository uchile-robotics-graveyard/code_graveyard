#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from std_srvs.srv import *
from geometry_msgs.msg import *
from bender_srvs.srv import *
from bender_msgs.msg import *

from bender_macros.vision import CheckFaceInFront
from bender_macros.nav import GoToPoseStamped
from bender_macros.nav import GoalFromPlace
from bender_macros.nav import ApproachToPoseStamped

class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.person_detector_client = rospy.ServiceProxy('/bender/pcl/hog_person_detector/enable', Onoff)
        rospy.loginfo('Waiting for service: ' + self.person_detector_client.resolved_name)
        self.person_detector_client.wait_for_service()
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')
        
        rospy.loginfo('Waiting for service: ' + self.person_detector_client.resolved_name)
        self.person_detector_client.wait_for_service()
        rospy.loginfo('Server ready: ' + self.person_detector_client.resolved_name)
        
        # enable person detector
        self.person_detector_client(True)
        return 'succeeded'

class UnsetupMachine(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.person_detector_client = rospy.ServiceProxy('/bender/pcl/hog_person_detector/enable', Onoff)
        rospy.loginfo('Waiting for service: ' + self.person_detector_client.resolved_name)
        self.person_detector_client.wait_for_service()
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')
        
        rospy.loginfo('Waiting for service: ' + self.person_detector_client.resolved_name)
        self.person_detector_client.wait_for_service()
        rospy.loginfo('Server ready: ' + self.person_detector_client.resolved_name)
        
        # enable person detector
        self.person_detector_client(False)
        
        return 'succeeded'

class WaitingState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: WAIT A WHILE')
        rospy.sleep(1.0)
        return 'succeeded'

class WaitFreePass(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: WAIT FREE PATH')
        
        # wait 4 seconds to continue
        #rospy.sleep(4.0)
        
        return 'succeeded'


class BlockerInteraction(smach.State):
 
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
            io_keys=['end_interaction_time']
        )
        
        self.synt_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize', synthesize)
        rospy.loginfo('Waiting for service: ' + self.synt_client.resolved_name)
        self.synt_client.wait_for_service()
                  
    def execute(self, userdata):
        rospy.loginfo('Executing state Interaction With Blocking User')
        
        self.synt_client.wait_for_service()
        try:
            resp = self.synt_client('excuse me, may i pass?')
        except Exception, e:
            rospy.logwarn('failed to cancel the goal')
        
        # wait speech
        #talking = rospy.get_param('/bender/speech/synthesizer/talking')
        rospy.sleep(3.0)
        
        
        # wait for the person to move
        rospy.sleep(5.0)
        
        # mark last interaction time
        userdata.end_interaction_time = rospy.Time.now()
        
        return 'succeeded'


def getInstance():
    
    sm_trigger = smach.StateMachine(outcomes=['succeeded','aborted','preempted','repeat'])
    sm_trigger.userdata.blocker_pose = geometry_msgs.msg.PoseStamped()
    sm_trigger.userdata.last_triggered_time = rospy.Time.now()
    sm_trigger.userdata.face_pose = geometry_msgs.msg.PoseStamped()
    with sm_trigger:
        
        
        def trigger_cb(ud, msg):
            
            #rospy.logwarn('trigger in action!')
            min_retrigger_time = rospy.Duration(30.0)
            now = rospy.Time.now()
            if (now - ud.last_triggered_time) < min_retrigger_time:
                return True
            
            ud.blocker_pose = geometry_msgs.msg.PoseStamped()
            for blocking_type, blocker in zip(msg.info, msg.detections):
             
                if blocking_type == "obstacle":
                    
                    ud.blocker_pose.pose.position = blocker.position
                    ud.blocker_pose.pose.orientation.w = 1.0
                    ud.blocker_pose.header = msg.header
                    
                    return False
     
            return True
        
        smach.StateMachine.add('BLOCKING_MONITOR',
            smach_ros.MonitorState(
                "/bender/macros/person_blocking_detector/blocking_detections",
                bender_msgs.msg.PathBlockingDetections,
                trigger_cb,
                input_keys=['blocker_pose', 'last_triggered_time'],
                output_keys=['blocker_pose', 'last_triggered_time']                
            ),
            transitions={
                'invalid':'CANCEL_GOAL',
                'valid':'BLOCKING_MONITOR',
                'preempted':'preempted'
            },
            remapping={'blocker_pose':'blocker_pose'}
        )

        smach.StateMachine.add('CANCEL_GOAL',
            smach_ros.ServiceState('/bender/nav/goal_server/cancel', Empty),
            transitions = {'succeeded':'WAIT_A_WHILE'}
        )
        
        # used to ensure a proper handling of Navigation Goals
        smach.StateMachine.add('WAIT_A_WHILE', WaitingState(),
            transitions={'succeeded':'APPROACH_TO_BLOCKER',
                         'aborted':'aborted',
                         'preempted':'preempted'}
        )
        
        smach.StateMachine.add('APPROACH_TO_BLOCKER', ApproachToPoseStamped.getInstance(),
            transitions={'succeeded':'CHECK_PERSON_EXISTENCE',
                         'aborted':'BLOCKING_MONITOR',
                         'preempted':'preempted'},
            remapping={'goal_pose':'blocker_pose'}
        )
        
        smach.StateMachine.add('CHECK_PERSON_EXISTENCE', CheckFaceInFront.getInstance(),
            transitions={'succeeded':'INTERACT_WITH_BLOCKER',
                         'aborted':'BLOCKING_MONITOR',
                         'preempted':'preempted'},
            remapping={'face_pose':'face_pose'}
        )

        smach.StateMachine.add('INTERACT_WITH_BLOCKER', BlockerInteraction(),
            transitions={'succeeded':'WAIT_FREE_PATH',
                         'aborted':'BLOCKING_MONITOR',
                         'preempted':'preempted'},
            remapping={'end_interaction_time':'last_triggered_time'}
        )
        
        smach.StateMachine.add('WAIT_FREE_PATH', WaitFreePass(),
            transitions={'succeeded':'succeeded',
                         'aborted':'aborted',
                         'preempted':'preempted'}
        )
    
    # gets called when ANY child state terminates
    def interaction_child_term_cb(outcome_map):
    
      # terminate all running states if FOO finished with outcome 'outcome3'
      if outcome_map['BLOCKING_INTERACTIVE'] in ['succeeded','aborted']:
        return True
    
      # terminate all running states if BAR finished
      if outcome_map['GO_TO_PLACE_SM'] == 'succeeded':
        return True
    
      # TODO: terminate all running states if BAR finished
      #if outcome_map['GO_TO_PLACE_SM'] == 'aborted':
      #  return True
    
      # in all other case, just keep running, don't terminate anything
      return False
    
    
    # gets called when ALL child states are terminated
    def interaction_out_cb(outcome_map):
       
       if outcome_map['GO_TO_PLACE_SM'] == 'succeeded':
          return 'succeeded'
      
       #if outcome_map['BLOCKING_INTERACTIVE'] in ['succeeded','aborted']:
       #   return 'repeat'
       return 'repeat'

    
    sm_con = smach.Concurrence(
        outcomes=['succeeded','aborted','preempted','repeat'],
        default_outcome='repeat',
        input_keys=['place_name', 'map_name'],
        child_termination_cb = interaction_child_term_cb,
        outcome_cb = interaction_out_cb
    )
    go_to_place_sm = GoalFromPlace.getMachineInstance(GoToPoseStamped.getInstance())
    with sm_con:
        smach.Concurrence.add('GO_TO_PLACE_SM', go_to_place_sm,
            remapping={'place_name':'place_name',
                       'map_name':'map_name'})
        
        smach.Concurrence.add('BLOCKING_INTERACTIVE', sm_trigger)
        
        

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
        input_keys=['place_name', 'map_name']
    )
    with sm:
        
        smach.StateMachine.add('SETUP',Setup(),
            transitions={'succeeded':'INTERACTIVE_GO_TO_PLACE'})
    
        smach.StateMachine.add('INTERACTIVE_GO_TO_PLACE', sm_con,
            transitions={'succeeded':'succeeded',
                         'repeat':'INTERACTIVE_GO_TO_PLACE',
                         'aborted':'aborted',
                         'preempted':'preempted'},
            remapping={'place_name':'place_name',
                       'map_name':'map_name'}
        )
    
    return sm
    
# main
if __name__ == '__main__':

    rospy.init_node('interactive_go_to_place')

    sm = getInstance()

    # generate dummy userdata
    ud = smach.UserData()
    ud.place_name = 'kitchen'
    ud.map_name = 'map.sem_map'

    # introspection server
    sis = smach_ros.IntrospectionServer('interactive_go_to_place', sm, '/INTERACTIVE_GO_TO_PLACE_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
 
    
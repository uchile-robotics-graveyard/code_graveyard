#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros



class Foo(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted'],
            input_keys = ['in_data'],
            output_keys = ['out_data'])
        

    def execute(self, userdata):

        # Check for preempt
        #if self.preempt_requested():
        #    self.service_preempt()
        #    return 'preempted'

        rospy.sleep(1.0)
        
        # Check for preempt
        if self.preempt_requested():
            self.service_preempt()

            
            return 'preempted'
        
        userdata.out_data = userdata.in_data + 1
        
        rospy.logwarn(" - - - - - - - - - - - - - ")
        rospy.logwarn(    'out_data:' + str(userdata.in_data + 1 )  )
        rospy.logwarn(" - - - - - - - - - - - - - ")
        
        return 'succeeded'

    def request_preempt(self):
        # this method should not block longer than it takes to notify 
        # any child threads / processes to preempt
        
        smach.State.request_preempt(self)

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    sm.userdata.sm_data = 0
    

    sm_subbar = smach.StateMachine(
            outcomes=['succeeded','aborted','preempted'],
            input_keys = ['sm_sbar_data'],
            output_keys = ['sm_sbar_data'])
     
    with sm_subbar:
         
        smach.StateMachine.add('SBAR_A',Foo(),
            transitions = {'succeeded':'SBAR_B'},
            remapping = {
                 'in_data':'sm_sbar_data',
                 'out_data':'sm_sbar_data'
            }
        )
         
        smach.StateMachine.add('SBAR_B',Foo(),
            transitions = {'succeeded':'SBAR_C'},
            remapping = {
                 'in_data':'sm_sbar_data',
                 'out_data':'sm_sbar_data'
            }
        )
         
        smach.StateMachine.add('SBAR_C',Foo(),
            transitions = {'succeeded':'succeeded'},
            remapping = {
                 'in_data':'sm_sbar_data',
                 'out_data':'sm_sbar_data'
            }
        )

    sm_bar = smach.StateMachine(
            outcomes=['succeeded','aborted','preempted'],
            input_keys = ['sm_bar_data'],
            output_keys = ['sm_bar_data'])
    
    sm_bar.userdata.sm_bar_data = 0
    
    with sm_bar:
        
        smach.StateMachine.add('BAR_A',Foo(),
            transitions = {'succeeded':'SUBBAR'},
            remapping = {
                 'in_data':'sm_bar_data',
                 'out_data':'sm_bar_data'
            }
        )
        smach.StateMachine.add('SUBBAR',sm_subbar,
            transitions = {'succeeded':'BAR_C'},
            remapping = {
                 'sm_sbar_data':'sm_bar_data',
            }
        )
        smach.StateMachine.add('BAR_C',Foo(),
            transitions = {'succeeded':'succeeded'},
            remapping = {
                 'in_data':'sm_bar_data',
                 'out_data':'sm_bar_data'
            }
        )

    with sm:
        smach.StateMachine.add('STATE_A',Foo(),
            transitions = {'succeeded':'STATE_B'},
            remapping = {
                 'in_data':'sm_data',
                 'out_data':'sm_data'
            }
        )
        smach.StateMachine.add('STATE_B',Foo(),
            transitions = {'succeeded':'BAR'},
            remapping = {
                 'in_data':'sm_data',
                 'out_data':'sm_data'
            }
        )
        
        smach.StateMachine.add('BAR',sm_bar,
            transitions = {'succeeded':'STATE_C'},
            remapping = {
                 'sm_bar_data':'sm_data'
            }
        )
        
        smach.StateMachine.add('STATE_C',Foo(),
            transitions = {'succeeded':'STATE_A'},
            remapping = {
                 'in_data':'sm_data',
                 'out_data':'sm_data'
            }
        )
    
    initial_data = smach.UserData()
    initial_data.__setattr__('sm_data', 34)
    sm.set_initial_state(['STATE_A'],initial_data)

    return sm


# main
if __name__ == '__main__':

    rospy.init_node('sub_machine')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('sub_machine', sm, '/SUB_MACHINE_SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()

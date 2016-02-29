#!/usr/bin/env python

import roslib
roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros
import random
from subprocess import call 
from math import *
from std_srvs.srv import Empty
from bender_srvs.srv import FaceInfo, MaskInfo, ID, Onoff
from bender_msgs.msg import *
from bender_fun.BenderVaders import PyVader
from bender_fun.SwervinMervin.swervin_mervin import benrace
import os
from bender_macros.speech import Talk
from bender_macros.head import FaceOrder
from bender_macros.nav import RotateRobot
from bender_macros.arm import ArmStates
import selfie
from bender_arm_control.arm_commander import Limb

import rospkg
rospack = rospkg.RosPack()
#import sensor_msgs.msgs 
#include <sensor_msgs/image_encodings.h>


class SelectorEntretener(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Disfrazar','GameSpace','GameRace','Selfie','succeeded','aborted','preempted'])
#   smach.State.__init__(self, outcomes=['Disfrazar','GameRace','succeeded','aborted','preempted'])

        self.large=4 
        self.last=-1

    def execute(self, userdata):

        self.last=self.last+1
        rand = self.last%self.large
        if rand==0:
            self.last=0
            return 'Disfrazar'
        if rand==1:
            self.last=1
            return 'GameSpace'
        if rand==2:
            self.last=2
            return 'GameRace'
        if rand==3:
            self.last=3
            return 'Selfie'


class Disfrazar(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.videomask = rospy.ServiceProxy('/videomask/MaskActive',Onoff)
        self.counter = 0

    def execute(self, userdata):
        Talk.getInstance("let me put you some makeup! ",3)

        try:

            self.videomask(True)
            rospy.sleep(5)
            FaceOrder.ChangeFace("happy2")
            rospy.sleep(1)
            # FaceOrder.ChangeFace(0)
            Talk.getInstance("now you look pretty! ",3)
            rospy.sleep(5)
            FaceOrder.ChangeFace("happy3")
            rospy.sleep(1)
            # FaceOrder.ChangeFace(0)
            Talk.getInstance("that is pretty funny! ",3)
            rospy.sleep(5)
            FaceOrder.ChangeFace("surprise")
            rospy.sleep(1)
            # FaceOrder.ChangeFace(0)
            Talk.getInstance("oh the mask is not working now, i can see your real face! ",6)
            rospy.sleep(5)
            FaceOrder.ChangeFace("happy2")
            rospy.sleep(1)
            # FaceOrder.ChangeFace(0)
            Talk.getInstance("i like this game! ",3)        
            rospy.sleep(3)      
            FaceOrder.ChangeFace("happy1")
            rospy.sleep(1)
            # FaceOrder.ChangeFace(0)
            self.videomask(False)
        except rospy.ServiceException:
            self.videomask(False)
            return 'succeeded'

        return 'succeeded'

class GameSpace(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        Talk.getInstance("let's play bender vaders! ",3)
        Talk.getInstance("use the joystick in front of you ",3)

        path = rospack.get_path('bender_fun')
        path+="/src/bender_fun/BenderVaders/"
        full_path = os.path.abspath(path+'BenderSpace.py')
        try:
            call(["python",full_path])
  
            #PyVader.getInstance()

        except rospy.ServiceException:
            return 'succeeded'

        # rospy.sleep(0.2)
        return 'succeeded'

class GameRace(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        Talk.getInstance("let's play bender race! ",2)
        Talk.getInstance("use the joystick in front of you ",3)

        path = rospack.get_path('bender_fun')
        path+="/src/bender_fun/SwervinMervin/"
        full_path = os.path.abspath(path+'swervin_mervin/benrace.py')

        try:
            call(["python",full_path])          
            #benrace.getInstance()


        except rospy.ServiceException:
            return 'succeeded'

        # rospy.sleep(0.2)
        return 'succeeded'

    
        #HAcer emociones y speech
        # video = rospy.ServiceProxy('/playvideo', Empty)
        # try:
        #   resp=video()

        # except rospy.ServiceException, e:
        #   return 'preempted'

        # rospy.sleep(0.2)
        # return 'succeeded'

# class Selfie(smach.State):

#   def __init__(self):
#       smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
#   def execute(self, userdata):
#       Talk.getInstance("let's take a selfie! ",2)

#       #Girar cuerpo en 90 grados

#       #girar cabeza en 50 grados

#       #mover brazo

#       path = rospack.get_path('bender_behaviors')
#       path+="/src/bender_behaviors/Stage1/Robozoo/"
#       full_path = os.path.abspath(path+'selfie.py')   

#       try:
#           call(["python",full_path])      

#       except rospy.ServiceException, e:
#           return 'succeeded'

#       # rospy.sleep(0.2)
#       return 'succeeded'  

#       #take selfie

def Selfie():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    sm.userdata.trayectory_name_after_place = ['home']
    sm.userdata.joint_position = [0.781, 0.403, -0.619, 1.586, 0.614, 0.281]
    sm.userdata.pre_2 = [-0.23, 0.129, -0.102, 1.782, -0.031, 0.997]
    sm.userdata.interval = 3
    sm.userdata.segments = 30
    sm.userdata.lr_arm = 'l'
    sm.userdata.turn_around_angle_1 = 90
    sm.userdata.turn_around_angle_2 = -90
    arms = {'l':Limb('l'),'r':Limb('l')}
    with sm:
        smach.StateMachine.add('TURN_AROUND',RotateRobot.getInstance(),
                transitions={'succeeded':'MOVE_PRE_1','aborted':'MOVE_PRE_1'},
                remapping={'angle':'turn_around_angle_1'})
        smach.StateMachine.add('MOVE_PRE_1', ArmStates.setPositionNamed(arms,blind=True,init='home',goal='pre_1'),
                transitions={'succeeded':'MOVE_PRE_2','aborted':'aborted'},
                remapping={'lr_arm':'lr_arm'}
                )  
        smach.StateMachine.add('MOVE_PRE_2', ArmStates.setPositionNamed(arms,blind=True,init='pre_1',goal='pre_2'),
                transitions={'succeeded':'LIFT_ARM','aborted':'aborted'},
                remapping={'lr_arm':'lr_arm'}
                )
        smach.StateMachine.add('LIFT_ARM', ArmStates.setJointTrajectoryPosition(arms),
                transitions={'succeeded':'SELFIE'})
        smach.StateMachine.add('SELFIE', selfie.SelfieState(),
                transitions={'succeeded':'LIFT_ARM_DOWN'})
        smach.StateMachine.add('LIFT_ARM_DOWN', ArmStates.setJointTrajectoryPosition(arms),
                transitions={'succeeded':'MOVE_NAME_AFTER1'},
                remapping=  {'joint_position':'pre_2'})
        smach.StateMachine.add('MOVE_NAME_AFTER1', ArmStates.setPositionNamed(arms,blind=True,init='pre_2',goal='pre_1'),
                transitions={'succeeded':'MOVE_NAME_AFTER2','aborted':'aborted'},
                remapping={'lr_arm':'lr_arm'}
                )
        smach.StateMachine.add('MOVE_NAME_AFTER2', ArmStates.setPositionNamed(arms,blind=True,init='pre_1',goal='home'),
                transitions={'succeeded':'TURN_AROUND_BACK','aborted':'aborted'},
                remapping={'lr_arm':'lr_arm'}
                )
        smach.StateMachine.add('TURN_AROUND_BACK',RotateRobot.getInstance(),
                transitions={'succeeded':'succeeded','aborted':'succeeded'},
                remapping={'angle':'turn_around_angle_2'})


    return sm





def main():
    rospy.init_node('robozoo')    

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm:
    #     smach.StateMachine.add('SelectorEntretener', SelectorEntretener(), 
    #                            transitions={'Disfrazar':'Disfrazar',
    #                                 'GameRace':'GameRace',           
    #                                 'GameSpace':'GameSpace',
    #                                 'aborted': 'SelectorEntretener',
    #                                 'preempted': 'SelectorEntretener'})
    #     smach.StateMachine.add('Disfrazar', Disfrazar(), 
    #                            transitions={'succeeded':'SelectorEntretener',
    #                            'preempted': 'SelectorEntretener'})
    #     smach.StateMachine.add('GameSpace', GameSpace(), 
    #                            transitions={'succeeded':'SelectorEntretener',
    #                            'preempted': 'SelectorEntretener'})
    #     smach.StateMachine.add('GameRace', GameRace(), 
    #                            transitions={'succeeded':'SelectorEntretener',
    #                            'preempted': 'SelectorEntretener'})
        smach.StateMachine.add('Selfie', Selfie(), 
                               transitions={'succeeded':'succeeded',
                               'preempted': 'succeeded'})

    sm.set_initial_state(['Selfie'])

    sis = smach_ros.IntrospectionServer('robozoo', sm, '/ROBOZOO')
    sis.start()
    # Execute SMACH plan
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()



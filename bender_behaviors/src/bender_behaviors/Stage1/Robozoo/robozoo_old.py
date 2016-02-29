#!/usr/bin/env python

import roslib
roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros
import random
from math import *
from std_srvs.srv import Empty
from bender_srvs.srv import FaceInfo, MaskInfo, ID
from bender_msgs.msg import *
#import sensor_msgs.msgs 
#include <sensor_msgs/image_encodings.h>


class Catch(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):

        catch = rospy.ServiceProxy('/catch', Empty)

        try:
            catch()
        except rospy.ServiceException:
            return 'preempted'

        rospy.sleep(1)
        return 'succeeded'


class FindFace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Face Detected','Face Not Detected','succeeded','aborted','preempted'])
        self.counterface = 0
        self.counternoface = 0

    def execute(self, userdata):

        face_detection = rospy.ServiceProxy('/bender/vision/face_detector/detect_face', FaceInfo)
        # pub.publish(Twist())
        try:
            resp=face_detection("",False)
        except rospy.ServiceException:
            self.counterface=0
            return 'preempted'

        entro = False
        if resp.n_faces > 0 : 
            for i in range(0,resp.n_faces): 
                if resp.BBoxes[i].height > 75:
                    self.counterface+=1
                    self.counternoface=0
                    entro = True
        if entro is False:
            self.counterface=0
            self.counternoface+=1

        if self.counterface>=2:
            self.counterface=0
            return 'Face Detected'
        if self.counternoface>=2:
            self.counternoface=0
            return 'Face Not Detected'
        return 'succeeded'

class Disfrazar(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.counter = 0

    def execute(self, userdata):

        face_mask = rospy.ServiceProxy('/facemask/mask', MaskInfo)
        try:
            # rospy.loginfo(self.counter) 

            if fmod(self.counter, 4)==2:
                face_mask(0)
            else:
                face_mask(1)

        except rospy.ServiceException:
            return 'preempted'

        self.counter+=1
        rospy.sleep(0.2)
        return 'succeeded'

class Vacaciones(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):

        video = rospy.ServiceProxy('/playvideo', Empty)
        try:
            video()

        except rospy.ServiceException:
            return 'preempted'

        rospy.sleep(0.2)
        return 'succeeded'

class Lastima(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):

        lastima = rospy.ServiceProxy('/movarm', Empty)
        try:
            lastima()

        except rospy.ServiceException:
            return 'preempted'

        rospy.sleep(0.2)
        return 'succeeded'


class Bailar(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        dance = rospy.ServiceProxy('/dancegeneral', Empty)
        try:
            dance()
        except rospy.ServiceException:
            return 'preempted'
        rospy.sleep(0.2)
        return 'succeeded'

class Mundial(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        mund = rospy.ServiceProxy('/mundial', Empty)
        try:
            mund()
        except rospy.ServiceException:
            return 'preempted'

        rospy.sleep(0.2)
        return 'succeeded'

class SimularFalla(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):

        simulate = rospy.ServiceProxy('/simulate', Empty)
        try:
            simulate()
        except rospy.ServiceException:
            return 'preempted'

        rospy.sleep(0.2)
        return 'succeeded'


class SelectorEntretener(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Disfrazar','Vacaciones','Lastima','Mundial','succeeded','aborted','preempted'])
        self.large=4
        self.last=3

    def execute(self, userdata):
        follow = rospy.ServiceProxy('/detect_face', ID)

        try:
            resp1=follow()
            print resp1.ID
            if resp1.ID==0:
                add_face = rospy.ServiceProxy('/add_face', ID)
                add_face()
                bienve = rospy.ServiceProxy('/bienve', Empty)
                bienve()
                   
        except rospy.ServiceException:
            return 'preempted'

        self.last=self.last+1
        rand = self.last%self.large
        if rand==0:
            self.last=0
            return 'Disfrazar'
        if rand==1:
            self.last=1
            return 'Mundial'
        if rand==2:
            self.last=2
            return 'Lastima'
        if rand==3:
            self.last=3
            return 'Vacaciones'
        return 'Disfrazar'


class SelectorLlamarAtencion(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['Bailar','SimularFalla','succeeded','aborted','preempted'])    
        self.large=2
        self.last=1

    def execute(self, userdata):
        rand = random.randint(1, self.large-1)
        if (self.last+rand)%self.large==0:
            self.last=0
            return 'Bailar'
        if (self.last+rand)%self.large==1:
            self.last=1
            return 'SimularFalla'
        return 'SimularFalla'


def main():
    rospy.init_node('robozoo')    

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm:
        # Add states to the container
        smach.StateMachine.add('Catch', Catch(), 
                               transitions={'succeeded': 'FindFace',
                                    'preempted': 'Catch'})
        smach.StateMachine.add('FindFace', FindFace(), 
                               transitions={'Face Detected':'SelectorEntretener', 
                                    'Face Not Detected':'SelectorLlamarAtencion',
                                    'succeeded': 'FindFace',
                                    'preempted': 'FindFace'})

        smach.StateMachine.add('SelectorEntretener', SelectorEntretener(), 
                               transitions={'Disfrazar':'Disfrazar', 
                                    'Vacaciones':'Vacaciones',
                                    'Lastima':'Lastima',
                                    'Mundial':'Mundial',
                                    'aborted': 'FindFace',
                                    'preempted': 'FindFace'})
        smach.StateMachine.add('Disfrazar', Disfrazar(), 
                               transitions={'succeeded':'FindFace',
                               'preempted': 'FindFace'})
        smach.StateMachine.add('Vacaciones', Vacaciones(), 
                               transitions={'succeeded':'FindFace',
                               'preempted': 'FindFace'})
        smach.StateMachine.add('Lastima', Lastima(), 
                               transitions={'succeeded':'FindFace',
                               'preempted': 'FindFace'})
        smach.StateMachine.add('Mundial', Mundial(), 
                               transitions={'succeeded':'FindFace',
                               'preempted': 'FindFace'})
        smach.StateMachine.add('SelectorLlamarAtencion', SelectorLlamarAtencion(), 
                               transitions={'Bailar':'Bailar', 
                                    'SimularFalla':'SimularFalla',
                                    'preempted': 'FindFace'})
        smach.StateMachine.add('Bailar', Bailar(), 
                               transitions={'succeeded':'FindFace',
                               'preempted': 'FindFace'})
        smach.StateMachine.add('SimularFalla', SimularFalla(), 
                           transitions={'succeeded':'FindFace',
                               'preempted': 'FindFace'})

    sis = smach_ros.IntrospectionServer('robozoo', sm, '/ROBOZOO')
    sis.start()
    # Execute SMACH plan
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()


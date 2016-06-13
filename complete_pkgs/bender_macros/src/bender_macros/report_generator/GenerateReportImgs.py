#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')

# copy file to pendrive
import roslib.packages
import os
import shutil

import rospy
import smach
import smach_ros

from std_msgs.msg import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from bender_srvs.srv import ReportGenerator
from bender_srvs.srv import ReportGeneratorRequest
from bender_srvs.srv import ReportGeneratorResponse
from bender_macros.speech import Talk
from bender_utils.ros import benpy


class GenerateReport(smach.State):

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['succeeded','aborted','preempted'],
                input_keys = ['images','image_captions'],
                output_keys = ['report_path']
        )
        srv_name = '/bender/report_generator/report_generator/generate'
        self.report_client = benpy.ServiceProxy(srv_name, ReportGenerator)
        
    def execute(self, userdata):
        
        rospy.loginfo('Executing state Generate Report')
        Talk.getInstance('I will generate the pdf', 2.5)
    
        request = ReportGeneratorRequest()
        request.imgs = userdata.images
        request.img_captions = userdata.image_captions
    
        
        response = self.report_client(request)
        
        print 'report saved al location: ' + response.report_path
        userdata.report_path = response.report_path
        
        return 'succeeded'
        

class CopyToDevice(smach.State):
    
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['succeeded','device_not_found','aborted','preempted'],
                input_keys = ['device_name','report_path']
        )
        
    def execute(self, ud):
        
        dev_base = '/media/bender/'
        device = dev_base + ud.device_name
        dst = device + '/' + 'report.pdf'
        
        try:
            devices = os.listdir(dev_base)
            
            if ud.device_name not in devices:
                rospy.logwarn('Sorry, there is no device "' + device + '". The report is currently stored at "' + ud.report_path + '"')
                return 'device_not_found'
            
            shutil.copy(ud.report_path, dst)
            rospy.loginfo('report copied to device: ' + dst)
            return 'succeeded'
            
        except Exception, e:
            return 'aborted'
        


def getInstance():
        
    sm = smach.StateMachine(
            outcomes = ['succeeded','aborted','preempted','device_not_found'],
            input_keys = ['report_imgs','report_img_captions','device_name']
    )
    
    with sm:
                    
        smach.StateMachine.add('GENERATE_REPORT', GenerateReport(),
                transitions = {'succeeded':'succeeded'},
                remapping = {'images':'report_imgs',
                             'image_captions':'report_img_captions',
                             'report_path':'report_path'}
        )
        
        smach.StateMachine.add('COPY_TO_DEVICE', CopyToDevice(),
                transitions = {'succeeded':'succeeded',
                               'device_not_found':'device_not_found'},
                remapping = {'device_name':'device_name',
                             'report_path':'report_path'}
        )

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('report_generator_imgs')

    ud = smach.UserData()
    ud.report_imgs = []
    ud.report_img_captions = []
    ud.device_name = 'BERNUY RC'
    
    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('report_generator_imgs', sm, '/REPORT_GENERATOR_IMGS_SM')
    sis.start()
    rospy.sleep(2)
    outcome = sm.execute(ud)
    sis.stop()


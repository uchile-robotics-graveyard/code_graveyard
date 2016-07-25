#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import math
import smach
import smach_ros
import numpy
import Pycluster

from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from bender_srvs.srv import *
from bender_msgs.msg import *

#  - - - - macros - - - -
from bender_macros.speech import Talk
from bender_macros.head import MoveAsus
from bender_macros.head import FaceOrder

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from tf import transformations
from bender_utils.ros import benpy

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class SearchCrowd(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
            output_keys=['crowd_person_poses','crowd_images','crowd_ROIs_boxes','crowd_ROIs'])
        
        self.person_det_enable_client = benpy.ServiceProxy('/bender/pcl/hog_person_detector/enable', Onoff)
        self.person_detect_client = benpy.ServiceProxy('/bender/pcl/hog_person_detector/get_detection_x', Recognition)
        
        # control
        self.max_attempts = 3
        self.min_crowd_size = 0
        
    def execute(self, userdata):
        
        rospy.loginfo('Executing state  search Crowd')

        # enable people detection
        self.person_det_enable_client(select=True)

        # try to find a crowd
        crowd_size = 0
        attempts = 0
        while attempts < self.max_attempts:
            
            # wait for some recognitions
            rospy.sleep(0.5)
            
            try:
                resp = self.person_detect_client()
                n_found = len(resp.pose)
                rospy.loginfo("SearchCrowd: detected (" + str(n_found) + ") persons.")
                if n_found > crowd_size:
                    
                    crowd_size = n_found
                    userdata.crowd_person_poses = resp.pose
                    userdata.crowd_images = resp.imgs
                    userdata.crowd_ROIs_boxes = resp.detection_boxes
                    userdata.crowd_ROIs = resp.detections

            except rospy.ServiceException as e:
                rospy.logwarn("Search crowd: service call to hog person recognition failed. why?: " + str(e))

            attempts += 1


        self.person_det_enable_client(False)
        if crowd_size > self.min_crowd_size:
            return 'succeeded'
            
        Talk.getInstance("ups ... i could not see any crowd", 4)
        return 'aborted'


class CalculatePoses(smach.State):

    def __init__(self):
    
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'],
                input_keys  = ['n_clusters'],
                io_keys     = ['crowd_person_poses'],
                output_keys = ['crowd_go_poses','crowd_centers','crowd_indexes'])
        
        self.transformer_client = benpy.ServiceProxy('/bender/tf/simple_pose_transformer/transform', Transformer)
        self.robot_frame = "bender/base_link"
        self.map_frame   = "map"

    def execute(self, userdata):
        
        rospy.loginfo('Executing state CalculatePoses')
        
        rospy.loginfo("userdata: crowd_person_poses")
        print userdata.crowd_person_poses

        centroids, go_poses, poses_tf, idxs = self.analyze(userdata.crowd_person_poses, userdata.n_clusters)
        
        # failed to analyze
        if centroids == False:
            return "aborted"
        
        userdata.crowd_centers  = centroids
        userdata.crowd_go_poses = go_poses
        userdata.crowd_indexes  = idxs
        userdata.crowd_person_poses = poses_tf

        return 'succeeded'


    def transform_poses(self, poses, frame_out):
        
        req = TransformerRequest()
        req.frame_out = frame_out
        
        poses_tf = []
        for p in poses:
            try:
                req.pose_in = p
                req.pose_in.header.stamp = rospy.Time.now()
                transf_out = self.transformer_client(req)
                poses_tf.append(transf_out.pose_out)

            except rospy.ServiceException, e:
                rospy.logwarn("Failed to transform pose.")
                return []

        return poses_tf
    
    def ps_array_to_numpy(self,poses):
        # transforms a PoseStamped list into a numpy xy vector array
        
        ps_to_numpy = lambda ps: numpy.array([[ps.pose.position.x, ps.pose.position.y]]) 
        np_append = lambda p1,p2: numpy.append(p1, p2, axis=0)
        return reduce(np_append , map(ps_to_numpy, poses))

    def cluster_poses(self,poses,n_clusters) :
        
        def make_ps(val):

            ps = geometry_msgs.msg.PoseStamped()
            ps.header.frame_id = self.robot_frame
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x = val[0]
            ps.pose.position.y = val[1]
            ps.pose.orientation.w = 1.0
            return ps
        
        # transform poses to a numpy object
        xy = self.ps_array_to_numpy(poses)
        
        # kmeans for N clusters
        xy = numpy.array(zip(xy[:,0],xy[:,1]))

        idxs, res, a = Pycluster.kcluster(xy, n_clusters)
        res = Pycluster.clustercentroids(xy, clusterid = idxs)

        # generate centroids
        centroids = map(make_ps, res[0])
        
        # generate clusters 
        clusters = [[] for _ in range(n_clusters)]
        cnt = 0
        for c_idx in idxs:
            clusters[c_idx].append(poses[cnt])
            cnt += 1
        
        # delete empty clusters
        valid_clusters  = []
        valid_centroids = []
        for idx in range(len(clusters)):
            # a valid cluster must have elements!
            if clusters[idx]:
                valid_clusters.append(clusters[idx])
                valid_centroids.append(centroids[idx])

        return valid_centroids, valid_clusters, numpy.array(idxs)

    def analyze(self, poses, n_clusters):

        def analyze_cluster(cluster):
            # requires cluster poses on the robot_frame
            
            print cluster
            res = geometry_msgs.msg.PoseStamped()
            res.header.stamp = rospy.Time.now()
            res.header.frame_id = self.robot_frame
            
            # limits
            xmin =  1000.0
            xmax = -1000.0
            ymin =  1000.0
            ymax = -1000.0
            
            for ps in cluster:
                
                if ps.pose.position.x < xmin:
                    xmin = ps.pose.position.x 
                if ps.pose.position.x > xmax:
                    xmax = ps.pose.position.x
                if ps.pose.position.y < ymin:
                    ymin = ps.pose.position.y
                if ps.pose.position.y > ymax:
                    ymax = ps.pose.position.y
                 
            res.pose.position.x = xmin - 1.0
            res.pose.position.y = (ymax + ymin)/2.0
            res.pose.position.z = 0.0
            res.pose.orientation.w = 1.0
            
            return res

        n_poses = len(poses)
        rospy.loginfo("# poses to analyze: " + str(n_poses))
        
        # transform all poses to robot/map frame
        poses_robot_frame = self.transform_poses(poses, self.robot_frame)
        poses_map_frame   = self.transform_poses(poses, self.map_frame  )
        if not n_poses in [len(poses_map_frame), len(poses_robot_frame)]:
            return False, False, False, False
        
        # cluster transformed poses        
        if n_poses == 1:
            centroids =  poses_map_frame
            clusters  = [poses_robot_frame]
            idxs = [0]
        else:
            centroids, clusters, idxs = self.cluster_poses(poses_robot_frame, n_clusters)
            centroids = self.transform_poses(centroids, self.map_frame)
            
        # analyze clusters for approach
        go_poses = map(analyze_cluster, clusters)
        go_poses = self.transform_poses(go_poses, self.map_frame)
        
        return centroids, go_poses, poses_map_frame, idxs

class TestCalculatePoses(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['crowd_person_poses'])
           
    def execute(self, userdata):
        
        rospy.loginfo('Executing dummy state TestCalculatePoses')

        people = []

        p = geometry_msgs.msg.PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'bender/base_link'
        p.pose.orientation.w = 1.0
        p.pose.position.z = 1.0

        p.pose.position.x = 1.524273577104
        p.pose.position.y = -1.524273577104
        people.append(p)        

        p.pose.position.x = 1.524273577104
        p.pose.position.y = 1.524273577104
        people.append(p)

        p.pose.position.x = 1.524273577104
        p.pose.position.y = 0.524273577104
        people.append(p)

        p.pose.position.x =  1.524273577104
        p.pose.position.y = -1.0
        people.append(p)

        userdata.crowd_person_poses = people
        return 'succeeded'

def getInstance():
    
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
        input_keys=['n_clusters'],
        output_keys=['crowd_person_poses','crowd_images','crowd_ROIs_boxes',
                     'crowd_ROIs','crowd_go_poses','crowd_centers','crowd_indexes'])


    # - - -  parameters - - -

    # crowd poses
    sm.userdata.crowd_person_poses = [] # 
    sm.userdata.crowd_images       = [] # ?  
    sm.userdata.crowd_ROIs_boxes   = [] # ?
    sm.userdata.crowd_ROIs         = [] # ?
    sm.userdata.crowd_indexes      = [] # ?

    sm.userdata.crowd_go_poses  = None # approach pose
    sm.userdata.crowd_centers   = None # look pose

    with sm:

        smach.StateMachine.add('MOVE_ASUS', MoveAsus.getReadyMachine(15),
            transitions={'succeeded':'SEARCH_CROWD'}
        )

        smach.StateMachine.add('SEARCH_CROWD', SearchCrowd(),
            transitions = {'succeeded':'CALCULATE_POSES',
                           'aborted':'SEARCH_CROWD'},
            remapping = {'crowd_person_poses':'crowd_person_poses',
                         'crowd_images':'crowd_images',
                         'crowd_ROIs_boxes':'crowd_ROIs_boxes',
                         'crowd_ROIs':'crowd_ROIs'}
        )
        
        # smach.StateMachine.add('TEST', TestCalculatePoses(),
        #     transitions = {'succeeded':'CALCULATE_POSES'},
        #     remapping = {'crowd_person_poses':'crowd_person_poses'}
        # )

        smach.StateMachine.add('CALCULATE_POSES', CalculatePoses(),
            transitions = {'succeeded':'succeeded',
                           'aborted':'SEARCH_CROWD'
                           },
            remapping = {'n_clusters':'n_clusters',
                         'crowd_person_poses':'crowd_person_poses',
                         'crowd_go_poses':'crowd_go_poses',
                         'crowd_centers':'crowd_centers'}
        )

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('search_crowd')

    sm = getInstance()
    ud = smach.UserData()
    ud.n_clusters = 2

    # introspection server
    sis = smach_ros.IntrospectionServer('search_crowd', sm, '/SEARCH_CROWD_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()

    print "-------------------------------------------------"
    print "n_clusters:" + str(ud.n_clusters)
    print "crowd_go_poses:"
    print ud.crowd_go_poses
    print "crowd_centers:"
    print ud.crowd_centers
    print "crowd_person_poses:"
    print ud.crowd_person_poses
    print "indexes:"
    print ud.crowd_indexes

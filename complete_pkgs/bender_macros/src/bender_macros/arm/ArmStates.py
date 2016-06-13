#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from bender_utils.ros import benpy
from bender_arm_control.arm_commander import Limb
from geometry_msgs.msg import Pose, PoseStamped, Point
from bender_msgs.msg import CylindricalObject
from tf import transformations
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive

class orientateGripper(smach.State):

  def __init__(self,l):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],input_keys=['lr_arm'])
    self.arms = l
    
  def execute(self, userdata):
    self.limb = self.arms[userdata.lr_arm]
    if self.limb.check_planning():
      if self.limb.arm.orientate_gripper():
        return 'succeeded'
      else:
        return 'aborted'
    else:
      rospy.logwarn('No planning selected')
      return 'aborted'

class setPosition(smach.State):

  def __init__(self,arms):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                  input_keys=['pose','approx_ik','lr_arm'])
    self.arms = arms
    
  def execute(self, userdata):
    self.limb = self.arms[userdata.lr_arm]
    if self.limb.check_planning():
      rospy.loginfo('Sending to [{:.2f}, {:.2f}, {:.2f}] frame_id: {}. Using approx. IK: {}'.format(userdata.pose.pose.position.x,
        userdata.pose.pose.position.y, userdata.pose.pose.position.z, userdata.pose.header.frame_id, userdata.approx_ik))
      if self.limb.arm.set_position(userdata.pose,approx_ik=userdata.approx_ik): # poseStamped
        return 'succeeded'
      else:
        return 'aborted'
    else:
      rospy.logwarn('No planning selected')
      return 'aborted'

class setPositionNamed(smach.State):

  def __init__(self,l,blind=False,init=None,goal=None):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                  input_keys=['trayectory_name','lr_arm'])
    self.arms = l
    self.blind = blind
    self.init = init
    self.goal = goal

  def execute(self, userdata):
    self.limb = self.arms[userdata.lr_arm]
    if self.blind:
      self.limb.arm.move_joint_blind(self.init,self.goal)
      self.limb.arm.wait()
      return 'succeeded'
    elif self.blind == False or self.init == None or self.goal == None:
      if self.limb.check_planning():
        if type(userdata.trayectory_name) == str:
          if self.limb.arm.set_position_named(userdata.trayectory_name): # move_group trayectory
            return 'succeeded'
          else:
            return 'aborted'
        elif type(userdata.trayectory_name) == list:
          for trayectory in userdata.trayectory_name:
            if self.limb.arm.set_position_named(trayectory): # move_group trayectory
              continue
            else:
              return 'aborted'
          return 'succeeded'
      else:
        rospy.logwarn('No planning selected')
        return 'aborted'

class Servoing(smach.State):

  def __init__(self,l):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                  input_keys=['dx','dy','dz','lr_arm'])
    self.arms = l
    
  def execute(self, userdata):
    self.limb = self.arms[userdata.lr_arm]
    if self.limb.check_planning():
      if self.limb.arm.servoing(userdata.dx,userdata.dy,userdata.dz):
        return 'succeeded'
      else:
        return 'aborted'
    else:
      rospy.logwarn('No planning selected')
      return 'aborted'

class openGripper(smach.State):

  def __init__(self,l):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],input_keys=['lr_arm'])
    self.arms = l

  def execute(self, userdata):
    self.limb = self.arms[userdata.lr_arm]
    if self.limb.gripper.open().reached_goal:
      return 'succeeded'
    else:
      return 'aborted'

class closeGripper(smach.State):

  def __init__(self,l):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted','stalled','notgrabbing'],
                  input_keys=['effort','lr_arm'])
    self.arms = l
    
  def execute(self, userdata):
    self.limb = self.arms[userdata.lr_arm]
    if self.limb.gripper.close(userdata.effort).stalled: # max effort 950
      return 'stalled' # for grasping
    elif self.limb.gripper.close(userdata.effort).reached_goal: # max effort 950
      return 'notgrabbing' # for closing the gripper
    else:
      return 'aborted'

class positionObjectAndGrasp(smach.State):
    
    def __init__(self,limb):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                  input_keys = ['posestamped','height','radius','lr_arm'])
        self.arms = limb

    def get_pose(self,x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.p = Pose()
        self.p.position.x, self.p.position.y, self.p.position.z = x, y, z
        self.q = transformations.quaternion_from_euler(roll, pitch, yaw)
        self.p.orientation.x = self.q[0]
        self.p.orientation.y = self.q[1]
        self.p.orientation.z = self.q[2]
        self.p.orientation.w = self.q[3]
        return self.p
    
    def get_collision_cylinder(self,pose, name,side, dim = [0.22, 0.04]):
        self.obj = CollisionObject()
        self.obj.header.stamp = rospy.Time.now()
        self.obj.header.frame_id = 'bender/base_link'
        self.obj.id = name
        
        if side == 'r':
          # pose.position.z += 0.04
          dim[0] -= 0.02

        self.cylinder = SolidPrimitive()
        self.cylinder.type = SolidPrimitive.CYLINDER
        self.cylinder.dimensions = dim

        self.obj.primitives.append(self.cylinder)
        self.obj.primitive_poses.append(pose)
        
        self.obj.operation = CollisionObject.ADD
        return self.obj
        
    def execute(self,userdata):

        self.limb = self.arms[userdata.lr_arm]
        # Grasp obj
        self.object_pose = self.get_pose(userdata.posestamped.pose.position.x,userdata.posestamped.pose.position.y,userdata.posestamped.pose.position.z)
        self.pringles = self.get_collision_cylinder(self.object_pose, 'pringles',userdata.lr_arm, [userdata.height,userdata.radius])

        self.limb.arm.generate_grasp(self.pringles, axial_res = 5, angle_res = 10)

        self.possible_grasp = self.limb.arm.get_grasp()
        if len(self.possible_grasp.ik_solutions) == 0:
          rospy.logerr('NO SE GENERARON GRASPS :(')
          return 'aborted'
            
        self.pregrasp_joints = self.possible_grasp.ik_solutions[2*self.possible_grasp.order[0]].positions
        self.grasp_joints = self.possible_grasp.ik_solutions[2*self.possible_grasp.order[0]+1].positions
  
        self.result = self.limb.arm.set_joint(self.pregrasp_joints) # Movimiento con planificador

        if (self.result.error_code.val == MoveItErrorCodes.SUCCESS):
            self.limb.gripper.open(effort = 0.3)
            rospy.sleep(1.0)
            self.limb.arm.move_joint(self.grasp_joints, interval = 2.5,segments = 20) # Movimiento con collisiones permitidas
            rospy.sleep(3.0)
            self.limb.gripper.close(effort = 0.3)
            self.limb.gripper.open(effort = 0.3)
            self.limb.gripper.close(effort = 0.3)
            self.limb.gripper.open(effort = 0.3)
            return 'succeeded'
        else:
            return 'aborted'

class positionObjectAndGrasp_capmap(smach.State):
    
    def __init__(self,limb):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                  input_keys = ['posestamped','height','radius','lr_arm'])
        self.arms = limb

    def get_pose(self,x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.p = Pose()
        self.p.position.x, self.p.position.y, self.p.position.z = x, y, z
        self.q = transformations.quaternion_from_euler(roll, pitch, yaw)
        self.p.orientation.x = self.q[0]
        self.p.orientation.y = self.q[1]
        self.p.orientation.z = self.q[2]
        self.p.orientation.w = self.q[3]
        return self.p
    
    def get_collision_cylinder(self,pose, name,side, dim = [0.22, 0.04]):
        self.obj = CollisionObject()
        self.obj.header.stamp = rospy.Time.now()
        self.obj.header.frame_id = 'bender/base_link'
        self.obj.id = name
        
        if side == 'r':
          # pose.position.z += 0.04
          dim[0] -= 0.02

        self.cylinder = SolidPrimitive()
        self.cylinder.type = SolidPrimitive.CYLINDER
        self.cylinder.dimensions = dim

        self.obj.primitives.append(self.cylinder)
        self.obj.primitive_poses.append(pose)
        
        self.obj.operation = CollisionObject.ADD
        return self.obj
        
    def execute(self,userdata):

        self.limb = self.arms[userdata.lr_arm]
        # Grasp obj
        self.object_pose = self.get_pose(userdata.posestamped.pose.position.x,userdata.posestamped.pose.position.y,userdata.posestamped.pose.position.z)
        self.pringles = self.get_collision_cylinder(self.object_pose, 'pringles',userdata.lr_arm, [userdata.height,userdata.radius])

        #self.limb.arm.generate_grasp(self.pringles, axial_res = 5, angle_res = 10)

        self.possible_grasp = self.limb.arm.get_grasp_capmap(self.pringles)
        if not self.possible_grasp:
          rospy.logerr('No se encontraron grasps')
          return 'aborted'

        rospy.loginfo('Se encontraron {} grasps'.format(len(self.possible_grasp['pregrasp'])))
            
        self.pregrasp_joints = self.possible_grasp['pregrasp'][0]
        self.grasp_joints = self.possible_grasp['grasp'][0]
  
        self.result = self.limb.arm.set_joint(self.pregrasp_joints) # Movimiento con planificador

        if (self.result.error_code.val == MoveItErrorCodes.SUCCESS):
            self.limb.gripper.open(effort = 0.3)
            rospy.sleep(1.0)
            self.limb.arm.move_joint(self.grasp_joints, interval = 2.5,segments = 20) # Movimiento con collisiones permitidas
            rospy.sleep(3.0)
            self.limb.gripper.close(effort = 0.3)
            rospy.sleep(1.0)
            self.limb.gripper.open(effort = 0.3)
      #      self.limb.gripper.close(effort = 300)
      #      self.limb.gripper.open(effort = 300)
            return 'succeeded'
        else:
            return 'aborted'

class positionEfector_capmap(smach.State):
    
    def __init__(self,limb):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                  input_keys = ['posestamped','height','radius','lr_arm'])
        self.arms = limb

    def get_pose(self,x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.p = Pose()
        self.p.position.x, self.p.position.y, self.p.position.z = x, y, z
        self.q = transformations.quaternion_from_euler(roll, pitch, yaw)
        self.p.orientation.x = self.q[0]
        self.p.orientation.y = self.q[1]
        self.p.orientation.z = self.q[2]
        self.p.orientation.w = self.q[3]
        return self.p
    
    def get_collision_cylinder(self,pose, name,side, dim = [0.22, 0.04]):
        self.obj = CollisionObject()
        self.obj.header.stamp = rospy.Time.now()
        self.obj.header.frame_id = 'bender/base_link'
        self.obj.id = name
        
        if side == 'r':
          # pose.position.z += 0.04
          dim[0] -= 0.02

        self.cylinder = SolidPrimitive()
        self.cylinder.type = SolidPrimitive.CYLINDER
        self.cylinder.dimensions = dim

        self.obj.primitives.append(self.cylinder)
        self.obj.primitive_poses.append(pose)
        
        self.obj.operation = CollisionObject.ADD
        return self.obj
        
    def execute(self,userdata):

        self.limb = self.arms[userdata.lr_arm]
        # Grasp obj
        self.object_pose = self.get_pose(userdata.posestamped.pose.position.x,userdata.posestamped.pose.position.y,userdata.posestamped.pose.position.z)
        self.pringles = self.get_collision_cylinder(self.object_pose, 'pringles',userdata.lr_arm, [userdata.height,userdata.radius])

        #self.limb.arm.generate_grasp(self.pringles, axial_res = 5, angle_res = 10)

        self.possible_grasp = self.limb.arm.get_grasp_capmap(self.pringles)
        if not self.possible_grasp:
          rospy.logerr('No se encontraron grasps')
          return

        rospy.loginfo('Se encontraron {} grasps'.format(len(self.possible_grasp['pregrasp'])))
            
        self.pregrasp_joints = self.possible_grasp['pregrasp'][0]
        self.grasp_joints = self.possible_grasp['grasp'][0]
  
        self.result = self.limb.arm.set_joint(self.pregrasp_joints) # Movimiento con planificador

        if (self.result.error_code.val == MoveItErrorCodes.SUCCESS):
            self.limb.arm.move_joint(self.grasp_joints, interval = 2.5,segments = 20) # Movimiento con collisiones permitidas
            rospy.sleep(3.0)
            self.limb.gripper.open(effort = 0.3)
            rospy.sleep(1.0)
            return 'succeeded'
        else:
            return 'aborted'
    
class getPossibleGrasp_capmap(smach.State):
    
    def __init__(self,limb):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                  input_keys = ['posestamped','height','radius','lr_arm'],
                  output_keys = ['possible_grasp'])
        self.arms = limb

    def get_pose(self,x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.p = Pose()
        self.p.position.x, self.p.position.y, self.p.position.z = x, y, z
        self.q = transformations.quaternion_from_euler(roll, pitch, yaw)
        self.p.orientation.x = self.q[0]
        self.p.orientation.y = self.q[1]
        self.p.orientation.z = self.q[2]
        self.p.orientation.w = self.q[3]
        return self.p
    
    def get_collision_cylinder(self,pose, name,side, dim = [0.22, 0.04]):
        self.obj = CollisionObject()
        self.obj.header.stamp = rospy.Time.now()
        self.obj.header.frame_id = 'bender/base_link'
        self.obj.id = name
        
        if side == 'r':
          # pose.position.z += 0.04
          dim[0] -= 0.02

        self.cylinder = SolidPrimitive()
        self.cylinder.type = SolidPrimitive.CYLINDER
        self.cylinder.dimensions = dim

        self.obj.primitives.append(self.cylinder)
        self.obj.primitive_poses.append(pose)
        
        self.obj.operation = CollisionObject.ADD
        return self.obj
        
    def execute(self,userdata):

        self.limb = self.arms[userdata.lr_arm]
        # Grasp obj
        self.object_pose = self.get_pose(userdata.posestamped.pose.position.x,userdata.posestamped.pose.position.y,userdata.posestamped.pose.position.z)
        self.object = self.get_collision_cylinder(self.object_pose, 'object',userdata.lr_arm, [userdata.height,userdata.radius])

        #self.limb.arm.generate_grasp(self.object, axial_res = 5, angle_res = 10)

        self.possible_grasp = self.limb.arm.get_grasp_capmap(self.object)
        if not self.possible_grasp:
          add_obj_acm(self.object)
          self.possible_grasp = self.limb.arm.get_grasp_capmap(self.object)
          rospy.logerr('No se encontraron grasps')
          return 'aborted'

        rospy.loginfo('Se encontraron {} grasps'.format(len(self.possible_grasp['pregrasp'])))
        userdata.possible_grasp = self.possible_grasp
        return 'succeeded'

class grasp_capmap(smach.State):
    
    def __init__(self,limb):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                  input_keys = ['possible_grasp','lr_arm','max_attempts'])
        self.arms = limb

    def execute(self,userdata):

        self.limb = self.arms[userdata.lr_arm]

        self.possible_grasp = userdata.possible_grasp
         
        #self.pregrasp_joints = self.possible_grasp['pregrasp'][0]
        #self.grasp_joints = self.possible_grasp['grasp'][0]
  
        for i,pregrasp_joints in enumerate(self.possible_grasp['pregrasp']):
          self.grasp_joints = self.possible_grasp['grasp'][i]
          self.result = self.limb.arm.set_joint(pregrasp_joints) # Movimiento con planificador

          if i == userdata.max_attempts:
            return 'aborted'
          if (self.result.error_code.val == MoveItErrorCodes.SUCCESS):
            self.limb.gripper.open(effort = 0.3)
            rospy.sleep(1.0)
            self.limb.arm.move_joint(self.grasp_joints, interval = 2.5,segments = 20) # Movimiento con collisiones permitidas
            rospy.sleep(3.0)
            self.limb.gripper.close(effort = 0.3)
            rospy.sleep(1.0)
            self.limb.gripper.open(effort = 0.3)
        #      self.limb.gripper.close(effort = 300)
        #      self.limb.gripper.open(effort = 300)
            return 'succeeded'
          else:
            continue

class setJointTrajectoryPosition(smach.State):

  def __init__(self,arms):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                  input_keys=['joint_position','interval','segments','lr_arm'])
    self.arms = arms

  def execute(self, userdata):
    self.limb = self.arms[userdata.lr_arm]
    self.limb.arm.move_joint(userdata.joint_position,interval=userdata.interval,segments=userdata.segments)
    self.limb.arm.wait()
    return 'succeeded'

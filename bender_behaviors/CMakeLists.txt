cmake_minimum_required(VERSION 2.8.3)
project(bender_behaviors)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  diagnostic_msgs
  roscpp
  rospy
  smach
  smach_ros
  std_msgs
  std_srvs
)


catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bender_behaviors
  CATKIN_DEPENDS actionlib actionlib_msgs diagnostic_msgs roscpp rospy smach smach_ros std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

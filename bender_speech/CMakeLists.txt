cmake_minimum_required(VERSION 2.8.3)
project(bender_speech)

find_package(catkin REQUIRED COMPONENTS
  bender_msgs
  bender_srvs
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  bender_utils
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS bender_msgs bender_srvs roscpp roslib rospy std_msgs std_srvs bender_utils
)
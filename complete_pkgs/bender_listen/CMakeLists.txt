cmake_minimum_required(VERSION 2.8.3)
project(bender_listen)

# <!-- rosdep name="gstreamer_pocketsphinx"/ -->

find_package(catkin REQUIRED COMPONENTS
  bender_msgs
  bender_srvs
  diagnostic_msgs
  roscpp
  roslib
  rospy
  sound_play
  std_msgs
  std_srvs
  bender_config
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES bender_listen
  CATKIN_DEPENDS bender_msgs bender_srvs diagnostic_msgs roscpp roslib rospy sound_play std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${bender_config_INCLUDE_DIRS}
)

cmake_minimum_required(VERSION 2.8.3)
project(bender_model)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

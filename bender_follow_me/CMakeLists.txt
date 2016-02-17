cmake_minimum_required(VERSION 2.8.3)
project(bender_follow_me)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  bender_msgs
  bender_srvs
  dynamic_reconfigure
  geometry_msgs
  nodelet
  pluginlib
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  cmake_modules
  message_generation
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  bender_config
  #bender_laser
  tf
)


#find_package(OpenCV  REQUIRED)
find_package(PCL 1.7 REQUIRED)


# Eigen
# TODO: hacer checkeo sobre la version utilizada >=3.x
find_package(Eigen REQUIRED)
#message(STATUS HELLO_WORLD_${Eigen_INCLUDE_DIRS})

# Find OpenNI2
#find_package(PkgConfig)
#pkg_check_modules(OpenNI2 REQUIRED libopenni2)
#find_path(OpenNI2_INCLUDEDIR
#NAMES OpenNI.h
#HINTS /usr/include/openni2)
#find_library(OpenNI2_LIBRARIES
#NAMES OpenNI2 DummyDevice OniFile PS1090
#HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
#PATH_SUFFIXES lib)
#message(STATUS ${OpenNI2_LIBRARIES})

find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)
message(STATUS HELLO_WORLD_${OpenNI_INCLUDE_DIRS})
message(STATUS HELLO_WORLD_${OpenNI_LIBRARIES})

#pkg_check_modules(Nite REQUIRED XnVNite_1_5_2)
#message(STATUS HELLO_NITE_WORLD_${Nite_INCLUDE_DIRS})
#message(STATUS HELLO_NITE_WORLD_${Nite_LIBRARIES})


# Find Nite2
#message(status $ENV{NITE2_INCLUDE})
#message(status $ENV{NITE2_REDIST64})
#find_path(Nite2_INCLUDEDIR
#    NAMES NiTE.h
#    HINTS $ENV{NITE2_INCLUDE}
#)
#
#find_library(Nite2_LIBRARY
#    NAMES NiTE2
#    HINTS $ENV{NITE2_REDIST64}
#    PATH_SUFFIXES lib
#)

# - - - - -

#set(NITE2_INCLUDE "/home/evo/Desktop/NiTE-Linux-x64-2.2/Include/")
#set(NITE2_REDIST64 "/home/evo/Desktop/NiTE-Linux-x64-2.2/Redist/")

###Find pack for OpenNI2
#find_path(OpenNI2_INCLUDEDIR
#      NAMES OpenNI.h
#      HINTS /usr/include/openni2)
#find_library(OpenNI2_LIBRARIES
#         NAMES OpenNI2 DummyDevice OniFile PS1090
#         HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
#         PATH_SUFFIXES lib)
#message (STATUS ${OpenNI2_LIBRARIES})
###Find pack for NiTE2
#message(STATUS $ENV{NITE2_INCLUDE})
#message(STATUS $ENV{NITE2_REDIST64})
##/home/evo/Desktop/NiTE-Linux-x64-2.2/Include/
#find_path (NITE2_INCLUDEDIR
#           NAMES NiTE.h
#           HINTS "/home/evo/Desktop/NiTE-Linux-x64-2.2/Include/")
###/home/evo/Desktop/NiTE-Linux-x64-2.2/Redist/
#find_library (NITE2_LIBRARY
#          NAMES NiTE2
#          HINTS "/home/evo/Desktop/NiTE-Linux-x64-2.2/Redist/"
#          PATH_SUFFIXES lib)

# - - - -


add_message_files(
  FILES
  BodyDetections.msg
  SingleBodyDetection.msg
  OpenNiDetection.msg
  HOGdetection.msg
  TrackingState.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bender_follow_me
  CATKIN_DEPENDS message_runtime bender_msgs bender_srvs dynamic_reconfigure geometry_msgs nodelet pluginlib roscpp roslib rospy sensor_msgs std_msgs std_srvs cv_bridge image_transport pcl_ros bender_config tf bender_laser
  DEPENDS Eigen OpenNI opencv2
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${bender_config_INCLUDE_DIRS}
  ${bender_laser_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenNI_INCLUDE_DIRS}
  /usr/include/nite
#  /usr/include/ni/Linux-x86 # es para poder encontrar XnChar, XnBool, ...
  #${Nite_INCLUDE_DIRS}
)

# base_controller
#add_executable(base_controller src/BaseController.cpp)
#add_dependencies(base_controller ${catkin_EXPORTED_TARGETS})
#target_link_libraries(base_controller ${catkin_LIBRARIES})

# bayesian_filter
#add_executable(bayesian_filter src/BayesianFilter.cpp)
#add_dependencies(bayesian_filter ${catkin_EXPORTED_TARGETS})
#target_link_libraries(bayesian_filter ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

# ni_detector
#add_executable(ni_detector src/NIdetector.cpp)
#add_dependencies(ni_detector ${catkin_EXPORTED_TARGETS})
#target_link_libraries(ni_detector
#  ${catkin_LIBRARIES}
#  ${OpenNI_LIBRARIES}
#  ${Nite_LIBRARIES}
#  XnVNite_1_5_2
#)
#message("DEBUG variable catkin_INCLUDEDIR: ${catkin_INCLUDEDIR}")


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bender_follow_me.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

cmake_minimum_required(VERSION 2.8.3)
project(mp_tracker)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)


# Select a default build configuration if none was chosen
#set( CMAKE_BUILD_TYPE "Debug")
if(NOT CMAKE_BUILD_TYPE)
  set( CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()


## - - find packages - - 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  std_srvs
  bender_srvs
  bender_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  tf
)

find_package(OpenCV  REQUIRED)
find_package(PCL 1.7 REQUIRED)

# used by: HOG
find_package(JPEG REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(OpenMP) # Not required, but stronlgy recommended on multi-core systems
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS          "${CMAKE_C_FLAGS}          ${OpenMP_C_FLAGS}"  )
  set(CMAKE_CXX_FLAGS        "${CMAKE_CXX_FLAGS}        ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_C_FLAGS}"  )
endif()
#find_package(Eigen3 REQUIRED)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fpermissive -ggdb ")


## System dependencies are found with CMake's conventions

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare thinlgs to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mp_tracker
  CATKIN_DEPENDS cv_bridge geometry_msgs bender_srvs bender_msgs image_transport pcl_ros pcl_conversions roscpp roslib sensor_msgs std_msgs std_srvs tf
  DEPENDS  PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${FLANN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(${PYTHON_LIBRARIES})

#add_subdirectory(external/Bayes++)
#add_subdirectory(external/bayestracking)
add_subdirectory(external/libdetect)
#include_directories(external/bayestracking/src)
include_directories(external/libdetect/src)
#include_directories(external/Bayes++)


#add_executable(mp_tracker_node
#  src/mp_tracker_node.cpp
#  src/mp_tracker.cpp 
#  src/map.cpp src/util.cpp 
#  src/pointcloud_process.cpp
#)

add_executable(d_tracker_node src/d_tracker_node.cpp)
add_executable(d_tracker_plus_node src/d_tracker_plus_node.cpp)
add_library(d_tracker src/d_tracker.cpp src/util.cpp src/pointcloud_process.cpp)

#add_dependencies(mp_tracker_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(mp_tracker_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${pcl_LIBRARIES}  ${PCL_LIBS} ${OpenCV_LIBRARIES} "-fopenmp")
target_link_libraries(d_tracker_node d_tracker detect ${PYTHON_LIBRARIES} boost_python gsl gslcblas ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${pcl_LIBRARIES}  ${PCL_LIBS} ${OpenCV_LIBRARIES})
target_link_libraries(d_tracker_plus_node d_tracker detect ${PYTHON_LIBRARIES} boost_python gsl gslcblas ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${pcl_LIBRARIES}  ${PCL_LIBS} ${OpenCV_LIBRARIES})


# The End :)
#



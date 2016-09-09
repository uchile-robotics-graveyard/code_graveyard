
if(EXISTS "$ENV{HOME}/.ros/NiTE2/s.dat")

set(NITE2_DIR ~/NiTE-2.0.0/)
set(NITE2_LIB ~/NiTE-2.0.0/Redist/libNiTE2.so)
link_directories(${NITE2_DIR}/Redist)
include_directories(${NITE2_DIR}/Include)
include_directories(/usr/include/ni/)
include_directories(/usr/include/openni2/)

## Kinect Modules
## ----------------------------------------------------------
#
## -- KinectTracker --
 add_executable(kinect_tracker src/tracking/kinect_tracker/KinectTracker.cpp)
 add_dependencies(kinect_tracker ${catkin_EXPORTED_TARGETS})
 target_link_libraries(kinect_tracker ${catkin_LIBRARIES}  ${OPENNI_LIBRARIES} ${OpenCV_LIBRARIES} OpenNI2 NiTE2 ${NITE2_LIB} )


## -- Oclusiones --
#add_executable(kinect_occlusion_detector src/tracking/kinect_tracker/Oclusiones.cpp)

 
else()

  message("--------NITE file doesn't exist--------")
  message("--------NITE nodes not compiled--------")
endif()


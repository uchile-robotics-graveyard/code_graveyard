
find_path(LIBUSB_INCLUDEDIR
    NAMES libusb.h
    HINTS /usr/include/libusb-1.0
)
find_library(LIBUSB_LIBRARIES
    NAMES usb-1.0
    HINTS /usr/lib/ /usr/x86_64-linux-gnu/
    PATH_SUFFIXES lib
)

message(STATUS ${LIBUSB_INCLUDEDIR})
message(STATUS ${LIBUSB_LIBRARIES})

catkin_package(
  DEPENDS  LIBUSB_LIBRARIES
)

###########
## Build ##
###########

include_directories(
    ${LIBUSB_INCLUDE_DIRS}
)


## kinect_aux
add_executable(kinect_aux_node src/kinect/kinect_aux.cpp)
add_dependencies(kinect_aux_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinect_aux_node
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
    ${LIBUSB_LIBRARIES} 
)

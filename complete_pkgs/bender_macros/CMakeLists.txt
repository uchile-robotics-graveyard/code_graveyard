cmake_minimum_required(VERSION 2.8.3)
project(bender_macros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  std_msgs
  std_srvs
  bender_msgs
  bender_srvs
)


catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bender_macros
#  CATKIN_DEPENDS bender_utils bender_laser actionlib actionlib_msgs roscpp rospy smach smach_ros std_msgs std_srvs bender_msgs bender_srvs bender_utils nav_msgs bender_vision bender_nav bender_sensors bender_joy bender_fun bender_speech tf
#  DEPENDS
)

###########
## Build ##
###########

include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#  ${bender_utils_INCLUDE_DIRS}
)


# Person Blocking Macros
# - - - - - - - - - - - - - - - - - - - - - -

# #  - - Person Blocking Detector - -
# add_executable(person_blocking_detector src/PersonBlockingDetector.cpp)
# add_dependencies(person_blocking_detector ${catkin_EXPORTED_TARGETS} )
# target_link_libraries(person_blocking_detector ${catkin_LIBRARIES})

# # Enroll  Macros
# # - - - - - - - - - - - - - - - - - - - - - -

# #  - - Enroll Person - -
# add_executable(enroll_person src/EnrollPerson.cpp)
# add_dependencies(enroll_person ${catkin_EXPORTED_TARGETS})
# target_link_libraries(enroll_person ${catkin_LIBRARIES})


# # Search  Macros
# # - - - - - - - - - - - - - - - - - - - - - -

# #  - -  Search Face - -
# add_executable(search_face  src/SearchFace.cpp)
# add_dependencies(search_face ${catkin_EXPORTED_TARGETS})
# target_link_libraries(search_face ${catkin_LIBRARIES})


# #  - - Search Waving - -
# add_executable(search_waving src/SearchWaving.cpp)
# add_dependencies(search_waving ${catkin_EXPORTED_TARGETS})
# target_link_libraries(search_waving ${catkin_LIBRARIES})


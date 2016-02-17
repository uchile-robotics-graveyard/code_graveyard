cmake_minimum_required(VERSION 2.8.3)
project(bender_arm)

find_package(catkin REQUIRED COMPONENTS
  bender_msgs
  bender_srvs
  dynamixel_controllers
  dynamixel_msgs
  roscpp
  rospy
  std_msgs
)

find_package(OpenCV  REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bender_arm
  CATKIN_DEPENDS bender_msgs bender_srvs dynamixel_controllers dynamixel_msgs roscpp rospy std_msgs
  DEPENDS opencv2
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)


## -------------- C O N T R O L A D O R E S --------------##

## Brazo Izquierdo ##

add_executable(left_arm_controller_interface src/left_arm_controller_interface.cpp)
add_dependencies(left_arm_controller_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(left_arm_controller_interface ${catkin_LIBRARIES})

add_executable(left_arm_service_interface src/l_interface_plan_arm.cpp)
add_dependencies(left_arm_service_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(left_arm_service_interface ${catkin_LIBRARIES})

## Brazo Derecho ##

add_executable(right_arm_controller_interface src/right_arm_controller_interface.cpp)
add_dependencies(right_arm_controller_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(right_arm_controller_interface ${catkin_LIBRARIES})

add_executable(right_arm_service_interface src/r_interface_plan_arm.cpp)
add_dependencies(right_arm_service_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(right_arm_service_interface ${catkin_LIBRARIES}  )

## Brazo Base ## 

add_executable(bottom_arm_controller_interface src/bottom_arm_controller_interface.cpp)
add_dependencies(bottom_arm_controller_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(bottom_arm_controller_interface ${catkin_LIBRARIES})

add_executable(bottom_arm_service_interface src/bottom_arm_service_interface.cpp)
add_dependencies(bottom_arm_service_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(bottom_arm_service_interface ${catkin_LIBRARIES} )


##------------- B E N D E R  P L A N N I N G ---------------##

set(BENDER_PLANNING_OLD_HEADER
	include/bender_planning_old/planner.h
	include/bender_planning_old/hMatrix.h
	include/bender_planning_old/IKNode.h
	include/bender_planning_old/OMNode.h
	include/bender_planning_old/punto.h
	include/bender_planning_old/QueryTimer.h
	include/bender_planning_old/spline.h
)

set(BENDER_PLANNING_OLD_LEFT_SOURCE
	src/left_planner.cpp
	src/IKNode.cpp
	src/OMNode.cpp
	src/QueryTimer.cpp
	src/hMatrix.cpp
	src/Main_left.cpp
)

set(BENDER_PLANNING_OLD_RIGHT_SOURCE
	src/right_planner.cpp
	src/IKNode.cpp
	src/OMNode.cpp
	src/QueryTimer.cpp
	src/hMatrix.cpp
	src/Main_right.cpp
)

add_executable(LeftArmPlanner ${BENDER_PLANNING_OLD_LEFT_SOURCE} ${BENDER_PLANNING_OLD_HEADER})
add_dependencies(LeftArmPlanner ${catkin_EXPORTED_TARGETS})
target_link_libraries(LeftArmPlanner ${catkin_LIBRARIES} )

add_executable(RightArmPlanner ${BENDER_PLANNING_OLD_RIGHT_SOURCE} ${BENDER_PLANNING_OLD_HEADER})
add_dependencies(RightArmPlanner ${catkin_EXPORTED_TARGETS})
target_link_libraries(RightArmPlanner ${catkin_LIBRARIES} )


##----------- T O R Q U E --------------##

add_executable(r_torque src/right_cinematica.cpp)
add_dependencies(r_torque ${catkin_EXPORTED_TARGETS})
target_link_libraries(r_torque ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(l_torque src/left_cinematica.cpp)
add_dependencies(l_torque ${catkin_EXPORTED_TARGETS})
target_link_libraries(l_torque ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )


## ------------- B A S E ---------------- ##

add_executable(state_publisher src/state_publisher.cpp)
add_dependencies(state_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_publisher ${catkin_LIBRARIES} )

#add_executable(baseArm_Controller src/baseArm_Controller.cpp)
#target_link_libraries(baseArm_Controller ${catkin_LIBRARIES})

#add_executable(baseArm_Controller2 src/baseArm_Controller2.cpp)
#target_link_libraries(baseArm_Controller2 ${catkin_LIBRARIES})


## ------------- K I N E M A T I C S ------------ ##

add_executable(fknode src/fknode.cpp)
add_dependencies(fknode ${catkin_EXPORTED_TARGETS})
target_link_libraries(fknode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(iknode src/iknode.cpp)
add_dependencies(iknode ${catkin_EXPORTED_TARGETS})
target_link_libraries(iknode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )


## -------------------------------- ##



## --------------- T E S T ---------------- ##

#add_executable(baseArm_Controller_mod test_src/baseArm_Controller_modified.cpp)
#target_link_libraries(baseArm_Controller_mod ${catkin_LIBRARIES})

add_executable(macros test_src/macro_services.cpp)
add_dependencies(macros ${catkin_EXPORTED_TARGETS})
target_link_libraries(macros ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(grab_object_macro src/macros/grab_object.cpp)
add_dependencies(grab_object_macro ${catkin_EXPORTED_TARGETS})
target_link_libraries(grab_object_macro ${catkin_LIBRARIES})

##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(bender_gui_launch)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	bender_msgs
	bender_srvs
)
include_directories(${catkin_INCLUDE_DIRS})

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

catkin_python_setup()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml


##############################################################################
# Sections
##############################################################################


##############################################################################
# Sources
##############################################################################



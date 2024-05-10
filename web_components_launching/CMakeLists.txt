cmake_minimum_required(VERSION 3.0.2)
project(web_components_launching)

# Find catkin and any catkin packages on which your project depends.
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  visualization_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs visualization_msgs
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

# Install all launch files from the launch directory
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Python executable scripts
catkin_install_python(PROGRAMS src/marker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

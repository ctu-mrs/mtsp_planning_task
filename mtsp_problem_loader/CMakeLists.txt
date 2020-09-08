cmake_minimum_required(VERSION 2.8.3)
project(mtsp_problem_loader)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nodelet
  cmake_modules
  mtsp_msgs
  )

###############################################
## Declare ROS messages, services and actions ##
################################################

catkin_python_setup()

catkin_package(
  #  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet rospy std_msgs message_runtime mtsp_msgs
  # LIBRARIES
  # DEPENDS
  LIBRARIES mtsp_problem_loader
  )

file(GLOB python_scripts_DIR
    "scripts/*.py"
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
 
catkin_install_python(PROGRAMS ${python_scripts_DIR}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

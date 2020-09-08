cmake_minimum_required(VERSION 2.8.3)
project(mtsp_planner)

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
  mrs_lib
  mrs_msgs
  mtsp_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gdax_interface
   CATKIN_DEPENDS nodelet rospy std_msgs message_runtime mrs_msgs mrs_lib mtsp_msgs
#  LIBRARIES 
#  DEPENDS Eigen
)

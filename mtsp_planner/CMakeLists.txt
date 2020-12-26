cmake_minimum_required(VERSION 3.1.2)
project(mtsp_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
   CATKIN_DEPENDS nodelet rospy std_msgs message_runtime mrs_msgs mrs_lib mtsp_msgs
)

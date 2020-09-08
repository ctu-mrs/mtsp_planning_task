cmake_minimum_required(VERSION 2.8.3)
project(mtsp_msgs)

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
  message_runtime
  message_generation
  )

###############################################
## Declare ROS messages, services and actions ##
################################################

# messages for MPC_TRACKER
add_message_files(DIRECTORY msg FILES
  TspPoint.msg
  TspProblem.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  )

catkin_package(
  #  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime message_generation message_runtime
  # LIBRARIES
  # DEPENDS
  LIBRARIES mtsp_msgs
  )

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(mtsp_msgs src/main.cpp)

add_dependencies(mtsp_msgs
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(mtsp_msgs
  ${catkin_LIBRARIES}
  )

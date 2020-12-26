cmake_minimum_required(VERSION 3.1.2)
project(mtsp_msgs)

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
  message_runtime
  message_generation
  )

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

cmake_minimum_required(VERSION 2.8.3)
project(mtsp_state_machine)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

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
  mavros_msgs
)

catkin_package(
  # INCLUDE_DIRS include
   CATKIN_DEPENDS nodelet rospy std_msgs message_runtime mrs_msgs mrs_lib mtsp_msgs mavros_msgs
   LIBRARIES MtspStateMachine
   DEPENDS Eigen
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  )

add_library(MtspStateMachine src/mtsp_state_machine.cpp)

add_dependencies(MtspStateMachine
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MtspStateMachine
  ${catkin_LIBRARIES}
  )

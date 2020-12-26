cmake_minimum_required(VERSION 3.1.2)
project(mtsp_problem_loader)

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
  mtsp_msgs
  )

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS nodelet rospy std_msgs message_runtime mtsp_msgs
  LIBRARIES mtsp_problem_loader
  )

file(GLOB python_scripts_DIR
  "scripts/*.py"
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS ${python_scripts_DIR}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cmake_minimum_required(VERSION 2.8.3)
project(jetyak_uav_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  dji_sdk
  visualization_msgs
  message_generation
)

## Custom message files to be created
add_message_files(
  FILES
  Mode.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)

## includes, libraries, depends
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jetyak_uav_utils
  CATKIN_DEPENDS ar_track_alvar geometry_msgs ar_track_alvar_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf dji_sdk visualization_msgs
  DEPENDS system_lib
)

## Includes
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/pid.cpp
)

## Add executables
add_executable(controller_node src/controller.cpp)
add_executable(take_off_follow_node src/take_off_follow.cpp)

## Link executables
target_link_libraries(controller_node
  ${catkin_LIBRARIES}
)
target_link_libraries(take_off_follow_node
  ${catkin_LIBRARIES}
)

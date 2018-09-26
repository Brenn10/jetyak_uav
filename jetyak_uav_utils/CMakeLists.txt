cmake_minimum_required(VERSION 2.8.3)
project(jetyak_uav_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  dji_sdk
  visualization_msgs
  message_generation
  actionlib_msgs
)

find_package(DJIOSDK REQUIRED)

## Custom message files to be created
#add_message_files(DIRECTORY msg
#  FILES
#  ModeMsg.msg
#)
add_service_files(DIRECTORY srv
  FILES
  Mode.srv
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  geometry_msgs
#  nav_msgs
#  sensor_msgs
#  std_msgs
#  visualization_msgs
#)

generate_messages(
  DEPENDENCIES
  jetyak_uav_utils
)
generate_dynamic_reconfigure_options(
  cfg/FollowConstants.cfg
  cfg/LandConstants.cfg
)
## includes, libraries, depends
catkin_package(
  INCLUDE_DIRS include lib/bsc_common/include lib/dji_virtual_rc/include
  LIBRARIES jetyak_uav_utils
  CATKIN_DEPENDS dynamic_reconfigure ar_track_alvar geometry_msgs ar_track_alvar_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf dji_sdk visualization_msgs message_runtime
  DEPENDS system_lib
)

## Includes
include_directories(
  include
  lib/bsc_common/include
  lib/dji_virtual_rc/include
  ${catkin_INCLUDE_DIRS}
)

## Add executables
add_executable(virtual_rc_N3
                src/virtual_rc_N3.cpp
                lib/dji_virtual_rc/virtual_rc.cpp)

add_executable(behaviors_node
                src/behaviors.cpp
                lib/bsc_common/pid.cpp
                lib/bsc_common/util.cpp)

#add dependencies
add_dependencies(virtual_rc_N3 ${catkin_EXPORTED_TARGETS} )
add_dependencies(behaviors_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

## Link executables
target_link_libraries(virtual_rc_N3
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)
target_link_libraries(behaviors_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)

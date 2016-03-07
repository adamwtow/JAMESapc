cmake_minimum_required(VERSION 2.8.3)
project(pc_sub_repub)
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
install(
   PROGRAMS scripts/bag_sub_repub.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

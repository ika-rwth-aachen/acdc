cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(rosbag_noise)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  rospy
  definitions
)

catkin_package(
  CATKIN_DEPENDS definitions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

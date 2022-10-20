cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(camera_based_semantic_grid_mapping)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  tf
)

# Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

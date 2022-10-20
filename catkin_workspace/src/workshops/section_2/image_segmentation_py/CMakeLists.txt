cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(image_segmentation_py)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

# Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
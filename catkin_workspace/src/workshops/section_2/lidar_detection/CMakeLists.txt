cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)

project(lidar_detection)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  roslib
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  tensorflow_cpp
  definitions
)

# already save includes and libs of required packages
set(MY_LINK_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
set(MY_LINK_LIBRARIES ${catkin_LIBRARIES})

# Boost thread and boost optionals are required
find_package(Boost REQUIRED)

# Eigen for tensor calculations
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  list(REMOVE_ITEM MY_LINK_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
endif()

catkin_package(CATKIN_DEPENDS
  nodelet
  roscpp
  roslib
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  tensorflow_cpp
)

add_library(lidar_detection
  src/lidar_detection.cpp
  src/definitions.cpp
  src/list_creator.cpp
  src/detector.cpp
  src/pillar_utils.cpp
)
target_compile_definitions(lidar_detection PUBLIC MODE_PP)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fext-numeric-literals ")
set(CMAKE_BUILD_TYPE Release)

target_include_directories(lidar_detection PUBLIC
  include
  ${MY_LINK_INCLUDE_DIRS}
)

add_dependencies(lidar_detection ${lidar_detection_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_detection
  ${MY_LINK_LIBRARIES}
)

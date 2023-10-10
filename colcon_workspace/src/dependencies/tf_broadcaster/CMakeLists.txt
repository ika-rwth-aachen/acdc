cmake_minimum_required(VERSION 3.8)
project(tf_broadcaster)

# Workaround to find GeographicLib according to https://bugs.launchpad.net/ubuntu/+source/geographiclib/+bug/1805173
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# TfBroadcasterNode
add_executable(tf_broadcaster_node src/TfBroadcasterNode.cpp)

target_include_directories(tf_broadcaster_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tf_broadcaster_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(tf_broadcaster_node
  GeographicLib
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)

install(TARGETS tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

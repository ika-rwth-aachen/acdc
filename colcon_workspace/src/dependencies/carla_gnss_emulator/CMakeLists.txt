cmake_minimum_required(VERSION 3.8)
project(carla_gnss_emulator)

# Workaround to find GeographicLib according to https://bugs.launchpad.net/ubuntu/+source/geographiclib/+bug/1805173
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# GNSSEmulatorNode
add_executable(gnss_emulator_node src/GNSSEmulatorNode.cpp)

target_include_directories(gnss_emulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gnss_emulator_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(gnss_emulator_node
  GeographicLib  
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS gnss_emulator_node
  DESTINATION lib/${PROJECT_NAME})

# Install Launch- and Config-Files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)

#   ament_add_gtest(gnss_localization_node-test test/test_gnss_localization_node.cpp)
#   target_include_directories(gnss_localization_node-test PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
#   target_include_directories(gnss_localization_node-test PUBLIC test)
#   ament_target_dependencies(gnss_localization_node-test 
#     GeographicLib  
#     geometry_msgs
#     nav_msgs
#     rclcpp
#     sensor_msgs
#   )
# endif()


ament_package()

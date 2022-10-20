# For usage of the mdoern cmake features a high version of CMake is requierd.
cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

# Prefent this CMakeLists to be read multiple times in a single project.
include_guard(GLOBAL)

# Define library meta information her. Since CMake 3.0.0 a version and language deinfintion is required.
project(acdc_launchpack VERSION 0.1 LANGUAGES CXX)

# Set Cxx11 the standard of this project. This makes a compiler that supports Cxx11 necesarry.
set(CMAKE_CXX_STANDARD 11)
# Enforce Cxx11 to be used by the compiler.
set(CMAKE_CXX_STANDARD_REQUIRED 11)

# Find catkin and all required components. Roscpp is the implementation of ROS und Cxx. The Sensor-Msgs are used to
# send the laserscanner data. The Geometry-Msgs are used to send the target vehicle state.
find_package(catkin REQUIRED REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  definitions)

# Create the catkin package that depends on all requirements mentions above.
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  definitions)

# Add executable that uses ROS to transform the ika objects into the correct frame.
add_executable(transformation_node src/transformation_node.cpp)

# Link the node to the ROS packages used to encode send and receive signals.
add_dependencies(transformation_node ${catkin_EXPORTED_TARGETS})
target_include_directories(transformation_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(transformation_node ${catkin_LIBRARIES})

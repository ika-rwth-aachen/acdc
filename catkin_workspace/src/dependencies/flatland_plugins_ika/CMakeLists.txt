# For usage of the mdoern cmake features a high version of CMake is requierd.
cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

# Prefent this CMakeLists to be read multiple times in a single project.
include_guard(GLOBAL)

# Define library meta information her. Since CMake 3.0.0 a version and language deinfintion is required.
project(flatland_ika_plugins VERSION 0.1 LANGUAGES CXX)

# Set Cxx11 the standard of this project. This makes a compiler that supports Cxx11 necesarry.
set(CMAKE_CXX_STANDARD 14)
# Enforce Cxx11 to be used by the compiler.
set(CMAKE_CXX_STANDARD_REQUIRED 14)

# Find catkin and all required components.
find_package(catkin REQUIRED COMPONENTS
  rostest
  pluginlib
  roscpp
  rospy
  flatland_server
  flatland_plugins
  cmake_modules
  tf
  flatland_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  definitions
  angles)

# Create the catkin package that depends on all requirements mentions above.
catkin_package(INCLUDE_DIRS include
  LIBRARIES flatland_ika_plugins_lib
  CATKIN_DEPENDS pluginlib roscpp flatland_server std_msgs geometry_msgs visualization_msgs definitions angles tf flatland_msgs)

# Declare a C++ library that contains the single-track-drive.
add_library(flatland_ika_plugins_lib
  src/drives/SingleTrackDrive.cpp
  src/drives/SimulatedDrive.cpp
  src/drives/SimulatedTire.cpp
  src/sensors/LaserScanner.cpp
  src/sensors/LaserCallback.cpp
  src/drives/TrajectoryDrive.cpp)

# Define headers for this library. PUBLIC headers are used by compiling by the external developers, and will be added to
# consumers' build paths.
target_include_directories(flatland_ika_plugins_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Set the install directories to the correct locations (provided by catkin).
install(TARGETS flatland_ika_plugins_lib EXPORT FlatLandIkaConfig
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install plugin file
install(FILES flatland_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Link the node to the ROS packages.
add_dependencies(flatland_ika_plugins_lib ${catkin_EXPORTED_TARGETS})
target_include_directories(flatland_ika_plugins_lib PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(flatland_ika_plugins_lib ${catkin_LIBRARIES})


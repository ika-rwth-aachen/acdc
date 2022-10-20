cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR) # required for modern CMake features

include_guard(GLOBAL) # this CMakeLists can only be read once in this project

project(racing VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11) # compiler that supports C++11 is necessary
set(CMAKE_CXX_STANDARD_REQUIRED  11) # C++11 must be used by the compiler

# Find catkin and all required components.
# - roscpp: client library of ROS in Cpp
# - sensor_msgs: package containing lidar scan message definition
# - geometry_msgs: package containing "twist" message for target values for actuators
find_package(catkin REQUIRED REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs)

# Create the catkin package that depends on all requirements mentions above
catkin_package(INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs)

# add a building output: a library independent of ROS
add_library(racing_library
    src/VehicleController.cpp)

# add a building output: a ROS-node that wraps the ROS-independent library
add_executable(vehicle_controller_node src/vehicle_controller_node.cpp)

add_executable(vehicle_timer_node src/vehicle_timer_node.cpp)

# Define headers for this library. PUBLIC headers are used by compiling by the exteral developers, and will be added to
# consumers' build paths.
target_include_directories(racing_library PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Set the install directories to the correct locations (provided by catkin)
install(TARGETS racing_library vehicle_controller_node EXPORT RacingLibraryConfig
    ARCHIVE  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Link the ROS node to the ROS-independent libraray
target_link_libraries(vehicle_controller_node racing_library)

# Link the ROS node to the ROS packages used to encode send and receive signals
add_dependencies(vehicle_controller_node ${catkin_EXPORTED_TARGETS})
target_include_directories(vehicle_controller_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(vehicle_controller_node ${catkin_LIBRARIES})

# Link the node to the ROS packages used to encode send and receive signals.
add_dependencies(vehicle_timer_node ${catkin_EXPORTED_TARGETS})
target_include_directories(vehicle_timer_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(vehicle_timer_node ${catkin_LIBRARIES})

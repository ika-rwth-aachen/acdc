cmake_minimum_required(VERSION 3.8) # required for modern CMake features

include_guard(GLOBAL) # this CMakeLists can only be read once in this project

project(racing VERSION 0.1 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament and all required components.
# - rclcpp: client library of ROS in Cpp
# - sensor_msgs: package containing lidar scan message definition
# - geometry_msgs: package containing "twist" message for target values for actuators
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# Create the ament package that depends on all requirements mentions above
#ament_package(
#    include
#    LIBRARIES ${PROJECT_NAME}
#    rclcpp
#    sensor_msgs
#    geometry_msgs)

include_directories(include())


# add a building output: a library independent of ROS
add_library(racing_library
    src/VehicleController.cpp)

# Define headers for this library. PUBLIC headers are used by compiling by the exteral developers, and will be added to
# consumers' build paths.
target_include_directories(racing_library PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)


# add a building output: a ROS-node that wraps the ROS-independent library
add_executable(vehicle_controller_node src/vehicle_controller_node.cpp)

# Link the ROS node to the ROS-independent libraray
target_link_libraries(vehicle_controller_node racing_library)

# Link the ROS node to the ROS packages used to encode send and receive signals
ament_target_dependencies(vehicle_controller_node
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
)

add_executable(vehicle_timer_node src/vehicle_timer_node.cpp)

# Link the node to the ROS packages used to encode send and receive signals.
ament_target_dependencies(vehicle_timer_node
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
)

#target_include_directories(vehicle_controller_node PRIVATE ${catkin_INCLUDE_DIRS})
#target_link_libraries(vehicle_controller_node ${catkin_LIBRARIES})


# Set the install directories to the correct locations


install(
    DIRECTORY include launch config
    DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  vehicle_controller_node
  vehicle_timer_node
  racing_library
  DESTINATION lib/${PROJECT_NAME}
)

#target_include_directories(vehicle_timer_node PRIVATE ${catkin_INCLUDE_DIRS})
#target_link_libraries(vehicle_timer_node ${catkin_LIBRARIES})

ament_package()

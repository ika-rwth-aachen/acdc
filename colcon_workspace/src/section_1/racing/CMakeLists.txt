cmake_minimum_required(VERSION 3.8) # required for modern CMake features

include_guard(GLOBAL) # this CMakeLists can only be read once in this project

project(racing)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})
add_compile_definitions(ROS_DISTRO=$ENV{ROS_DISTRO})

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
# - nav_msgs: package containing "odometry" message for current state of the vehicle
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
add_library(${PROJECT_NAME}_library SHARED
    src/VehicleController.cpp)

# Define headers for this library. PUBLIC headers are used by compiling by the exteral developers, and will be added to
# consumers' build paths.
target_include_directories(${PROJECT_NAME}_library PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)
target_compile_features(${PROJECT_NAME}_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# add a building output: a ROS-node that wraps the ROS-independent library
add_executable(vehicle_controller_node src/vehicle_controller_node.cpp)
# Link the ROS node to the ROS-independent library
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


# Set the install directories to the correct locations
install(
    DIRECTORY include launch config
    DESTINATION share/${PROJECT_NAME}/
)

# Install the executables
install(TARGETS
  vehicle_controller_node
  vehicle_timer_node
  racing_library
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)

# define ament_lint config files
set(ament_cmake_clang_format_CONFIG_FILE ${dev_environment_INCLUDE_DIRS}/.clang-format)
set(ament_cmake_clang_tidy_CONFIG_FILE ${dev_environment_INCLUDE_DIRS}/.clang-tidy)
set(ament_cmake_flake8_CONFIG_FILE ${dev_environment_INCLUDE_DIRS}/ament_flake8.ini)

ament_lint_auto_find_test_dependencies()
endif()

ament_package()

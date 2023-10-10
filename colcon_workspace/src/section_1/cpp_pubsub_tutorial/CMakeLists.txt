cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub_tutorial)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})
add_compile_definitions(ROS_DISTRO=$ENV{ROS_DISTRO})


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


#Add executables for publisher and subscriber
add_executable(publisher src/publisher_node.cpp)
# Link the ROS node to the ROS packages used to encode send and receive signals
ament_target_dependencies(publisher 
    rclcpp 
    std_msgs)

add_executable(subscriber src/subscriber_node.cpp)
# Link the ROS node to the ROS packages used to encode send and receive signals
ament_target_dependencies(subscriber 
    rclcpp 
    std_msgs)

# Set the install directories to the correct locations
install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}/
)

# Install the executables
install(TARGETS
  publisher
  subscriber
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

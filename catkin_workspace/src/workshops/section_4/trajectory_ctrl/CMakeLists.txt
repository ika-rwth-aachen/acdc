# CMake is requierd
cmake_minimum_required(VERSION 2.8.3)

# Specify project name and version number
project(trajectory_ctrl)

# Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  definitions
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/trj_ctrl.cfg
)

# Include catkin dependencies
catkin_package(CATKIN_DEPENDS
  definitions
  roscpp
  std_msgs
  dynamic_reconfigure
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include/trajectory_ctrl
)

# Add actual node and linking targets
add_executable(trajectory_control_node src/TrajectoryCtrl.cpp src/PID.cpp)

# Set the C++ standard to C++14
target_compile_features(trajectory_control_node PUBLIC cxx_std_14)

add_dependencies(trajectory_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_control_node ${catkin_LIBRARIES})

# Include header files
target_include_directories(trajectory_control_node PUBLIC
    include/trajectory_ctrl
    ${catkin_INCLUDE_DIRS}
)

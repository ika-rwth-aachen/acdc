# For usage of the cmake features "IMPORTED Targets" a high version of
# CMake is requierd
cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

# Specify project name and version number
project(object_fusion_wrapper VERSION 0.1 LANGUAGES CXX)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  definitions
  object_fusion
)
# Include catkin dependencies
catkin_package(CATKIN_DEPENDS
  message_runtime
  roscpp
  definitions
  object_fusion
)

# Find package boost
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Eigen3 REQUIRED)

# Add actual node and linking targets
add_executable(object_fusion_wrapper_node src/IkaFusionWrapperNode.cpp)

# set(CMAKE_BUILD_TYPE Release)

# Set the C++ standard to C++14
target_compile_features(object_fusion_wrapper_node PUBLIC cxx_std_14)

add_dependencies(object_fusion_wrapper_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_fusion_wrapper_node ${catkin_LIBRARIES} Boost::boost Boost::filesystem)

# Include header files
target_include_directories(object_fusion_wrapper_node PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

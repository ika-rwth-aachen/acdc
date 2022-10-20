cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(v2x_spat_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  definitions
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS   
   roscpp
   definitions 
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# SPaT Generator Node
add_executable(v2x_spat_generator_node src/spat_generator_node.cpp)
add_dependencies(v2x_spat_generator_node ${v2x_spat_generator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_features(v2x_spat_generator_node PUBLIC cxx_std_11)

target_include_directories(v2x_spat_generator_node PUBLIC
 include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(v2x_spat_generator_node
  ${catkin_LIBRARIES}
)
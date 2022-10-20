cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(etsi_message_generation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    nodelet
    roscpp
    roslaunch
    definitions
    tf2_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS message_runtime nodelet roscpp definitions tf2_ros
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## EXECUTABLES START ##
add_executable(${PROJECT_NAME}_cam_generator src/cam_generator.cpp)
## EXECUTABLES END ##

## TARGET PROPERTIES START ##
set_target_properties(${PROJECT_NAME}_cam_generator PROPERTIES OUTPUT_NAME cam_generator)
## TARGET PROPERTIES END ##

## DEPENDENCIES START ##
add_dependencies(${PROJECT_NAME}_cam_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## DEPENDENCIES END ##

## LINKING START ##
target_link_libraries(${PROJECT_NAME}_cam_generator ${catkin_LIBRARIES})
## LINKING END ##


#############
## Install ##
#############
install(TARGETS 
    ${PROJECT_NAME}_cam_generator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  ## nodelet_plugins.xml ##
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
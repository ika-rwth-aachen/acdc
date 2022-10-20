cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(etsi_visualization)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED 14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  definitions 
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS   
   roscpp
   tf2_ros
   definitions 
   visualization_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## EXECUTABLES START ##
add_executable(${PROJECT_NAME}_ETSIViz src/ETSIViz.cpp src/SPATViz.cpp src/MAPViz.cpp)
## EXECUTABLES END ##

## TARGET PROPERTIES START ##
set_target_properties(${PROJECT_NAME}_ETSIViz PROPERTIES OUTPUT_NAME ETSIViz)
## TARGET PROPERTIES END ##

## DEPENDENCIES START ##
add_dependencies(${PROJECT_NAME}_ETSIViz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## DEPENDENCIES END ##

## LINKING START ##
target_link_libraries(${PROJECT_NAME}_ETSIViz ${catkin_LIBRARIES})
## LINKING END ##

#############
## Install ##
#############
install(TARGETS 
    ${PROJECT_NAME}_ETSIViz
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  #nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
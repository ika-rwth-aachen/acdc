cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(mqtt_launchpack)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslaunch
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS roscpp std_msgs
)

#############
## Install ##
#############
install(TARGETS 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  ## nodelet_plugins.xml ##
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
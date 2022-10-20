cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)
project(definitions)

add_compile_options(-std=c++14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    visualization_msgs
    message_generation
    tf2
)

find_package(Eigen3 REQUIRED)

add_message_files(DIRECTORY msg)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    visualization_msgs
)

# Link build dependencies to the message files
catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    visualization_msgs
    message_runtime
    INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

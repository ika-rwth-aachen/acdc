# For usage of the cmake features "IMPORTED Targets" a high version of
# CMake is requierd
cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)

# Specify project name and version number
project(rviz_plugins VERSION 0.1 LANGUAGES CXX)

# Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  rviz
  definitions
)

# Include catkin dependencies
catkin_package(CATKIN_DEPENDS definitions)

link_directories(${catkin_LIBRARY_DIRS})

# This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

# This plugin includes Qt widgets, so we must include Qt.
# We'll use the version that rviz used so they are compatible.
if (rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  # pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else ()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  # make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif ()

# Avoid qt singnal keywords to avoid conflicts with boost singal ones.
add_definitions(-DQT_NO_KEYWORDS)

# Adding the library for object lists
add_library(rviz_plugins_object_list
    src/visualisation/IkaObjectList/IkaObjectListDisplay.cpp
    src/visualisation/IkaObjectList/IkaObjectVisual.cpp
    src/visualisation/IkaObjectList/IkaObjectVisualSelectionHandler.cpp
    src/visualisation/IkaObjectList/AngleProperty.cpp
    src/overlay/IkaObjectListOverlay.cpp
    src/overlay/IkaOverlayUtils.cpp)

# Adding the library for detection lists
add_library(rviz_plugins_detection_list
    src/visualisation/IkaDetectionList/IkaDetectionListDisplay.cpp
    src/visualisation/IkaDetectionList/IkaDetectionVisual.cpp
    src/visualisation/IkaDetectionList/IkaDetectionVisualSelectionHandler.cpp)

# Set the C++ standard to C++14
target_compile_features(rviz_plugins_object_list PUBLIC cxx_std_14)
target_compile_features(rviz_plugins_detection_list PUBLIC cxx_std_14)

target_link_libraries(rviz_plugins_object_list ${QT_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(rviz_plugins_detection_list ${QT_LIBRARIES} ${catkin_LIBRARIES})

# Include header files
target_include_directories(rviz_plugins_object_list PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(rviz_plugins_detection_list PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

install(TARGETS
    rviz_plugins_object_list
    rviz_plugins_detection_list
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

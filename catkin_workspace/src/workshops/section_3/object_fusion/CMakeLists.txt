# For usage of the cmake features "IMPORTED Targets" a high version of
# CMake is requierd
cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

# Makes a multi add_subdirectory police possible do to abbility of debugging in clion.
include_guard(GLOBAL)

# Define library. Only source files here!
project(object_fusion VERSION 0.3 LANGUAGES CXX)

# Adding CMake flags
set(CMAKE_CXX_FLAGS "-fPIC ${CMAKE_CXX_FLAGS}")

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Finding Boost
find_package(Boost REQUIRED COMPONENTS filesystem)

# Finding Eigen
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    definitions
)

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS definitions
  LIBRARIES ${PROJECT_NAME}
)

add_library(object_fusion
    src/ikaFusLib.cpp
    src/data/Data.cpp
    src/data/Params.cpp
    src/modules/AbstractFusionModule.cpp
    src/modules/fuser/ClassificationFuser.cpp
    src/modules/fuser/ExistenceFuser.cpp
    src/modules/fuser/HeadingAngleFuser.cpp
    src/modules/fuser/StateFuser.cpp
    src/modules/manager/ObjectRemover.cpp
    src/modules/manager/ObjectCreator.cpp
    src/modules/manager/MeasurementHistorian.cpp
    src/modules/matcher/Matcher.cpp
    src/modules/matcher/algorithms/Auction.cpp
    src/modules/matcher/distance_measures/Mahalanobis.cpp
    src/modules/matcher/distance_measures/IntersectionOverUnion.cpp
    src/modules/predictor/EgoMotionCompensation.cpp
    src/modules/predictor/StatePredictor.cpp
    src/modules/predictor/ExistencePredictor.cpp
    src/utility/FusionUtility.cpp)

# set(CMAKE_BUILD_TYPE Release)

add_dependencies(object_fusion ${catkin_EXPORTED_TARGETS})

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(object_fusion PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

# If we have compiler requirements for this library, list them
# here
target_compile_features(object_fusion
    PUBLIC cxx_auto_type cxx_std_14
    PRIVATE cxx_variadic_templates)

# Depend on a library that we defined in the top-level file
target_link_libraries(object_fusion
    Boost::filesystem
    ${catkin_LIBRARIES})

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS object_fusion EXPORT fusionConfig
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Mark header files for installation
install(FILES
    include/ikaFusLib.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT fusionConfig DESTINATION cmake)

# This makes the project importable from the build directory
export(TARGETS object_fusion FILE fusionConfig.cmake)

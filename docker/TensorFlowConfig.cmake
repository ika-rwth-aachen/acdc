# find dependencies
find_package(Protobuf REQUIRED)

# find include directories
find_path(INCLUDE_DIR tensorflow/core/public/session.h PATH_SUFFIXES tensorflow)
list(APPEND INCLUDE_DIRS ${INCLUDE_DIR})
if(INCLUDE_DIR)
    list(APPEND INCLUDE_DIRS ${INCLUDE_DIR}/src)
endif()

# find libraries
find_library(LIBRARY libtensorflow_cc.so PATH_SUFFIXES tensorflow)
find_library(LIBRARY_FRAMEWORK libtensorflow_framework.so PATH_SUFFIXES tensorflow)

# handle the QUIETLY and REQUIRED arguments and set *_FOUND
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TensorFlow DEFAULT_MSG INCLUDE_DIRS LIBRARY)
mark_as_advanced(INCLUDE_DIRS LIBRARY LIBRARY_FRAMEWORK)

# set INCLUDE_DIRS and LIBRARIES
if(TensorFlow_FOUND)
    set(TensorFlow_INCLUDE_DIRS ${INCLUDE_DIRS})
    if(LIBRARY_FRAMEWORK)
        set(TensorFlow_LIBRARIES ${LIBRARY} ${LIBRARY_FRAMEWORK} ${Protobuf_LIBRARY})
    else()
        set(TensorFlow_LIBRARIES ${LIBRARY} ${Protobuf_LIBRARY})
    endif()
endif()
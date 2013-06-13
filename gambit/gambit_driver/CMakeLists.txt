cmake_minimum_required(VERSION 2.4.6 FATAL_ERROR)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

option(REALTIME "Enable Xenomai realtime support for arm control loop")
option(DUMMY_WRIST "Allows arm to run without wrist attached")

rosbuild_init()

find_library(LOG4CXX_LIBRARY log4cxx)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if(DUMMY_WRIST)
    message(STATUS "Building with fake wrist")
    add_definitions("-DDUMMY_WRIST")
endif()

if(REALTIME)
    message(STATUS "Building with realtime support")
    add_definitions("-DREALTIME -D__XENO__ -pipe -Wall -D_REENTRANT -D_GNU_SOURCE")
    include_directories("/usr/xenomai/include")
    link_directories("/usr/xenomai/lib")
endif()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} 
    src/arm.cpp 
    src/actuator.cpp 
    src/actuator_copley.cpp
    src/actuator_group.cpp
    src/actuator_dummy.cpp
    src/actuator_RX28.cpp 
    src/DOF.cpp 
    src/smoothed_DOF.cpp
    src/robot.cpp
    src/wrist.cpp 
)

if(REALTIME)
    target_link_libraries(${PROJECT_NAME} xenomai native)
endif()

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(wrist src/wrist_node.cpp)
target_link_libraries(wrist ${PROJECT_NAME} ${LOG4CXX_LIBRARY})

rosbuild_add_executable(arm src/arm_node.cpp)
target_link_libraries(arm ${PROJECT_NAME} ${LOG4CXX_LIBRARY})

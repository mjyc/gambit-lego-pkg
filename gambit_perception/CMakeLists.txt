cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# For debug
set(CMAKE_CXX_FLAGS "-g -Wall")

rosbuild_add_boost_directories()

# Builing nodelets
#rosbuild_add_library(gambit_perception src/convert_pointcloud_imgdep_nodelet.cpp)

# Building other executables
# Tools
#rosbuild_add_executable(interactive_mask src/interactive_mask.cpp)
#rosbuild_link_boost(interactive_mask signals system)
#rosbuild_add_executable(learn_basetable src/learn_basetable.cpp)
#rosbuild_link_boost(learn_basetable signals system)
#rosbuild_add_executable(interactive_area src/interactive_area.cpp)
#rosbuild_link_boost(interactive_area signals filesystem system)
#rosbuild_add_executable(capture_iamges src/capture_images.cpp)
#rosbuild_link_boost(capture_iamges signals filesystem system)

# Tests
#rosbuild_add_executable(cv_utils_test src/cv_utils_test.cpp)
#rosbuild_link_boost(cv_utils_test filesystem system)

# Main Programs
#rosbuild_add_executable(event_main src/event_main.cpp)
#rosbuild_link_boost(event_main signals system)
#rosbuild_add_executable(event_main_icra2013 src/event_main_icra2013.cpp)
#rosbuild_link_boost(event_main_icra2013 signals filesystem system)
rosbuild_add_executable(event_main_simple_blocksworld src/event_main_simple_blocksworld.cpp)
rosbuild_link_boost(event_main_simple_blocksworld signals filesystem system)



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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/packet.cpp 
    src/bus/interface_tty.cpp src/actuator.cpp)
#target_link_libraries(${PROJECT_NAME} ftd2xx)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(test_packet test/packet_test.cpp)
target_link_libraries(test_packet ${PROJECT_NAME})

rosbuild_add_executable(ping util/ping.cpp)
target_link_libraries(ping ${PROJECT_NAME})
rosbuild_add_executable(write util/write.cpp)
target_link_libraries(write ${PROJECT_NAME})
rosbuild_add_executable(read util/read.cpp)
target_link_libraries(read ${PROJECT_NAME})
rosbuild_add_executable(posdump util/posdump.cpp)
target_link_libraries(posdump ${PROJECT_NAME})
rosbuild_add_executable(ping2 util/ping2.cpp)
target_link_libraries(ping2 ${PROJECT_NAME})

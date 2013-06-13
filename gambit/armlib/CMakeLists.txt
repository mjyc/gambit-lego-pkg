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

include_directories(ik)
 
#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} 
    src/arm.cpp 

    src/hybrid.cpp
    src/gambit.cpp
    
    src/js_linear_trajectory.cpp
    src/recorded_trajectory.cpp

    src/geom.cpp
)

# [2012.08.24]
# Below codes need to be modified to compile w/o WAM code.
# rosbuild_add_executable(play_traj tests/play_traj.cpp)
# target_link_libraries(play_traj ${PROJECT_NAME})

# rosbuild_add_executable(test_ik tests/ik.cpp)
# target_link_libraries(test_ik ${PROJECT_NAME})

rosbuild_add_executable(go_to tests/go_to.cpp)
target_link_libraries(go_to ${PROJECT_NAME})

# rosbuild_add_executable(clockwise tests/clockwise.cpp)
# target_link_libraries(clockwise ${PROJECT_NAME})


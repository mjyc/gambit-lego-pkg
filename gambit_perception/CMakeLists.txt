cmake_minimum_required(VERSION 2.8.3)
project(gambit_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_geometry
  image_transport
  message_filters
  cv_bridge
  sensor_msgs
  pcl_conversions
  pcl_ros
  nodelet
  tf
  visualization_msgs
)

find_package(OpenCV REQUIRED core imgproc highgui)
# find_package(Eigen REQUIRED)
# include_directories(${EIGEN_INCLUDE_DIRS})
# add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES
    CATKIN_DEPENDS roscpp image_geometry image_transport message_filters cv_bridge sensor_msgs pcl_conversions pcl_ros nodelet tf visualization_msgs
    # DEPENDS
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${EIGEN_INCLUDE_DIRS}
)

add_executable(interactive_mask src/interactive_mask.cpp)
target_link_libraries(interactive_mask
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(learn_basetable src/learn_basetable.cpp)
target_link_libraries(learn_basetable
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(interactive_area src/interactive_area.cpp)
target_link_libraries(interactive_area
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(event_main src/event_main.cpp)
add_dependencies(event_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(event_main
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(event_main_simple_blocksworld src/event_main_simple_blocksworld.cpp)
add_dependencies(event_main_simple_blocksworld ${catkin_EXPORTED_TARGETS})
target_link_libraries(event_main_simple_blocksworld
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

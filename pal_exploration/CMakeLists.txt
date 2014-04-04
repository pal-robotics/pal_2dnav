cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/../catkin/cmake/debug_message.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

find_package(PCL REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

rosbuild_genmsg()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# Header files are included so that they'll show up in QtCreator
rosbuild_add_library(pal_exploration src/base_strategy.cpp src/frontiers_strategy.cpp include/${PROJECT_NAME}/base_strategy.h include/${PROJECT_NAME}/frontiers_strategy.h src/grid.cpp include/${PROJECT_NAME}/grid.h src/grid_map.cpp include/${PROJECT_NAME}/grid_map.h include/${PROJECT_NAME}/visualization.h include/${PROJECT_NAME}/frontiers.h include/${PROJECT_NAME}/algorithms.h src/visualization.cpp src/algorithms.cpp src/markers.cpp include/${PROJECT_NAME}/markers.h src/frontiers.cpp)

rosbuild_add_executable(exploration_server src/exploration_server.cpp)
target_link_libraries(exploration_server ${PROJECT_NAME})

cmake_minimum_required(VERSION 2.8.3)
project(pal_nav_utils)

set(ROS_DEPS actionlib geometry_msgs pcl_conversions roscpp sensor_msgs std_msgs tf visualization_msgs)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPS})
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${ROS_DEPS}
)

# Header files are included so that they'll show up in QtCreator
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/grid.h
  include/${PROJECT_NAME}/grid_map.h
  include/${PROJECT_NAME}/visualization.h
  include/${PROJECT_NAME}/algorithms.h
  include/${PROJECT_NAME}/conversions.h
  src/markers.cpp include/${PROJECT_NAME}/markers.h
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(pal_exploration)

set(ROS_DEPS actionlib dynamic_reconfigure geometry_msgs move_base_msgs pcl_conversions roscpp sensor_msgs std_msgs tf visualization_msgs)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPS})
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_action_files(DIRECTORY action FILES Exploration.action)
generate_messages(DEPENDENCIES actionlib_msgs)

generate_dynamic_reconfigure_options(cfg/PalExploration.cfg)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package()

# Header files are included so that they'll show up in QtCreator
add_library(${PROJECT_NAME} src/base_strategy.cpp src/frontiers_strategy.cpp include/${PROJECT_NAME}/base_strategy.h include/${PROJECT_NAME}/frontiers_strategy.h src/grid.cpp include/${PROJECT_NAME}/grid.h src/grid_map.cpp include/${PROJECT_NAME}/grid_map.h include/${PROJECT_NAME}/visualization.h include/${PROJECT_NAME}/frontiers.h include/${PROJECT_NAME}/algorithms.h src/visualization.cpp src/algorithms.cpp src/markers.cpp include/${PROJECT_NAME}/markers.h src/frontiers.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(exploration_server src/exploration_server.cpp)
add_dependencies(exploration_server ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(exploration_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})

install(TARGETS exploration_server
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

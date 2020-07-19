cmake_minimum_required(VERSION 2.8.3)
project(rdl_navigation_layers)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    costmap_2d
    dynamic_reconfigure
)

include_directories(${catkin_INCLUDE_DIRS} include/rdl_navigation_layers)

catkin_package()

add_library(rdl_obstacles_layer src/rdl_obstacles_layer.cpp)
target_link_libraries(rdl_obstacles_layer ${catkin_LIBRARIES})

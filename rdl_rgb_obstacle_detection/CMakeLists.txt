cmake_minimum_required(VERSION 2.8.3)
project(rdl_rgb_obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  message_generation
  dynamic_reconfigure
)

add_message_files(FILES Object.msg Obstacles.msg Goals.msg)

generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/image_processing.cfg
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs std_msgs message_runtime
)

# make sure configure headers are built before any node using them
add_custom_target(dummy_target)
add_dependencies(dummy_target ${PROJECT_NAME}_gencfg)

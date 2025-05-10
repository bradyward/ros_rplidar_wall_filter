cmake_minimum_required(VERSION 3.10)
project(ros_rplidar_wall_filter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kurome REQUIRED)

add_executable(wall_filter_node src/wall_filter_node.cpp)
ament_target_dependencies(wall_filter_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  kurome
)

install(TARGETS
  wall_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


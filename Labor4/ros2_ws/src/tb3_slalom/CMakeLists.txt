cmake_minimum_required(VERSION 3.5)
project(tb3_slalom)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(tb3_slalom_node src/tb3_slalom.cpp)
ament_target_dependencies(tb3_slalom_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)

install(TARGETS
  tb3_slalom_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(obstacles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(service_interface REQUIRED)
find_package(obstacles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InflateObstacles.srv"
)

add_executable(test_node src/test_service.cpp)
ament_target_dependencies(test_node 
	rclcpp
  service_interface
  obstacles
)

add_executable(test_client src/test_client.cpp)
ament_target_dependencies(test_client 
	rclcpp
  service_interface
  obstacles
)

add_executable(inflate_obstacles_node src/inflate_obstacles_node.cpp)
ament_target_dependencies(inflate_obstacles_node 
	rclcpp
  service_interface
  obstacles
  geometry_msgs
  obstacles_msgs
)

add_executable(clipper src/clipper_test.cpp)
ament_target_dependencies(clipper 
	rclcpp
  service_interface
  obstacles
  geometry_msgs
)

install(TARGETS
  test_node
  test_client
  inflate_obstacles_node
  clipper
  DESTINATION lib/${PROJECT_NAME})

ament_package()

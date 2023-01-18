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
# find_package(service_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)

# add_executable(test_node src/test_service.cpp)
# ament_target_dependencies(test_node 
# 	rclcpp
#   service_interface
# )

# add_executable(test_client src/test_client.cpp)
# ament_target_dependencies(test_client 
# 	rclcpp
#   service_interface
# )

add_executable(inflate_obstacles src/inflate_obstacles.cpp)
ament_target_dependencies(inflate_obstacles
	rclcpp
  geometry_msgs
  obstacles_msgs
)

# add_executable(clipper src/clipper_test.cpp)
# ament_target_dependencies(clipper 
# 	rclcpp
#   geometry_msgs
# )

install(TARGETS
  inflate_obstacles
  DESTINATION lib/${PROJECT_NAME})

ament_package()

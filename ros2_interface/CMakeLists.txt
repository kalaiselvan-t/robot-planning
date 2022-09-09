cmake_minimum_required(VERSION 3.8)
project(ros2_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(lidar_node src/shelfino_lidar_publisher.cpp)
ament_target_dependencies(lidar_node rclcpp std_msgs)

install(TARGETS
  lidar_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
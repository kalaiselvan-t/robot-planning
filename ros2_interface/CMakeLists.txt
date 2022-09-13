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

include_directories(
  include
)

add_executable(lidar_node 
  src/shelfino_lidar_publisher.cpp
  src/Subscriber.cpp
)
ament_target_dependencies(lidar_node rclcpp std_msgs sensor_msgs)

target_link_libraries(pose_pub_node
  ${rclcpp_LIBRARIES} -lzmq 
)

install(TARGETS
  lidar_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
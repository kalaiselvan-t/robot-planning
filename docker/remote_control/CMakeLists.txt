cmake_minimum_required(VERSION 3.8)
project(remote_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-w)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(remote_control_node 
  src/remote_control.cpp
)
ament_target_dependencies(remote_control_node 
  rclcpp
  geometry_msgs
)

target_link_libraries(remote_control_node
  ${rclcpp_LIBRARIES} -lncurses -pthread
)

install(TARGETS
  remote_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

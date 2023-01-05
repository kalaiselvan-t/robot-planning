cmake_minimum_required(VERSION 3.8)
project(roadmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(roadmap src/roadmap.cpp src/r_map.cpp)

install(
  TARGETS roadmap
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

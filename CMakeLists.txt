cmake_minimum_required(VERSION 3.10)
project(twist_limiter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(twist_limiter_node src/main.cpp)
ament_target_dependencies(twist_limiter_node
                          geometry_msgs
                          rclcpp)

install(TARGETS
    twist_limiter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

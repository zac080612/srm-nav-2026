cmake_minimum_required(VERSION 3.8)
project(nav_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_core REQUIRED)

add_executable(figure8_node src/figure8_node.cpp)
target_link_libraries(figure8_node nav::nav_core)
ament_target_dependencies(figure8_node rclcpp nav_msgs std_msgs visualization_msgs)

install(TARGETS figure8_node DESTINATION lib/${PROJECT_NAME})

ament_package()

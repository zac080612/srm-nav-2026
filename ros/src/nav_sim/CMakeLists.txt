cmake_minimum_required(VERSION 3.8)
project(nav_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_core REQUIRED)

add_executable(fake_mcu_node src/fake_mcu_node.cpp)
target_link_libraries(fake_mcu_node nav::nav_core)
ament_target_dependencies(fake_mcu_node rclcpp std_msgs geometry_msgs)

add_executable(pile_detector_node src/pile_detector_node.cpp)
target_link_libraries(pile_detector_node nav::nav_core)
ament_target_dependencies(pile_detector_node rclcpp sensor_msgs visualization_msgs)

install(TARGETS fake_mcu_node pile_detector_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY urdf world launch config DESTINATION share/${PROJECT_NAME})

ament_package()

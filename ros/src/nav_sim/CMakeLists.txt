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

# 实车感知：订阅 Livox 点云 → 发布 /piles（桩位置）+ /scan（避障用 2D 激光）
add_executable(perception_node src/pile_detector_cloud_node.cpp)
target_link_libraries(perception_node nav::nav_core)
ament_target_dependencies(perception_node rclcpp sensor_msgs visualization_msgs)

# 实车标定：发速度脉冲 + 点云质心差，反解 R_bc 轴映射（一次性工具，不参与正式跑车）
add_executable(calib_node src/calib_node.cpp)
target_link_libraries(calib_node nav::nav_core)
ament_target_dependencies(calib_node rclcpp std_msgs sensor_msgs)

install(TARGETS fake_mcu_node pile_detector_node perception_node calib_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY urdf world launch config DESTINATION share/${PROJECT_NAME})

ament_package()

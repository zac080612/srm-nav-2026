cmake_minimum_required(VERSION 3.8)
project(nav_protocol)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_core REQUIRED)

# 串口通信节点：真实串口后端（19 字节帧写串口发给下位机，读反馈解析比赛阶段）
# 已启用（上机用）。仿真 launch 不会启动它，所以保留编译不影响仿真。
add_executable(serial_node src/serial_node.cpp)
target_link_libraries(serial_node nav::nav_core)
ament_target_dependencies(serial_node rclcpp std_msgs)
install(TARGETS serial_node DESTINATION lib/${PROJECT_NAME})

ament_package()

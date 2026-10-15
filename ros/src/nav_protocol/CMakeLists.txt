cmake_minimum_required(VERSION 3.8)
project(nav_protocol)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_core REQUIRED)

# ⚠️⚠️⚠️ 串口通信节点：默认【注释掉】，不参与编译 ⚠️⚠️⚠️
# 原因：模拟阶段走 fake_mcu（/raw_frame → /cmd_vel → Gazebo），
#       若编译并启动串口节点，会因串口打不开（/dev/ttyACM0 不存在）而报错。
# 上车时：取消下面三行注释，并重新 bash build.sh 即可编译。
#
# add_executable(serial_node src/serial_node.cpp)
# target_link_libraries(serial_node nav::nav_core)
# ament_target_dependencies(serial_node rclcpp std_msgs)
# install(TARGETS serial_node DESTINATION lib/${PROJECT_NAME})

ament_package()

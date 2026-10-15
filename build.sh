#!/usr/bin/env bash
# 一键构建：先装 core 纯 C++ 库 → install-core/，再 colcon 构建 ROS 包
# 用法：bash build.sh
set -e
cd "$(dirname "$0")"

# 1. 构建并安装核心库（协议/控制/状态机，纯 C++，含单元测试）
cmake -S core -B build-core -DCMAKE_INSTALL_PREFIX="$PWD/install-core"
cmake --build build-core -j"$(nproc)"
cmake --install build-core

# 2. 构建 ROS 包（figure8 控制节点 + fake_mcu 模拟）
cd ros
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_PREFIX_PATH="$PWD/../install-core"

echo
echo "构建完成。运行：bash run_sim.sh"

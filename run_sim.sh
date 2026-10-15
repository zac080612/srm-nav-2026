#!/usr/bin/env bash
# 一键启动模拟：Gazebo + 全向车 + fake_mcu + figure8 控制节点
# 用法：bash run_sim.sh
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
source ros/install/setup.bash

ros2 launch nav_sim figure8.launch.py

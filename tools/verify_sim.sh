#!/usr/bin/env bash
# headless 端到端验证：不弹 GUI，跑一段时间，
# 检查桩检测数量、状态机推进、协议坏帧。
# 结果写到项目根目录 verify_result.txt。
# 用法：bash tools/verify_sim.sh && cat verify_result.txt
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source ros/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export ROS_DOMAIN_ID=42
export GAZEBO_MASTER_URI=http://127.0.0.1:11399

ros2 launch nav_sim figure8.launch.py gui:=false rviz:=false >/tmp/launch.log 2>&1 &
LPID=$!
sleep 200

{
  echo "=== 桩检测数量 ==="
  grep -E "检测到" /tmp/launch.log | tail -5
  echo "=== 状态机推进 ==="
  grep -oE "state=[A-Za-z]+" /tmp/launch.log | sort | uniq -c
  echo "=== 协议坏帧 ==="
  grep -E "坏帧" /tmp/launch.log | tail -3
} > verify_result.txt 2>&1

kill $LPID 2>/dev/null
pkill -f gzserver 2>/dev/null
pkill -f gzclient 2>/dev/null
pkill -f figure8_node 2>/dev/null
pkill -f fake_mcu_node 2>/dev/null
pkill -f pile_detector 2>/dev/null

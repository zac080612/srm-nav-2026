#!/usr/bin/env bash
# 临时：启动模拟 40 秒，打印 /piles 一次看检测到的是哪个桩
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source ros/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1 ROS_DOMAIN_ID=42
export GAZEBO_MASTER_URI=http://127.0.0.1:11399
pkill -f gzserver 2>/dev/null || true
pkill -f figure8_node 2>/dev/null || true
sleep 1
ros2 launch nav_sim figure8.launch.py gui:=false rviz:=false > /tmp/piles_test.log 2>&1 &
LPID=$!
sleep 40
echo "=== /piles ==="
timeout 3 ros2 topic echo /piles --no-daemon --once 2>/dev/null | grep -E "x:|y:" | head -10
kill $LPID 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f figure8_node 2>/dev/null || true
pkill -f fake_mcu_node 2>/dev/null || true
pkill -f pile_detector 2>/dev/null || true

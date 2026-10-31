#!/usr/bin/env bash
# 临时诊断：跑 90 秒模拟，抓避障日志
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source ros/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1 ROS_DOMAIN_ID=42
export GAZEBO_MASTER_URI=http://127.0.0.1:11399
pkill -f gzserver 2>/dev/null || true
pkill -f figure8_node 2>/dev/null || true
sleep 1
ros2 launch nav_sim figure8.launch.py gui:=false rviz:=false > /tmp/launch2.log 2>&1 &
LPID=$!
echo "launch pid=$LPID"
sleep 90
kill $LPID 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f figure8_node 2>/dev/null || true
sleep 2
echo "=== 避障日志 ==="
grep -E "避障" /tmp/launch2.log | tail -40
echo "=== 状态统计 ==="
grep -oE "state=[A-Za-z]+" /tmp/launch2.log | sort | uniq -c

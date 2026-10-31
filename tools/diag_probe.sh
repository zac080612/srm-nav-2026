#!/usr/bin/env bash
# 临时：启动模拟 + probe 5 层激光
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source ros/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1 ROS_DOMAIN_ID=42
export GAZEBO_MASTER_URI=http://127.0.0.1:11399
pkill -f gzserver 2>/dev/null || true
pkill -f figure8_node 2>/dev/null || true
sleep 1
ros2 launch nav_sim figure8.launch.py gui:=false rviz:=false > /tmp/probe.log 2>&1 &
LPID=$!
sleep 25
timeout 12 python3 tools/probe_scans.py 2>/dev/null | tail -6
kill $LPID 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f figure8_node 2>/dev/null || true
pkill -f fake_mcu_node 2>/dev/null || true
pkill -f pile_detector 2>/dev/null || true
pkill -f probe_scans 2>/dev/null || true
echo PROBE_DONE

#!/usr/bin/env bash
# 障碍避障测试：headless 跑仿真，输出桩检测/状态机/锁定/坏帧汇总
# 用法：bash tools/test_obstacle.sh [秒数，默认 180]
cd "$(dirname "$0")/.."
DUR="${1:-180}"
source /opt/ros/humble/setup.bash
source ros/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export ROS_DOMAIN_ID=42
export GAZEBO_MASTER_URI=http://127.0.0.1:11399

pkill -f gzserver 2>/dev/null
pkill -f gzclient 2>/dev/null
pkill -f figure8_node 2>/dev/null
sleep 2

ros2 launch nav_sim figure8.launch.py gui:=false rviz:=false >/tmp/obstacle_test.log 2>&1 &
LPID=$!
sleep "$DUR"

{
  echo "=== 桩检测（最近 6 条）==="
  grep -E "检测到" /tmp/obstacle_test.log | tail -6
  echo "=== 状态机推进 ==="
  grep -oE "state=[A-Za-z]+" /tmp/obstacle_test.log | sort | uniq -c
  echo "=== 桩锁定 ==="
  grep -E "已锁定" /tmp/obstacle_test.log | head -2
  echo "=== 坏帧 ==="
  grep -E "坏帧" /tmp/obstacle_test.log | tail -3
  echo "=== 车位置尾部（看是否冲出场地）==="
  grep -E "body=" /tmp/obstacle_test.log | tail -3
  echo "=== 日志尾部 ==="
  tail -5 /tmp/obstacle_test.log
} > obstacle_result.txt 2>&1

kill $LPID 2>/dev/null
pkill -f gzserver 2>/dev/null
pkill -f gzclient 2>/dev/null
pkill -f figure8_node 2>/dev/null
pkill -f fake_mcu_node 2>/dev/null
pkill -f pile_detector 2>/dev/null
echo "结果已写入 obstacle_result.txt"

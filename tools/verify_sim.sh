#!/usr/bin/env bash
# headless 端到端验证：gui:=false 启动整套模拟，跑约 55 秒，
# 检查协议闭环（坏帧数）、车是否移动（odom）、状态机是否推进。
# 结果写到项目根目录 verify_result.txt。
# 用法：bash tools/verify_sim.sh && cat verify_result.txt
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source ros/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch nav_sim figure8.launch.py gui:=false >/tmp/launch.log 2>&1 &
LPID=$!
sleep 55

{
  echo "=== odom raw (once) ==="
  timeout 5 ros2 topic echo /odom --once 2>&1 | head -22
  echo "=== topics ==="
  ros2 topic list 2>&1 | grep -E "odom|cmd_vel|raw_frame"
  echo "=== figure8 状态推进日志 ==="
  grep -E "state=" /tmp/launch.log | tail -12
  echo "=== fake_mcu 坏帧统计 ==="
  grep -E "坏帧" /tmp/launch.log | tail -3
} > verify_result.txt 2>&1

kill $LPID 2>/dev/null
pkill -f gzserver 2>/dev/null
pkill -f gzclient 2>/dev/null
pkill -f figure8_node 2>/dev/null
pkill -f fake_mcu_node 2>/dev/null

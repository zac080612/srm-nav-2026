#!/usr/bin/env bash
# 一键标定 R_bc 和 speed_scale：清进程 → 找串口 → 起雷达 → 起串口 → 跑标定
# 用法：bash scripts/calibrate.sh [串口设备] [v_test] [pulse_ms] [tf_yaw]
#   串口设备留空 = 自动用 /dev/serial/by-id/ 下第一个
#   v_test 默认 1.0（满杆≈2m/s 时约 0.5m/s）；pulse_ms 默认 800（与 1.2 时相同）
#   tf_yaw 默认 -1.5708（-90°：雷达相对车体安装角，按肉眼实测方向与点云方向的差确定）
set -u
DEV="${1:-}"
V_TEST="${2:-1.0}"
PULSE_MS="${3:-800}"
TF_YAW="${4:--1.5708}"

REPO="$(cd "$(dirname "$0")/.." && pwd)"
LOG=/tmp/srm_calib
mkdir -p "$LOG"

# 环境全部在脚本内完成，不依赖终端手动 source
export ROS_LOCALHOST_ONLY=1
set +u    # ROS 官方 setup.bash 不兼容 nounset，source 期间临时关闭
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$HOME/ws_livox/install/setup.bash" 2>/dev/null || true
# shellcheck disable=SC1091
source "$REPO/ros/install/setup.bash"
set -u

cleanup() {
  pkill -f calib_node 2>/dev/null
  pkill -f serial_node 2>/dev/null
  pkill -f livox_ros_driver2_node 2>/dev/null
  sleep 1
}
trap cleanup EXIT INT TERM

say() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

say "清理旧进程..."
cleanup

# ---- 1. 找串口 ----
if [ -z "$DEV" ]; then
  FIRST=$(ls /dev/serial/by-id/ 2>/dev/null | head -1)
  [ -n "$FIRST" ] && DEV="/dev/serial/by-id/$FIRST"
fi
if [ -z "$DEV" ] || [ ! -e "$DEV" ]; then
  echo "❌ 找不到串口设备（检查 USB 线、下位机电源）"
  echo "   当前 /dev/serial/by-id/ 下："
  ls /dev/serial/by-id/ 2>/dev/null | sed 's/^/     /' || echo "     （空）"
  exit 1
fi
say "串口：$DEV"

# ---- 2. 雷达（没起才起）----
if ! ros2 topic list --no-daemon 2>/dev/null | grep -qx /livox/lidar; then
  say "雷达没起，自动启动..."
  ros2 launch "$HOME/ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py" \
    > "$LOG/livox.log" 2>&1 &
  for i in $(seq 1 30); do
    sleep 1
    ros2 topic list --no-daemon 2>/dev/null | grep -qx /livox/lidar && break
  done
fi
if ! ros2 topic list --no-daemon 2>/dev/null | grep -qx /livox/lidar; then
  echo "❌ 雷达没起来，日志尾部："
  tail -15 "$LOG/livox.log" 2>/dev/null | sed 's/^/   /'
  exit 1
fi
say "点云 /livox/lidar OK"

# ---- 3. 串口后端 ----
ros2 run nav_protocol serial_node --ros-args -p device:="$DEV" \
  > "$LOG/serial.log" 2>&1 &
sleep 2
if ! grep -q "已打开" "$LOG/serial.log"; then
  echo "❌ 串口没打开，日志尾部："
  tail -8 "$LOG/serial.log" | sed 's/^/   /'
  exit 1
fi
say "串口后端 OK"

# ---- 4. 标定（前台跑，结果直接打印）----
echo
echo "=============================================="
echo " 标定开始：车会动 3 轮"
echo " 要求：车落地、场地留 30cm、手放急停旁边"
echo " v_test=$V_TEST pulse_ms=$PULSE_MS tf_yaw=$TF_YAW"
echo "=============================================="
ros2 run nav_sim calib_node --ros-args \
  -p v_test:="$V_TEST" -p pulse_ms:="$PULSE_MS" -p tf_z:=0.20 -p tf_yaw:="$TF_YAW"

say "标定结束，结果见上方输出。"

#!/usr/bin/env bash
# 比赛一键启动：雷达 → 定位 → 主程序。纯实车，不启动任何仿真组件
#   （不含 Gazebo / fake_mcu / pile_detector_node / RViz）
# 用法：bash scripts/run_race.sh [串口设备]
#   串口默认 /dev/ttyACM0，建议传 /dev/serial/by-id/... 的稳定路径
# 退出（Ctrl+C / SSH 断开）时会先让下位机收到零速帧，再关串口

set -u
DEV="${1:-/dev/ttyACM0}"
AUTO_START="${AUTO_START:-false}"
LOG_DIR="${LOG_DIR:-/tmp/srm_race}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

REPO="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$LOG_DIR"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$HOME/ws_livox/install/setup.bash" 2>/dev/null || true
source "$HOME/ws_lio/install/setup.bash" 2>/dev/null || true
# shellcheck disable=SC1091
source "$REPO/ros/install/setup.bash"

say() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

# 安全停止：先停控制节点，等 serial_node 看门狗把零速帧发满 500ms，再关串口。
# 顺序不能反 —— 协议没有超时保护，下位机会一直执行最后收到的指令。
stop_all() {
  pkill -f figure8_node 2>/dev/null
  sleep 2
  pkill -f serial_node 2>/dev/null
  pkill -f perception_node 2>/dev/null
  pkill -f point_lio 2>/dev/null
  pkill -f livox_ros_driver2_node 2>/dev/null
  sleep 1
}
trap stop_all EXIT INT TERM HUP

wait_topic() {
  local topic="$1" secs="$2" i
  for i in $(seq 1 "$secs"); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then return 0; fi
    sleep 1
  done
  return 1
}

# Point-LIO 各安装版本 launch 名字不同，优先选含 mid360 的
find_lio_launch() {
  local prefix
  prefix=$(ros2 pkg prefix point_lio 2>/dev/null) || return 1
  ls "$prefix"/share/point_lio/launch/*.launch.py 2>/dev/null | grep -i mid360 | head -1 \
    || ls "$prefix"/share/point_lio/launch/*.launch.py 2>/dev/null | head -1
}

echo "=============================================="
echo " SRM 导航赛道 比赛启动"
echo " 串口    : $DEV"
echo " 开始方式: $([ "$AUTO_START" = true ] && echo '立即开始（调试用）' || echo '等 game_progress（比赛用）')"
echo " 日志目录: $LOG_DIR"
echo "=============================================="

say "清理旧进程..."
stop_all

if [ ! -e "$DEV" ]; then
  echo "❌ 串口设备不存在：$DEV"
  echo "   当前可用："
  ls /dev/serial/by-id/ 2>/dev/null | sed 's/^/     /' || echo "     （没有 /dev/serial/by-id 设备）"
  exit 1
fi
say "串口 OK"

LIO_LAUNCH="$(find_lio_launch)"
if [ -z "$LIO_LAUNCH" ]; then
  echo "❌ 找不到 Point-LIO 的 launch，/odom 会没有数据"
  echo "   ros2 pkg prefix point_lio → $(ros2 pkg prefix point_lio 2>&1)"
  exit 1
fi
say "定位 launch：$(basename "$LIO_LAUNCH")"

say "[1/3] 启动雷达..."
ros2 launch "$HOME/ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py" \
  > "$LOG_DIR/livox.log" 2>&1 &
if ! wait_topic /livox/lidar 25; then
  echo "❌ 雷达没起来，日志尾部："
  tail -15 "$LOG_DIR/livox.log" | sed 's/^/   /'
  exit 1
fi
say "      点云话题 OK"

say "[2/3] 启动定位..."
ros2 launch "$LIO_LAUNCH" > "$LOG_DIR/lio.log" 2>&1 &
if ! wait_topic /odom 30; then
  say "      ⚠️ /odom 还没出来（Point-LIO 常要车动一下才有输出），先继续"
fi

say "[3/3] 启动主程序..."
ros2 launch nav_sim figure8_real.launch.py \
  device:="$DEV" auto_start:="$AUTO_START" > "$LOG_DIR/main.log" 2>&1 &

sleep 6
for t in /piles /scan /raw_frame; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then
    say "      $t OK"
  else
    say "      ⚠️ $t 没有，检查 $LOG_DIR/main.log"
  fi
done

echo
echo "=============================================="
echo " 已启动。Ctrl+C 停止（会先发零速帧再关串口）"
echo " 实时日志: tail -f $LOG_DIR/main.log"
echo "=============================================="

tail -f "$LOG_DIR/main.log"

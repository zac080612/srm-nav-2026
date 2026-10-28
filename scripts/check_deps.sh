#!/usr/bin/env bash
# 依赖自检：逐项检查实车运行需要的系统包、编译命令、第三方库和两个外部工作区
# 用法：bash scripts/check_deps.sh

ok=0
miss=0
todo=()

ok_()   { printf '  [OK]   %s\n' "$1"; ok=$((ok + 1)); }
miss_() { printf '  [缺]   %s\n' "$1"; miss=$((miss + 1)); todo+=("$2"); }

echo "========== SRM 导航 实车依赖检测 =========="

echo
echo "[1] 系统包"
for p in build-essential cmake git python3-pip python3-colcon-common-extensions libpcl-dev libeigen3-dev; do
  if dpkg -s "$p" >/dev/null 2>&1; then
    ok_ "$p"
  else
    miss_ "$p" "sudo apt install $p"
  fi
done

echo
echo "[2] 编译命令"
for c in gcc g++ make cmake git colcon; do
  if command -v "$c" >/dev/null 2>&1; then
    ok_ "$c  ($(command -v "$c"))"
  else
    miss_ "$c" "缺命令 $c"
  fi
done

echo
echo "[3] ROS 2"
if [ -f /opt/ros/humble/setup.bash ]; then
  ok_ "/opt/ros/humble"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  miss_ "ROS 2 Humble" "按官方文档安装 ROS 2 Humble"
fi

echo
echo "[4] 第三方库头文件"
if ls -d /usr/include/pcl-* >/dev/null 2>&1; then
  ok_ "PCL 头文件"
else
  miss_ "PCL 头文件" "sudo apt install libpcl-dev"
fi
if [ -d /usr/include/eigen3 ]; then
  ok_ "Eigen 头文件"
else
  miss_ "Eigen 头文件" "sudo apt install libeigen3-dev"
fi

echo
echo "[5] 第三方工作区"
for s in ~/ws_livox/install/setup.bash \
         ~/ws_livox/src/livox_ros_driver2/install/setup.bash \
         ~/ws_lio/install/setup.bash; do
  # shellcheck disable=SC1090
  [ -f "$s" ] && source "$s" 2>/dev/null
done

if ros2 pkg list 2>/dev/null | grep -qx livox_ros_driver2; then
  ok_ "livox_ros_driver2（雷达驱动）"
else
  miss_ "livox_ros_driver2（雷达驱动）" "手册 4.2：clone 后 ./build.sh humble"
fi

if ros2 pkg list 2>/dev/null | grep -qEi 'point_lio|pointlio|fast_lio'; then
  ok_ "Point-LIO（定位）"
else
  miss_ "Point-LIO（定位）" "手册 4.2：clone 后 colcon build"
fi

echo
echo "[6] 本项目编译产物"
for b in ros/install/nav_control/lib/nav_control/figure8_node \
         ros/install/nav_sim/lib/nav_sim/perception_node \
         ros/install/nav_sim/lib/nav_sim/calib_node \
         ros/install/nav_protocol/lib/nav_protocol/serial_node; do
  if [ -x "$b" ]; then
    ok_ "$(basename "$b")"
  else
    miss_ "$(basename "$b")" "bash build.sh"
  fi
done

echo
echo "========== 结论 =========="
printf '通过 %d 项，缺 %d 项\n' "$ok" "$miss"
if [ "$miss" -eq 0 ]; then
  echo "依赖齐全，可以编译运行。"
else
  echo "按下面的命令补齐（已去重）："
  printf '  %s\n' "${todo[@]}" | sort -u
fi

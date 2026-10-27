#!/usr/bin/env bash
# 上车前自检：不启动任何节点，只检查环境是否就绪
# 用法：bash scripts/preflight.sh
# 退出码 0 = 可以上车（允许 WARN）；1 = 有 FAIL 必须先解决

cd "$(dirname "$0")/.." || exit 1

ok=0
warn=0
fail=0
pass()  { printf '  [OK]   %s\n' "$1"; ok=$((ok + 1)); }
warnf() { printf '  [WARN] %s\n' "$1"; warn=$((warn + 1)); }
failf() { printf '  [FAIL] %s\n' "$1"; fail=$((fail + 1)); }

echo "=== 1. 系统 ==="
if [ -f /etc/os-release ] && grep -q 'Ubuntu 22' /etc/os-release; then
  pass "Ubuntu 22.04"
else
  warnf "不是 Ubuntu 22.04：$(sed -n 's/^PRETTY_NAME="\(.*\)"/\1/p' /etc/os-release 2>/dev/null)"
fi

if [ -n "$ROS_DISTRO" ]; then
  pass "ROS_DISTRO=$ROS_DISTRO"
else
  failf "ROS 环境没 source（先 source /opt/ros/humble/setup.bash）"
fi

echo "=== 2. 串口权限 ==="
if id -nG 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then
  pass "当前用户在 dialout 组"
else
  failf "不在 dialout 组：sudo usermod -aG dialout \$USER 后重新登录"
fi

echo "=== 3. 串口设备 ==="
if [ -d /dev/serial/by-id ] && [ -n "$(ls -A /dev/serial/by-id 2>/dev/null)" ]; then
  pass "检测到串口设备："
  ls /dev/serial/by-id/ | sed 's/^/         /'
else
  failf "没有 /dev/serial/by-id 设备（下位机 USB 线插了吗）"
fi

echo "=== 4. 依赖包 ==="
if ros2 pkg list 2>/dev/null | grep -qx livox_ros_driver2; then
  pass "livox_ros_driver2"
else
  warnf "livox_ros_driver2 没装（雷达驱动）"
fi
if ros2 pkg list 2>/dev/null | grep -qi 'point_lio\|pointlio\|fast_lio'; then
  pass "定位包在"
else
  warnf "没找到 point_lio / fast_lio（谁来发 /odom）"
fi

echo "=== 5. 本仓库编译产物 ==="
for b in ros/install/nav_control/lib/nav_control/figure8_node \
         ros/install/nav_sim/lib/nav_sim/perception_node \
         ros/install/nav_sim/lib/nav_sim/calib_node \
         ros/install/nav_protocol/lib/nav_protocol/serial_node; do
  if [ -x "$b" ]; then
    pass "$(basename "$b")"
  else
    failf "$b 不存在，先 bash build.sh"
  fi
done

echo "=== 6. 运行中的话题（节点没起就只是警告）==="
for t in /livox/lidar /odom /piles /scan; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then
    pass "话题 $t 存在"
  else
    warnf "话题 $t 没有（对应节点还没起？）"
  fi
done

echo
echo "通过 $ok / 警告 $warn / 失败 $fail"
if [ "$fail" -eq 0 ]; then
  echo "可以上车。"
  exit 0
fi
echo "先解决 FAIL 项再上车。"
exit 1

#!/usr/bin/env bash
# 临时：启动 40 秒，看 5 层激光在上桩方向（66.8°）的读数
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source ros/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1 ROS_DOMAIN_ID=42
export GAZEBO_MASTER_URI=http://127.0.0.1:11399
pkill -f gzserver 2>/dev/null || true
pkill -f figure8_node 2>/dev/null || true
sleep 1
ros2 launch nav_sim figure8.launch.py gui:=false rviz:=false > /tmp/scan_test.log 2>&1 &
LPID=$!
sleep 35
python3 - <<'EOF'
import subprocess, json, math, sys
for t in ["scan_low1","scan_low2","scan_mid","scan_high1","scan_high2"]:
    try:
        out = subprocess.run(["timeout","3","ros2","topic","echo","/"+t,"--no-daemon","--once","--field","ranges"],
                             capture_output=True, text=True, timeout=10).stdout
        ranges = eval(out.strip().split("---",1)[0]) if out.strip().startswith("[") else None
        if ranges is None: 
            print(t, "no data"); continue
        n = len(ranges); inc = 2*math.pi/n
        # 上桩方向 66.8°、下桩方向 45°、箱子方向 ~41°（世界系→车体系）
        def at(deg):
            i = int((math.radians(deg)+math.pi)/inc) % n
            return ranges[i] if ranges[i] < 100 else float('inf')
        print(f"{t}: 下桩45°={at(45):.2f} 箱46°={at(46):.2f} 上桩66.8°={at(66.8):.2f} 上桩67.5°={at(67.5):.2f} 66°={at(66):.2f}")
    except Exception as e:
        print(t, "err", e)
EOF
kill $LPID 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f figure8_node 2>/dev/null || true
pkill -f fake_mcu_node 2>/dev/null || true
pkill -f pile_detector 2>/dev/null || true

#!/usr/bin/env bash
# 本机 Ubuntu / WSL 一键跑模拟（无画面，跑完自动检查）。
# 一条命令，不用管当前目录、不用记 source 顺序：
#   bash /路径/到/srm-nav-2026/sim_local.sh
# 首次运行会自动编译；之后每次直接跑。
set -e
cd "$(dirname "$0")"          # 自动切到项目根目录（脚本在哪，项目就在哪）

# 1) 编译产物不存在 → 自动编译。
#    防坑：build-core 如果被 Windows 的 MSVC 建过，WSL 的 cmake 会报缓存路径不匹配，
#    这里检测到就删掉重建。
if [ ! -f ros/install/setup.bash ]; then
  echo "[sim_local] 没找到编译产物，开始编译（约 2~5 分钟）..."
  if grep -qi 'c:/Users' build-core/CMakeCache.txt 2>/dev/null; then
    echo "[sim_local] 检测到 build-core 是 Windows 建的，删掉重建"
    rm -rf build-core
  fi
  bash build.sh 2>&1 | tail -6
fi

# 2) 加载环境
source /opt/ros/humble/setup.bash
source ros/install/setup.bash

# 3) 确认包能找到
if ! ros2 pkg prefix nav_sim >/dev/null 2>&1; then
  echo "[sim_local] nav_sim 还是找不到，强制重新编译一次..."
  rm -rf build-core ros/install ros/build ros/log install-core
  bash build.sh 2>&1 | tail -6
  source /opt/ros/humble/setup.bash
  source ros/install/setup.bash
fi

# 4) 无画面端到端验证：桩数 / 状态机 / 坏帧，结果写 verify_result.txt
echo "[sim_local] 开始无画面模拟（约 4 分钟，跑完自动检查）..."
bash tools/verify_sim.sh
echo
cat verify_result.txt

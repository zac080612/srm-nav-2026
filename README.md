# srm-nav-2026

SRM 校内赛 **导航赛道** 上位机代码仓库：控制全向平移车绕两根桩完成 8 字路线（先下桩逆时针、后上桩顺时针），起终点同为停车位。

> 传感器：Livox Mid-360 雷达；计算设备：Intel NUC；通信：串口 115200 8N1（19 字节帧）。

---

## 目录结构

```text
srm-nav-2026/
├── core/                    # 纯 C++ 核心库（零 ROS 依赖，可单独编译 + 单测）
│   ├── include/nav/*.hpp    # 接口
│   ├── src/*.cpp            # 协议 / 控制律 / 状态机 / 轴映射 / 桩检测 / 标定
│   └── test/*.cpp           # 8 个单元测试
├── ros/src/
│   ├── nav_control/         # figure8 控制节点（状态机 + 控制律 + 打包）
│   ├── nav_sim/             # 模拟：fake_mcu + 桩检测 + Gazebo 世界 + launch + RViz2
│   └── nav_protocol/        # 串口后端（默认不编译，上车时启用）
├── tools/                   # headless 验证 / 诊断脚本
├── build.sh                 # 一键构建（core + ROS 包）
└── run_sim.sh               # 一键启动模拟
```

**分层原因**：`core/` 是「上车会咬人、但不需要 ROS 就能验」的逻辑（协议、控制、状态机、桩检测）。
抽成纯 C++ 后，在任何有 C++17 编译器的地方都能开发和单测，**模拟与实车共用同一份 core**，
实车只换「传感器源」和「执行器」。

---

## 环境要求

- WSL2 + Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11（`sudo apt install ros-humble-gazebo-ros-pkgs`）
- CMake、g++、git、python3

---

## 构建

```bash
bash build.sh
```

脚本会：先构建并安装 `core/`（含单元测试），再 `colcon build` 三个 ROS 包。

## 跑测试

```bash
cmake -S core -B ~/build-nav-core
cmake --build ~/build-nav-core -j
(cd ~/build-nav-core && ctest --output-on-failure)   # 8/8 全绿
```

## 跑模拟（可视化）

```bash
bash run_sim.sh
```

会弹出两个窗口：

- **Gazebo**：车从停车位出发，自动走完八字（绕下桩→交叉点→绕上桩→回停车位）；
- **RViz2**：实时显示激光扫描、检测到的桩（黄色圆柱）、坐标系。

无界面快速验证（确认「车能动、协议闭环」）：

```bash
bash tools/verify_sim.sh && cat verify_result.txt
```

---

## 模拟数据流

```
Gazebo LiDAR ──/scan──> pile_detector ──/piles──> figure8 ──/raw_frame──> fake_mcu ──/cmd_vel──> Gazebo 车
                        (桩检测：聚类+圆拟合)      (状态机+控制律+R_bc+SafeCmd+打包19字节)  (解析+换算)
```

---

## 技术要点

- **通信**：上→下固定 19 字节（`0xA5` + 长度 + seq + CRC8 + 3×float32 + CRC16）；下→上 `9+len`（含 cmd_id）。CRC 反射、无最终异或，自检值 CRC8(`"123456789"`)=`0x0B`、CRC16=`0x6F91`。
- **90° 轴交叉**：协议里 `vx` 进底盘 y、`vy` 取负进底盘 x，用 2×2 矩阵 `R_bc` 统一处理（实车需标定）。
- **控制律 A**（不需要 yaw/里程计/SLAM）：`v = v_t·t + k_r·(r−R)·u`，只用「桩在车体系的位置」绕桩。
- **状态机**：`Idle → StraightIn → OrbitDown → Cross → OrbitUp → StraightOut → Stop`。
- **安全**：限幅 ≤0.2、NaN/Inf 归零、500ms 看门狗、失联保护、launch 整体关闭。
- **桩检测**：点云聚类 + 已知半径圆拟合（模拟 2D LaserScan 与实车 3D 点云投影复用同一套）。

---

## 上车前待办

- [ ] 启用串口后端（`nav_protocol` 取消注释编译，改用 `/dev/serial/by-id/...`）
- [ ] 解析下行帧 `cmd_id=0x0001` 的 `game_progress`，替代模拟里写死的 `started`
- [ ] `R_bc` 实车标定（点云静止参照物法，`calib.cpp` 已实现算法）
- [ ] 速度量级实测（协议约 10000:1，`vx=1.0`≈四分之一杆）
- [ ] 实车看门狗（协议无超时保护，上位机崩溃时车会继续冲）


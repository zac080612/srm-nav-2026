# srm-nav-2026

SRM 校内赛 **导航赛道**上位机代码仓库。

> 目标：控制实车绕两根桩完成 8 字路线（先下瓣逆时针、后上瓣顺时针），起终点同为停车位。
> 依据：[`导航赛道学习计划书.md`](../导航赛道学习计划书.md) · [`协议与ROS2方案核查.md`](../协议与ROS2方案核查.md) · [`测试与交付操作手册.md`](../测试与交付操作手册.md)

---

## 1. 当前状态

已完成：**`core/` 的骨架**（接口、CMake、测试骨架全部就位；**实现留 TODO 给你填**）。
未开始：`ros/`（点云、Gazebo、串口、`fake_mcu`）—— 等 Ubuntu + ROS 2 环境就绪后再生成。

---

## 2. 目录结构

```text
srm-nav-2026/
├── core/                     ★ 纯 C++，零依赖，Windows / WSL / NUC 都能编译
│   ├── CMakeLists.txt
│   ├── include/nav/*.hpp     接口（已写好，含语义与判据注释）
│   ├── src/*.cpp             ← 只放了空实现 + TODO，等你填
│   └── test/*.cpp            测试骨架（断言里写死了协议真值）
├── ros/                      待生成：ROS 2 节点、fake_mcu、launch、world
├── tools/                    待生成：虚拟串口端到端、桩检测回归、R_bc 标定、故障注入
├── scripts/                  待生成：preflight.sh / postflight.sh
└── docs/                     协议要点、标定报告、每天的调试记录
```

**为什么这样分层**：`core/` 里全是"上车会咬人、但不需要 ROS 就能验"的逻辑（协议打包、CRC、
控制律、状态机、`R_bc` 映射、标定算法）。把它抽干净之后，你可以在任何有 C++17 编译器的地方
开发与单测，实车环境只跑薄薄一层包装。

---

## 3. 构建与测试

### 先装工具链（本机当前没有编译器和 CMake）

推荐 **WSL2 + Ubuntu**（一步同时解决编译、仿真、以及与 NUC 环境一致）：

```bash
wsl --install                 # Windows 侧，装完重启
# 进入 Ubuntu 后：
sudo apt update && sudo apt install -y build-essential cmake git
```

备选：Windows 原生装 CMake + MSYS2/MinGW-w64（只能编译 `core/`，到了 NUC 还得重来）。

### 构建 + 跑测试

```bash
cmake -S core -B build
cmake --build build -j
cd build && ctest --output-on-failure          # 全部
# CMake ≥ 3.20 也可以从根目录跑：ctest --test-dir build --output-on-failure
```

单跑某一个（回到仓库根目录）：

```bash
./build/test_crc                 # Linux / WSL
.\build\Debug\test_crc.exe       # Windows + Visual Studio 这类多配置生成器
```

> **骨架阶段测试会失败**，这是设计意图：断言里写死了协议给的真值，
> 你每填完一个 `src/*.cpp`，对应的测试就转绿。**不要改断言去迁就实现。**

---

## 4. TODO 清单（按这个顺序填）

| 顺序 | 文件 | 要写什么 | 对应测试 |
| --- | --- | --- | --- |
| 1 | `src/crc.cpp` | CRC8 / CRC16（先位运算版，再查表版） | `test_crc` |
| 2 | `src/frame.cpp` | 19 B 打包 + 下行流式组帧 | `test_frame` |
| 3 | `src/rbc_map.cpp` | `R_bc` 映射（一行矩阵乘法）+ 正交性自检 + 读参数 | `test_rbc` |
| 4 | `src/control.cpp` | 车体系绕桩控制律（径向/切向 P） | `test_control` |
| 5 | `src/safe_cmd.cpp` | 限幅 / `isfinite` / 500 ms 看门狗 | `test_safe_cmd` |
| 6 | `src/calib.cpp` | 位移方向估计（质心差法起步）+ 解 `R_bc` | `test_calib` |
| 7 | `src/state_machine.cpp` | 7 状态机（含"恰好触发一次"与丢桩降级） | `test_state_machine` |

每个 `.cpp` 顶部都写了**实现步骤 + 坑 + 手算自检例子**，照着填即可。

---

## 5. 协议真值速查（测试断言用的就是这些）

**上→下 19 字节**：`sof 0xA5` | `data_length=12`(u16 LE) | `seq` | `crc8`(覆盖 0–3，初值 `0xFF`) | `vx,vy,wz`(3×f32 LE) | `crc16`(覆盖 0–16，初值 `0xFFFF`，低字节在前)

**下→上 `9+len`**：`sof` | `data_length` | `seq` | `crc8` | `cmd_id`(u16 LE) | data | `crc16`(覆盖 `7+len`，**含 cmd_id**)
`cmd_id = 0x0001`（比赛状态，11 B）、`0x000B`（机器人状态，13 B）

**CRC 自检**（对 ASCII `"123456789"`）：CRC8 = `0x0B`，CRC16 = `0x6F91`

**三个参考帧**（`seq=1`）：

| 输入 | 19 字节 |
| --- | --- |
| `vx=1, vy=0, wz=0` | `A5 0C 00 01 26 00 00 80 3F 00 00 00 00 00 00 00 00 6B A3` |
| 停车帧 | `A5 0C 00 01 26 00 00 00 00 00 00 00 00 00 00 00 00 18 9F` |
| `vx=1, vy=-1, wz=3` | `A5 0C 00 01 26 00 00 80 3F 00 00 80 BF 00 00 40 40 D3 96` |

**反馈示例帧**：`A5 0B 00 01 5C 01 00 AA 2C 01 88 77 66 55 44 33 22 11 EF 74`
→ `cmd_id=0x0001`、`game_type=game_progress=0xA`、`stage_remain_time=300`、`sync_time_stamp=0x1122334455667788`

**安全约束**：`wz` 不生效；限幅是上位机责任；不要发 `NaN`/`Inf`；协议**没有超时保护** → 启动即发零速帧、异常/退出补发零速帧、500 ms 看门狗。

---

## 6. `R_bc` 是什么

那个「90° 轴交叉」在代码里的**全部**实现就是 `rbc_map.cpp` 里的一行矩阵乘法：

```
(vx, vy)^T = R_bc · (v_body_x, v_body_y)^T
```

`R_bc` 由 2026-10-11 的实车标定确定（点云静止参照物法，见操作手册 T7），写进参数文件。
**符号或轴搞反时只改参数、不改代码。** 默认值取协议字面直译，仅作起点参考，**不要当结论**。

---

## 7. 提交纪律

- 小步提交，每步是"能跑通的状态"；
- 提交信息写**为什么改**；
- `bags/`、`*.pcd`、`*.bag` 一律不进仓库（已在 `.gitignore`）；
- 每个第三方依赖在 README 标注出处与 LICENSE；**绝不包含 SRM 队内代码**。

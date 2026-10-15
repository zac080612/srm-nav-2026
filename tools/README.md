# tools/ —— 待生成（离线工具）

按操作手册第 1 章的顺序，这里将放以下**不需要实车、不需要 Gazebo** 的工具：

| 工具 | 用途 | 手册编号 | 能否只靠 `core/` |
| --- | --- | --- | --- |
| `loopback_test/` | `socat` 虚拟串口对：发送端 100 Hz × 5 s，接收端统计 CRC 错误 / 最大间隔 | T2 | 需要 POSIX 串口 → Linux/WSL |
| `replay_check/` | 用录制的点云（pcd/bag）跑桩检测，统计位置标准差与丢失率 | T6 | 需要 PCL → Linux |
| `rbc_calibrate/` | `R_bc` 标定：发 0.02 速度 → 取两帧点云 → 调 `core/` 的 `estimate_translation_direction` → 输出参数 | T7 | 算法在 `core/`，取点云要 ROS |
| `fault_injection/` | 拔串口 / 喂 NaN / 杀进程 / 雷达掉线的安全行为验证 | T10 | 混合 |

**已经能在 `core/` 里做的部分**：`rbc_calibrate` 的核心算法（`estimate_translation_direction`、
`solve_rbc_from_samples`）和 `fault_injection` 里的 NaN/看门狗用例，都已经有单测了
（`core/test/test_calib.cpp`、`core/test/test_safe_cmd.cpp`），不需要等 ROS。

**数据目录约定**：录制的点云与 bag 放仓库外的 `bags/` 或 `pcd/`（已在 `.gitignore`），
只把小样本（几帧）放进来做回归测试基线。

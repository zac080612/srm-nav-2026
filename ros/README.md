# ros/ —— 待生成（需要 Ubuntu + ROS 2 环境）

这里将来放**只能在 Linux + ROS 2 上编译**的部分：

| 包 | 内容 | 对应测试/任务 |
| --- | --- | --- |
| `nav_protocol_node/` | 把 `core/` 的帧打包接上串口（`termios`），100 Hz 定频发送 + 看门狗 | 操作手册 T2/T8 |
| `nav_perception/` | 点云裁剪 → 投影 2D → 聚类找桩；输出统一在车体系 | 操作手册 T6 |
| `nav_sim/` | **`fake_mcu`**（假下位机）、全向 URDF（planar move 插件）、两桩 world、仿真 launch | 操作手册 T3、计划书 2.7 |
| `nav_bringup/` | 实车与仿真的 launch、参数 yaml（含 `R_bc`、`R`、`v_t`、`k_r`、限幅） | 计划书第 9 节 |

**现在不生成的原因**：没有 ROS 2 环境时无法编译验证，生成了也只是死代码。
等你在 WSL2 或 NUC 上把 ROS 2 装好（并确认发行版是 Humble 还是 Jazzy），再按上面这张表生成骨架。

**注意**：`core/` 与这里的边界要守住 ——
- 共用且不依赖 ROS：`core/`
- 依赖 ROS/PCL/termios：`ros/`
这样仿真（10/8）与实车跑的是同一份 `core/`，才不会出现"仿真过了上车不一样"。

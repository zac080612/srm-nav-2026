# 实车启动：感知（点云→桩+激光）+ 控制 + 串口后端
#
# 启动前必须先单独起（本 launch 不含，因为要按现场 IP 配置）：
#   1. Livox 雷达驱动      → /livox/lidar
#   2. 定位（Point-LIO）   → /odom
#
# 运行：
#   ros2 launch nav_sim figure8_real.launch.py device:=/dev/serial/by-id/xxx

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 感知：Livox 点云 → /piles（桩）+ /scan（避障）----
    perception = Node(
        package="nav_sim",
        executable="perception_node",
        output="screen",
        parameters=[{
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            # 实车实测：雷达离地 0.2m。tf_z 把点云 z 抬到地面坐标系。
            "tf_z": LaunchConfiguration("tf_z"),
            # 桩下半部是半径 10cm 的半圆（高约 10cm）。高度切片只保留这一段：
            # 地面上方 0.02~0.15，既排除地面，也把 1cm 细杆和偏高的杂物切掉。
            "min_height": LaunchConfiguration("min_height"),
            "max_height": LaunchConfiguration("max_height"),
            "min_range": 0.30,
            "max_range": LaunchConfiguration("max_range"),
            "voxel": LaunchConfiguration("voxel"),
            # 半径范围按半圆实测收紧：只认半径 10cm 左右的圆
            "min_radius": LaunchConfiguration("min_radius"),
            "max_radius": LaunchConfiguration("max_radius"),
            "min_points": LaunchConfiguration("min_points"),
            "cluster_dist": LaunchConfiguration("cluster_dist"),
            # 圆形验证：弧长 30°（太严真桩会被滤掉），残差 0.02m
            "min_arc_deg": LaunchConfiguration("min_arc_deg"),
            "max_fit_rms": LaunchConfiguration("max_fit_rms"),
            # 雷达相对车体的平面变换：tf_yaw 按标定值（约 -90°，雷达装转了 90°）
            "tf_x": 0.0,
            "tf_y": 0.0,
            "tf_yaw": LaunchConfiguration("tf_yaw"),
        }],
    )

    # ---- 控制：状态机 + 绕桩 + 避障 → /raw_frame ----
    figure8 = Node(
        package="nav_control",
        executable="figure8_node",
        output="screen",
        parameters=[{
            "park_x": LaunchConfiguration("park_x"),
            "park_y": LaunchConfiguration("park_y"),
            "v_lin": LaunchConfiguration("v_lin"),
            "v_t": LaunchConfiguration("v_t"),
            "a_max": 1.0,
            "k_r": 0.8,
            # 真实速度上限(m/s)：在换算成协议值之前卡住，保证 v_lin/v_t 不打折
            "v_max_ms": LaunchConfiguration("v_max_ms"),
            # 协议值安全兜底（换算后），不是速度限制；默认放宽到 2.0 避免截断
            "speed_limit": LaunchConfiguration("speed_limit"),
            # 量纲换算：控制律按 m/s 规划，协议字段是与底盘速度成正比的数值
            "speed_scale": LaunchConfiguration("speed_scale"),
            # 实车等串口反馈的 game_progress 再启动；调试时可在命令行改 true
            "auto_start": LaunchConfiguration("auto_start"),
            # R_bc 轴映射：默认按协议字面直译，实车必须标定后改这四个值
            "r11": LaunchConfiguration("r11"),
            "r12": LaunchConfiguration("r12"),
            "r21": LaunchConfiguration("r21"),
            "r22": LaunchConfiguration("r22"),
        }],
    )

    # ---- 串口后端：/raw_frame → 串口 → 下位机；读反馈 → /game_progress ----
    serial = Node(
        package="nav_protocol",
        executable="serial_node",
        output="screen",
        parameters=[{
            "device": LaunchConfiguration("device"),
            "baud": LaunchConfiguration("baud"),
        }],
    )

    # 任一节点退出就整体关闭，避免车失控
    shutdown_on_any_exit = RegisterEventHandler(
        OnProcessExit(on_exit=[EmitEvent(event=Shutdown(
            reason="有节点退出，关闭整个系统，避免车失控"))])
    )

    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/ttyACM0",
                              description="串口设备（建议用 /dev/serial/by-id/... 稳定路径）"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("cloud_topic", default_value="/livox/lidar"),
        # ---- 感知：按实车实测值设默认（雷达离地 0.2m，桩半圆半径 10cm）----
        DeclareLaunchArgument("tf_z", default_value="0.20",
                              description="雷达离地高度(m)，把点云 z 抬到地面系"),
        DeclareLaunchArgument("tf_yaw", default_value="-1.22",
                              description="雷达相对车体的安装角(弧度)，实测约-70°"),
        DeclareLaunchArgument("min_height", default_value="-0.50",
                              description="高度切片放宽：不依赖 tf_z 精度，识别完全靠圆形状"),
        DeclareLaunchArgument("max_height", default_value="1.50",
                              description="高度切片放宽：桩半圆+细杆都在内，靠圆拟合+半径识别"),
        DeclareLaunchArgument("max_range", default_value="4.0",
                              description="只看场地内的距离(m)"),
        DeclareLaunchArgument("voxel", default_value="0.04"),
        DeclareLaunchArgument("min_radius", default_value="0.085",
                              description="桩半径下限(m)：实测约0.10，留拟合余量到0.085"),
        DeclareLaunchArgument("max_radius", default_value="0.15",
                              description="桩半径上限(m)：实测约0.10，留余量"),
        DeclareLaunchArgument("min_points", default_value="3"),
        DeclareLaunchArgument("cluster_dist", default_value="0.20"),
        DeclareLaunchArgument("min_arc_deg", default_value="0.0",
                              description="赛场干净，弧长检查关闭"),
        DeclareLaunchArgument("max_fit_rms", default_value="0.06",
                              description="赛场干净，残差放宽"),
        DeclareLaunchArgument("park_x", default_value="-1.5"),
        DeclareLaunchArgument("park_y", default_value="-2.5"),
        DeclareLaunchArgument("v_lin", default_value="0.40"),
        DeclareLaunchArgument("v_t", default_value="0.40"),
        DeclareLaunchArgument("v_max_ms", default_value="0.40",
                              description="真实速度上限(m/s)，在换算成协议值前卡住"),
        DeclareLaunchArgument("speed_limit", default_value="2.0",
                              description="协议值安全兜底(换算后)，不是速度限制"),
        DeclareLaunchArgument("speed_scale", default_value="2.00",
                              description="协议数值 / (m/s)；实车标定约 1.8~2.2，取中间值"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        # R_bc 实测为单位矩阵：协议 vx 直通车前、vy 直通左方，无轴交叉
        DeclareLaunchArgument("r11", default_value="1.0"),
        DeclareLaunchArgument("r12", default_value="0.0"),
        DeclareLaunchArgument("r21", default_value="0.0"),
        DeclareLaunchArgument("r22", default_value="1.0"),
        perception,
        figure8,
        serial,
        shutdown_on_any_exit,
    ])

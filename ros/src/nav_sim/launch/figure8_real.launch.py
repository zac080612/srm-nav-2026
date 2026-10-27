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
            # 高度切片：桩的高度范围（地面以上）。上车按桩实际高度调。
            "min_height": 0.10,
            "max_height": 1.50,
            "min_range": 0.30,
            "max_range": 6.00,
            "voxel": 0.05,
            # 雷达相对车体（livox_frame → base_link）的平面变换，实测后填
            "tf_x": 0.0,
            "tf_y": 0.0,
            "tf_z": 0.0,
            "tf_yaw": 0.0,
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
        DeclareLaunchArgument("park_x", default_value="-1.5"),
        DeclareLaunchArgument("park_y", default_value="-2.5"),
        DeclareLaunchArgument("v_lin", default_value="0.20"),
        DeclareLaunchArgument("v_t", default_value="0.20"),
        DeclareLaunchArgument("speed_limit", default_value="0.20"),
        DeclareLaunchArgument("speed_scale", default_value="1.0",
                              description="协议数值 / (m/s)；实车量出 1 个协议值对应多少 m/s 后填"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("r11", default_value="0.0"),
        DeclareLaunchArgument("r12", default_value="1.0"),
        DeclareLaunchArgument("r21", default_value="-1.0"),
        DeclareLaunchArgument("r22", default_value="0.0"),
        perception,
        figure8,
        serial,
        shutdown_on_any_exit,
    ])

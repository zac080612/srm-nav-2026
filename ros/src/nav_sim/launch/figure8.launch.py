import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory("nav_sim")
    world_path = os.path.join(pkg_dir, "world", "figure8.world")
    rviz_path = os.path.join(pkg_dir, "config", "figure8.rviz")

    # 用 gazebo_ros 官方 launch 启动：负责设置 GAZEBO_PLUGIN_PATH 并加载
    # libgazebo_ros_init.so。factory:=false 关掉 libgazebo_ros_factory.so
    # （它在 WSL2 上创建 /spawn_entity 服务会崩；车已内联进 world，不需要 spawn）。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"]),
        launch_arguments={
            "world": world_path,
            "gui": LaunchConfiguration("gui"),
            "factory": "false",
        }.items(),
    )

    fake_mcu = Node(
        package="nav_sim",
        executable="fake_mcu_node",
        output="screen",
    )

    pile_detector = Node(
        package="nav_sim",
        executable="pile_detector_node",
        output="screen",
    )

    figure8 = Node(
        package="nav_control",
        executable="figure8_node",
        output="screen",
        parameters=[{
            "park_x": -1.5,
            "park_y": -2.5,
            "v_lin": 0.7,
            "a_max": 1.0,
            "v_t": 0.7,
            "k_r": 0.8,
            "speed_limit": 1.5,
            "speed_scale": 1.0,
            "auto_start": True,
        }],
    )

    # 真实串口后端（默认注释，上车才启用）：
    # serial = Node(
    #     package="nav_protocol",
    #     executable="serial_node",
    #     output="screen",
    #     parameters=[{
    #         "device": "/dev/ttyACM0",
    #         "baud": 115200,
    #     }],
    # )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # 关键安全网：任一节点退出就关闭整个 launch。
    shutdown_on_any_exit = RegisterEventHandler(
        OnProcessExit(on_exit=[EmitEvent(event=Shutdown(
            reason="有节点退出，关闭整个模拟，避免车失控"))])
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="是否启动 Gazebo GUI"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="是否启动 RViz2"),
        gazebo,
        fake_mcu,
        pile_detector,
        figure8,
        rviz,
        shutdown_on_any_exit,
    ])

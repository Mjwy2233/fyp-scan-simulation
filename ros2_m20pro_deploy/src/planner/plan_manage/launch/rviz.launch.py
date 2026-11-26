"""启动 RViz2 可视化界面，加载 SCAN-Planner 预置视图配置。

Start RViz2 with a SCAN-Planner display configuration (default.rviz or navigation.rviz).

运行方式 / Usage:
  ros2 launch scan_planner rviz.launch.py
  ros2 launch scan_planner rviz.launch.py rviz_config:=navigation.rviz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_rviz(context):
    """解析 rviz_config 路径（支持包内相对文件名或绝对路径）并拉起 RViz2。"""
    rviz_arg = LaunchConfiguration("rviz_config").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "1", "true", "yes", "on"
    )
    if os.path.isabs(rviz_arg):
        config = rviz_arg
    else:
        config = os.path.join(
            get_package_share_directory("scan_planner"), "rviz", rviz_arg
        )
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", config],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    ]


def generate_launch_description():
    """声明 RViz2 启动参数 / Declare RViz2 launch arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false",
                                  description="是否使用 /clock 仿真时钟"),
            DeclareLaunchArgument("rviz_config", default_value="default.rviz",
                                  description="RViz 配置文件名 (位于 share/scan_planner/rviz/) 或绝对路径"),
            OpaqueFunction(function=_launch_rviz),
        ]
    )

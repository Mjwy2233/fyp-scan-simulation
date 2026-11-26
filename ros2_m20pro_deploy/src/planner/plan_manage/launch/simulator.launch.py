"""SCAN-Planner 轻量化确定性仿真环境子启动文件。

Native ROS 2 simulation launcher (global map source + local sensor renderer + odom vis).

节点组成与数据流 / Node Roles & Data Flow:
1. 全局地图源 (二选一):
   - `map_generator/map_pub` (`use_pcd_map:=true`): 加载先验 `.pcd` 文件并发布 `/map_generator/global_cloud`
   - `mockamap/mockamap_node` (`use_pcd_map:=false`): 生成随机 3D 障碍物森林并发布 `/map_generator/global_cloud`
2. 局部传感器仿真器 (`local_sensing_node/pcl_render_node` 或 `opengl_render_node`):
   - 订阅 `/map_generator/global_cloud` 与 `/quad_0/body_pose`
   - 按雷达/相机视场裁剪并发布 `/quad_0/cloud`、`/quad_0/lidar_pose` (或 `/quad_0/depth`、`/quad_0/camera_pose`)
3. 里程计可视化与 TF 广播 (`odom_visualization/odom_visualization`):
   - 订阅 `/quad_0/body_pose`，广播 `world -> base` TF 并发布 `/quad_0/path` 历史轨迹供 RViz2 显示
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    """将启动参数字符串解析为布尔值 / Parse launch argument string into bool."""
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    """构建仿真环境节点列表 (真机模式下直接返回空列表) / Build simulation nodes."""
    if _as_bool(LaunchConfiguration("is_real_world").perform(context)):
        return []

    use_pcd_map = _as_bool(LaunchConfiguration("use_pcd_map").perform(context))
    use_gpu = _as_bool(LaunchConfiguration("use_gpu").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    pcd_map_file = LaunchConfiguration("pcd_map_file").perform(context)
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    map_x = float(LaunchConfiguration("map_size_x").perform(context))
    map_y = float(LaunchConfiguration("map_size_y").perform(context))
    map_z = float(LaunchConfiguration("map_size_z").perform(context))

    # 校验 PCD 地图文件是否存在 / Ensure PCD file exists when use_pcd_map is enabled
    if use_pcd_map and (not pcd_map_file or not os.path.isfile(pcd_map_file)):
        raise RuntimeError(
            "use_pcd_map=true requires pcd_map_file to reference an existing PCD file"
        )

    scan_share = get_package_share_directory("scan_planner")
    simulator_yaml = os.path.join(scan_share, "config", "simulator.yaml")
    nodes = []

    # 节点 1: 全局点云地图发布节点 (PCD 先验地图 或 随机障碍物地图)
    if use_pcd_map:
        nodes.append(
            Node(
                package="map_generator",
                executable="map_pub",
                name="map_pub",
                namespace="map_generator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "file_name": pcd_map_file,
                        "frame_id": "world",
                        "publish_rate": 0.2,
                        "downsample_res": 0.1,
                    }
                ],
            )
        )
    else:
        nodes.append(
            Node(
                package="mockamap",
                executable="mockamap_node",
                name="mockamap_node",
                output="screen",
                parameters=[
                    simulator_yaml,
                    {
                        "use_sim_time": use_sim_time,
                        "x_length": int(map_x),
                        "y_length": int(map_y),
                        "z_length": int(map_z),
                    },
                ],
                remappings=[("mock_map", "/map_generator/global_cloud")],
            )
        )

    # 节点 2 & 3: 局部点云/深度渲染器 + 里程计轨迹可视化与 world->base TF 广播
    nodes.extend(
        [
            Node(
                package="local_sensing_node",
                executable="opengl_render_node" if use_gpu else "pcl_render_node",
                name="pcl_render_node",
                output="screen",
                parameters=[
                    simulator_yaml,
                    {
                        "use_sim_time": use_sim_time,
                        "sensor_type": sensor_type,
                        "body_pose_topic": "body_pose",
                        "map.x_size": map_x,
                        "map.y_size": map_y,
                        "map.z_size": map_z,
                        "use_global_map_topic": True,
                        "pcd_map_file": pcd_map_file,
                    },
                ],
                remappings=[
                    ("global_map", "/map_generator/global_cloud"),
                    ("body_pose", "/quad_0/body_pose"),
                    ("cloud", "/quad_0/cloud"),
                    ("sensor_cloud", "/quad_0/sensor_cloud"),
                    ("depth", "/quad_0/depth"),
                    ("dyn_cloud", "/quad_0/dyn_cloud"),
                    ("uav_cloud", "/quad_0/uav_cloud"),
                ],
            ),
            Node(
                package="odom_visualization",
                executable="odom_visualization",
                name="odom_visualization",
                output="screen",
                parameters=[simulator_yaml, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("body_pose", "/quad_0/body_pose"),
                    ("pose", "/quad_0/pose"),
                    ("path", "/quad_0/path"),
                    ("velocity", "/quad_0/velocity"),
                    ("trajectory", "/quad_0/trajectory"),
                    ("robot", "/quad_0/robot"),
                    ("height", "/quad_0/height"),
                ],
            ),
        ]
    )
    return nodes


def generate_launch_description():
    """声明仿真环境参数 / Declare simulation launch arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )

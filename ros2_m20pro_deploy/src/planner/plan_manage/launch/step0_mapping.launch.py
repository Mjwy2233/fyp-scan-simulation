"""Step 0 启动文件：山猫 M20 Pro 手动遥控扫图建图流水线（双雷达点云融合 + Super-LIO/SLAM + rosbag2）。

Step 0 manual mapping pipeline launch file (Dual-LiDAR fusion + Super-LIO/SLAM + rosbag2).

业务流程 / Workflow (Lynx M20 Pro 1F -> 2F Manual Mapping):
  1. 启动本文件：拉起双雷达点云融合 (`dual_lidar_fusion_node`，可选，GOS 直连原厂 `/LIDAR/POINTS` 时可关闭)、
     SLAM 建图后端 (`super_lio` 主方案 / `m20_builtin` / `fast_lio` / `elevator_lio`) 与 `ros2 bag record`。
  2. 操作员手动遥控山猫 M20 Pro 从 1 楼走廊平稳走过折返楼梯至 2 楼楼道。
  3. 结束后在另一终端运行一键存图与校验命令：
       `ros2 run scan_planner save_map.py --pcd <输出地图路径.pcd>`
"""

import datetime
import os
import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    """将启动参数字符串解析为布尔值 / Parse launch string to bool."""
    return str(value).lower() in ("1", "true", "yes", "on")


def _default_repo_map(scan_share):
    """推导仓库根目录默认 map.pcd 路径，供 mode:=sim 离线演练 Step 0 流水线。"""
    ws_root = os.path.abspath(os.path.join(scan_share, "..", "..", "..", ".."))
    candidate = os.path.join(ws_root, "map.pcd")
    return candidate if os.path.isfile(candidate) else ""


def _setup(context):
    """解析 YAML 配置与命令行覆盖参数，组装 Step 0 建图流水线节点。"""
    scan_share = get_package_share_directory("scan_planner")
    fusion_yaml = os.path.join(scan_share, "config", "dual_lidar_fusion.yaml")
    step0_yaml_path = os.path.join(scan_share, "config", "step0_mapping.yaml")
    with open(step0_yaml_path, "r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle).get("step0_mapping", {})

    mode = LaunchConfiguration("mode").perform(context) or cfg.get("default_mode", "sim")
    if mode not in ("sim", "real"):
        raise RuntimeError("mode must be 'sim' or 'real'")

    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    record_bag_arg = LaunchConfiguration("record_bag").perform(context)
    record_bag = (
        _as_bool(record_bag_arg)
        if record_bag_arg != ""
        else bool(cfg.get("output", {}).get("record_bag", True))
    )
    fusion_arg = LaunchConfiguration("enable_dual_lidar_fusion").perform(context)
    enable_fusion = (
        _as_bool(fusion_arg)
        if fusion_arg != ""
        else (True if mode == "sim" else bool(cfg.get("real_enable_dual_lidar_fusion", True)))
    )
    enable_rviz = _as_bool(LaunchConfiguration("enable_rviz").perform(context))
    slam_backend = LaunchConfiguration("slam_backend").perform(context) or cfg.get(
        "slam_backend", "super_lio"
    )
    map_dir = LaunchConfiguration("map_dir").perform(context) or cfg.get("output", {}).get(
        "map_dir", "/tmp/scan_planner_maps"
    )
    map_name = LaunchConfiguration("map_name").perform(context) or cfg.get("output", {}).get(
        "map_name", "building_1f_2f"
    )
    bag_dir = LaunchConfiguration("bag_dir").perform(context) or cfg.get("output", {}).get(
        "bag_dir", "/tmp/scan_planner_bags"
    )
    pcd_save_path = os.path.join(map_dir, f"{map_name}.pcd")

    real_topics = cfg.get("real_topics", {})
    front_topic = real_topics.get("front_lidar", "/lidar_front/points")
    rear_topic = real_topics.get("rear_lidar", "/lidar_rear/points")
    fused_topic = real_topics.get("fused_lidar", "/fused_lidar/points")
    slam_cloud_input = (
        fused_topic if enable_fusion else real_topics.get("m20_merged_lidar", "/LIDAR/POINTS")
    )

    actions = [
        LogInfo(
            msg=(
                f"[Step 0 Mapping] mode={mode}, slam_backend={slam_backend}, "
                f"dual_lidar_fusion={enable_fusion}, slam_cloud={slam_cloud_input}, "
                f"pcd_save_path={pcd_save_path}, record_bag={record_bag}"
            )
        )
    ]

    # --------------------------------------------------------------------------
    # 1. 仿真建图演练前端 (仅 mode:=sim): 加载 map.pcd 并切分为前后模拟双雷达
    # --------------------------------------------------------------------------
    if mode == "sim":
        sim_pcd = LaunchConfiguration("sim_pcd_file").perform(context) or _default_repo_map(scan_share)
        if not sim_pcd or not os.path.isfile(sim_pcd):
            raise RuntimeError(
                "Step 0 sim mode requires sim_pcd_file (default: <workspace>/map.pcd)"
            )
        controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
        actions.extend(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(scan_share, "launch", "simulator.launch.py")
                    ),
                    launch_arguments={
                        "is_real_world": "false",
                        "sensor_type": "lidar",
                        "use_gpu": "false",
                        "use_pcd_map": "true",
                        "pcd_map_file": sim_pcd,
                        "use_sim_time": str(use_sim_time).lower(),
                    }.items(),
                ),
                Node(
                    package="scan_planner",
                    executable="open_loop_controller",
                    name="sim_mapping_odom",
                    output="screen",
                    parameters=[
                        controllers_yaml,
                        {
                            "use_sim_time": use_sim_time,
                            "init_x": -5.5,
                            "init_y": 5.5,
                            "init_z": 0.5,
                        },
                    ],
                    remappings=[("body_pose", "/quad_0/body_pose")],
                ),
                Node(
                    package="scan_planner",
                    executable="sim_dual_lidar_splitter.py",
                    name="sim_dual_lidar_splitter",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "front_topic": front_topic,
                            "rear_topic": rear_topic,
                            "sim_base_frame": "base",
                            "target_frame": "base_link",
                        }
                    ],
                ),
            ]
        )

    # --------------------------------------------------------------------------
    # 2. 前后双激光雷达点云融合节点 (保留 XYZIRT 字段供 Super-LIO 去畸变与存盘)
    # --------------------------------------------------------------------------
    if enable_fusion:
        actions.append(
            Node(
                package="scan_planner",
                executable="dual_lidar_fusion_node",
                name="dual_lidar_fusion",
                output="screen",
                parameters=[
                    fusion_yaml,
                    {
                        "use_sim_time": use_sim_time,
                        "front_topic": front_topic,
                        "rear_topic": rear_topic,
                        "output_topic": fused_topic,
                        "pcd_save_path": pcd_save_path,
                        "enable_map_accumulation": True,
                    },
                ],
            )
        )

    # --------------------------------------------------------------------------
    # 3. 外部 SLAM 建图后端挂钩 (仅 mode:=real，默认 super_lio)
    # --------------------------------------------------------------------------
    if mode == "real":
        slam_map = cfg.get("slam_launch", {})
        if slam_backend in slam_map:
            pkg_name = slam_map[slam_backend]["package"]
            launch_file = slam_map[slam_backend]["launch_file"]
            try:
                slam_share = get_package_share_directory(pkg_name)
                slam_launch_path = os.path.join(slam_share, "launch", launch_file)
                if os.path.isfile(slam_launch_path):
                    actions.append(
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(slam_launch_path)
                        )
                    )
                else:
                    actions.append(
                        LogInfo(
                            msg=(
                                f"[Step 0] Package '{pkg_name}' found, but '{slam_launch_path}' "
                                "not found. Launch SLAM externally or update step0_mapping.yaml."
                            )
                        )
                    )
            except PackageNotFoundError:
                actions.append(
                    LogInfo(
                        msg=(
                            f"[Step 0] External SLAM package '{pkg_name}' is not installed in this "
                            "isolated workspace yet. Fusion & rosbag2 remain active; launch "
                            f"{slam_backend} (e.g. Liansheng-Wang/Super-LIO ros2 branch) subscribing "
                            f"to cloud='{slam_cloud_input}' and IMU='{real_topics.get('imu', '/IMU')}'."
                        )
                    )
                )
        elif slam_backend == "m20_builtin":
            actions.append(
                LogInfo(
                    msg=(
                        "[Step 0] Using Lynx M20 Pro built-in NOS mapping (`sudo drmap mapping`). "
                        "rosbag2 backup recording is active."
                    )
                )
            )
        else:
            actions.append(
                LogInfo(
                    msg=(
                        "[Step 0] Using dual_lidar_fusion TF map accumulation mode. "
                        "Ensure map_frame -> base_link TF is published while walking 1F -> 2F."
                    )
                )
            )

    # --------------------------------------------------------------------------
    # 4. Rosbag2 录制进程 (记录原始点云、融合点云、IMU、里程计与 TF 树)
    # --------------------------------------------------------------------------
    if record_bag:
        os.makedirs(bag_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(bag_dir, f"{map_name}_{stamp}")
        topics = cfg.get("output", {}).get(
            "bag_topics", [slam_cloud_input, "/IMU", "/tf", "/tf_static"]
        )
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", bag_path] + list(topics),
                output="screen",
            )
        )
        actions.append(LogInfo(msg=f"[Step 0] Recording rosbag2 to: {bag_path}"))

    # --------------------------------------------------------------------------
    # 5. 可选 RViz2 可视化界面
    # --------------------------------------------------------------------------
    if enable_rviz:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(scan_share, "launch", "rviz.launch.py")
                ),
                launch_arguments={
                    "rviz_config": "navigation.rviz",
                    "use_sim_time": str(use_sim_time).lower(),
                }.items(),
            )
        )

    save_srv = cfg.get("slam_launch", {}).get(slam_backend, {}).get("save_map_service", "/save_map")
    actions.append(
        LogInfo(
            msg=(
                "[Step 0] When finished walking 1F -> 2F, save & validate the prior map with:\n"
                f"  ros2 run scan_planner save_map.py --slam-service {save_srv} --pcd {pcd_save_path}"
            )
        )
    )
    return actions


def generate_launch_description():
    """声明 Step 0 建图流水线启动参数 / Declare Step 0 launch arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="",
                                  description="运行模式: sim (仿真演练) 或 real (山猫M20 Pro真机建图)"),
            DeclareLaunchArgument("slam_backend", default_value="",
                                  description="SLAM后端: super_lio (主推) / m20_builtin / fast_lio / elevator_lio / fusion_accum"),
            DeclareLaunchArgument("enable_dual_lidar_fusion", default_value="",
                                  description="是否启用双雷达融合节点 (true=外接rslidar双路融合 / false=GOS直连/LIDAR/POINTS)"),
            DeclareLaunchArgument("map_dir", default_value="",
                                  description="先验 PCD 地图保存目录"),
            DeclareLaunchArgument("map_name", default_value="",
                                  description="先验 PCD 地图文件名 (不含 .pcd)"),
            DeclareLaunchArgument("bag_dir", default_value="",
                                  description="rosbag2 录制目录"),
            DeclareLaunchArgument("record_bag", default_value="",
                                  description="是否录制 rosbag2 (true/false，留空读 YAML)"),
            DeclareLaunchArgument("enable_rviz", default_value="false",
                                  description="是否同时拉起 RViz2 实时查看点云"),
            DeclareLaunchArgument("sim_pcd_file", default_value="",
                                  description="仅 mode:=sim 使用的仿真场景 PCD 路径"),
            DeclareLaunchArgument("use_sim_time", default_value="false",
                                  description="是否使用仿真时钟 /clock"),
            OpaqueFunction(function=_setup),
        ]
    )

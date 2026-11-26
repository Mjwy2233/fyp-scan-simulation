"""Step 1 启动文件：山猫 M20 Pro 先验地图重定位 (Super-LIO) + SCAN-Planner 避障导航 + RViz 选点。

Step 1 prior-map relocalization & RViz Set-Goal navigation launch (Lynx M20 Pro + Super-LIO).

严格边界声明 / Hard Constraint Guarantee:
  - 核心规划与避障算法 (`scan_planner_node` 内的 3D Sliding Map、双圆柱膨胀、投影 A*、
    B-spline 轨迹优化) 及山猫 M20 Pro 底层运控逻辑完全未被修改。
  - 本文件仅负责前端双雷达融合、先验 `.pcd` 地图发布、Super-LIO (或 M20 原厂) 定位接入、
    标准速度指令 (`geometry_msgs/Twist`: `vx, vy, omega`) 话题重映射与 RViz2 选点可视化。
"""

import os
import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    """将启动参数字符串解析为布尔值 / Parse launch string to bool."""
    return str(value).lower() in ("1", "true", "yes", "on")


def _default_repo_map(scan_share):
    """推导仓库根目录默认 map.pcd 路径 / Locate default <workspace>/map.pcd."""
    ws_root = os.path.abspath(os.path.join(scan_share, "..", "..", "..", ".."))
    candidate = os.path.join(ws_root, "map.pcd")
    return candidate if os.path.isfile(candidate) else ""


def _setup(context):
    """读取 step1_navigation.yaml 并根据 sim/real 模式构建完整导航流水线。"""
    scan_share = get_package_share_directory("scan_planner")
    go2_share = get_package_share_directory("go2_description")

    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    simulator_yaml = os.path.join(scan_share, "config", "simulator.yaml")
    fusion_yaml = os.path.join(scan_share, "config", "dual_lidar_fusion.yaml")
    step1_yaml_path = os.path.join(scan_share, "config", "step1_navigation.yaml")

    with open(step1_yaml_path, "r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle).get("step1_navigation", {})

    prior_cfg = cfg.get("prior_map", {})
    real_cfg = cfg.get("real_robot", {})
    nav_cfg = cfg.get("navigation", {})

    mode = LaunchConfiguration("mode").perform(context) or cfg.get("default_mode", "sim")
    if mode not in ("sim", "real"):
        raise RuntimeError("mode must be 'sim' or 'real'")
    is_real = mode == "real"

    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    enable_rviz = _as_bool(LaunchConfiguration("enable_rviz").perform(context))
    rviz_config = LaunchConfiguration("rviz_config").perform(context) or "navigation.rviz"
    reference_path_file = LaunchConfiguration("reference_path_file").perform(context)

    navi_mode_arg = LaunchConfiguration("navi_mode").perform(context)
    navi_mode = int(navi_mode_arg) if navi_mode_arg != "" else int(nav_cfg.get("sim_navi_mode", 1))
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")

    controller_arg = LaunchConfiguration("controller_mode").perform(context)
    controller_mode = "closed_loop" if is_real else (controller_arg or nav_cfg.get("sim_controller_mode", "closed_loop"))
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")

    fusion_arg = LaunchConfiguration("enable_dual_lidar_fusion").perform(context)
    if fusion_arg != "":
        enable_fusion = _as_bool(fusion_arg)
    else:
        enable_fusion = bool(
            real_cfg.get("enable_dual_lidar_fusion", True)
            if is_real
            else nav_cfg.get("sim_enable_dual_lidar_fusion", True)
        )

    pcd_map_file = (
        LaunchConfiguration("pcd_map_file").perform(context)
        or prior_cfg.get("pcd_file", "")
        or (_default_repo_map(scan_share) if not is_real else "")
    )
    if not pcd_map_file or not os.path.isfile(pcd_map_file):
        raise RuntimeError(
            "Step 1 requires an existing prior PCD map via pcd_map_file:=<path> "
            "or step1_navigation.yaml (prior_map.pcd_file)"
        )

    init_x = float(LaunchConfiguration("init_x").perform(context) or nav_cfg.get("sim_init_x", -5.5))
    init_y = float(LaunchConfiguration("init_y").perform(context) or nav_cfg.get("sim_init_y", 5.5))
    init_z = float(LaunchConfiguration("init_z").perform(context) or nav_cfg.get("sim_init_z", 0.5))

    common = {"use_sim_time": use_sim_time}
    actions = [
        LogInfo(
            msg=(
                f"[Step 1 Navigation] mode={mode}, navi_mode={navi_mode}, "
                f"controller={controller_mode}, dual_lidar_fusion={enable_fusion}, "
                f"prior_pcd={pcd_map_file}"
            )
        )
    ]

    # --------------------------------------------------------------------------
    # 1. 加载先验点云地图 (Step 0 产物) -> /map_generator/global_cloud
    # --------------------------------------------------------------------------
    map_frame_id = (
        real_cfg.get("planner_frame_id", "world")
        if is_real
        else prior_cfg.get("frame_id", "world")
    )
    actions.append(
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
                    "frame_id": map_frame_id,
                    "publish_rate": float(prior_cfg.get("publish_rate", 0.2)),
                    "downsample_res": float(prior_cfg.get("downsample_res", 0.1)),
                }
            ],
        )
    )

    # --------------------------------------------------------------------------
    # 2. 按 sim / real 模式组装前端感知与定位话题映射
    # --------------------------------------------------------------------------
    if is_real:
        topics = real_cfg.get("topics", {})
        loc_backend = (
            LaunchConfiguration("localization_backend").perform(context)
            or real_cfg.get("localization_backend", "super_lio")
        )
        front_topic = topics.get("front_lidar", "/lidar_front/points")
        rear_topic = topics.get("rear_lidar", "/lidar_rear/points")
        fused_topic = topics.get("fused_lidar", "/fused_lidar/points")

        # 若切换为山猫 M20 Pro 原厂定位对比模式，默认自动对准原厂 /ODOM 与 /ALIGNED_POINTS
        if loc_backend == "m20_builtin":
            body_pose_topic = "/ODOM"
            sensor_pose_topic = "/ODOM"
            planner_cloud_topic = "/ALIGNED_POINTS"
            cloud_is_world = True
            need_extrinsic = False
        else:
            body_pose_topic = topics.get("body_pose", "/lio/odom")
            sensor_pose_topic = topics.get("sensor_pose", body_pose_topic)
            planner_cloud_topic = (
                fused_topic
                if enable_fusion
                else topics.get("m20_merged_lidar", "/LIDAR/POINTS")
            )
            cloud_is_world = bool(real_cfg.get("cloud_is_world", False))
            need_extrinsic = bool(real_cfg.get("need_extrinsic", False))

        cmd_vel_topic = (
            LaunchConfiguration("cmd_vel_topic").perform(context)
            or topics.get("cmd_vel", "/cmd_vel")
        )
        safety = real_cfg.get("safety_limits", {})
        controller_overrides = {
            **common,
            "max_vx": float(safety.get("max_vx", 0.45)),
            "max_vy": float(safety.get("max_vy", 0.20)),
            "max_vyaw": float(safety.get("max_vyaw", 0.60)),
        }

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
                            "enable_map_accumulation": False,
                        },
                    ],
                )
            )

        # 外部 SLAM 重定位后端挂钩 (默认 super_lio -> relocation.py)
        loc_hooks = real_cfg.get("localization_launch", {})
        if loc_backend in loc_hooks:
            pkg_name = loc_hooks[loc_backend]["package"]
            launch_file = loc_hooks[loc_backend]["launch_file"]
            try:
                loc_share = get_package_share_directory(pkg_name)
                loc_launch = os.path.join(loc_share, "launch", launch_file)
                if os.path.isfile(loc_launch):
                    actions.append(
                        IncludeLaunchDescription(PythonLaunchDescriptionSource(loc_launch))
                    )
                else:
                    actions.append(
                        LogInfo(
                            msg=f"[Step 1] Found '{pkg_name}' but missing '{loc_launch}'; launch localization externally."
                        )
                    )
            except PackageNotFoundError:
                actions.append(
                    LogInfo(
                        msg=(
                            f"[Step 1] External localization package '{pkg_name}' not installed locally. "
                            f"Waiting for odometry on '{body_pose_topic}' and point cloud on '{planner_cloud_topic}'."
                        )
                    )
                )
        else:
            actions.append(
                LogInfo(
                    msg=(
                        f"[Step 1] Localization backend='{loc_backend}': subscribing odom='{body_pose_topic}', "
                        f"cloud='{planner_cloud_topic}', cloud_is_world={cloud_is_world}."
                    )
                )
            )
    else:
        body_pose_topic = "/quad_0/body_pose"
        cmd_vel_topic = (
            LaunchConfiguration("cmd_vel_topic").perform(context) or "/quad_0/cmd_vel"
        )
        controller_overrides = dict(common)

        if enable_fusion:
            sensor_pose_topic = "/quad_0/body_pose"
            planner_cloud_topic = "/fused_lidar/points"
            cloud_is_world = False
            need_extrinsic = False
            actions.extend(
                [
                    Node(
                        package="scan_planner",
                        executable="sim_dual_lidar_splitter.py",
                        name="sim_dual_lidar_splitter",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "sim_cloud_topic": "/quad_0/cloud",
                                "body_pose_topic": body_pose_topic,
                                "front_topic": "/lidar_front/points",
                                "rear_topic": "/lidar_rear/points",
                                "sim_base_frame": "base",
                                "target_frame": "base_link",
                            }
                        ],
                    ),
                    Node(
                        package="scan_planner",
                        executable="dual_lidar_fusion_node",
                        name="dual_lidar_fusion",
                        output="screen",
                        parameters=[
                            fusion_yaml,
                            {
                                "use_sim_time": use_sim_time,
                                "front_topic": "/lidar_front/points",
                                "rear_topic": "/lidar_rear/points",
                                "output_topic": "/fused_lidar/points",
                                "target_frame": "base_link",
                                "enable_map_accumulation": False,
                            },
                        ],
                    ),
                ]
            )
        else:
            sensor_pose_topic = "/quad_0/lidar_pose"
            planner_cloud_topic = "/quad_0/cloud"
            cloud_is_world = True
            need_extrinsic = False

        actions.extend(
            [
                Node(
                    package="local_sensing_node",
                    executable="pcl_render_node",
                    name="pcl_render_node",
                    output="screen",
                    parameters=[
                        simulator_yaml,
                        {
                            "use_sim_time": use_sim_time,
                            "sensor_type": "lidar",
                            "body_pose_topic": "body_pose",
                            "use_global_map_topic": True,
                            "pcd_map_file": pcd_map_file,
                        },
                    ],
                    remappings=[
                        ("global_map", "/map_generator/global_cloud"),
                        ("body_pose", body_pose_topic),
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
                    parameters=[simulator_yaml, common],
                    remappings=[
                        ("body_pose", body_pose_topic),
                        ("pose", "/quad_0/pose"),
                        ("path", "/quad_0/path"),
                        ("velocity", "/quad_0/velocity"),
                        ("trajectory", "/quad_0/trajectory"),
                        ("robot", "/quad_0/robot"),
                        ("height", "/quad_0/height"),
                    ],
                ),
                Node(
                    package="scan_planner",
                    executable="go2_gait_publisher",
                    name="go2_gait_publisher",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[("body_pose", body_pose_topic)],
                ),
            ]
        )

    # --------------------------------------------------------------------------
    # 3. 核心 SCAN-Planner 规划与避障节点 (核心算法未做任何修改) + 机器人 URDF
    # --------------------------------------------------------------------------
    actions.extend(
        [
            Node(
                package="scan_planner",
                executable="scan_planner_node",
                name="scan_planner_node",
                output="screen",
                parameters=[
                    planner_yaml,
                    {
                        **common,
                        "fsm.navi_mode": navi_mode,
                        "grid_map.frame_id": map_frame_id,
                        "grid_map.sensor_type": "lidar",
                        "grid_map.cloud_is_world": cloud_is_world,
                        "grid_map.need_extrinsic": need_extrinsic,
                    },
                ],
                remappings=[
                    ("body_pose", body_pose_topic),
                    ("sensor_pose", sensor_pose_topic),
                    ("cloud", planner_cloud_topic),
                    ("move_base_simple/goal", "/move_base_simple/goal"),
                    ("initial_path", "/initial_path"),
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="go2_robot_state_publisher",
                output="screen",
                parameters=[
                    common,
                    {
                        "robot_description": Command(
                            ["xacro ", os.path.join(go2_share, "xacro", "robot.xacro"),
                             " use_gazebo:=false"]
                        )
                    },
                ],
            ),
        ]
    )

    # --------------------------------------------------------------------------
    # 4. 轨迹跟踪控制器 (输出标准 geometry_msgs/Twist: vx, vy, omega)
    # --------------------------------------------------------------------------
    if controller_mode == "open_loop":
        actions.append(
            Node(
                package="scan_planner",
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=[
                    controllers_yaml,
                    common,
                    {"init_x": init_x, "init_y": init_y, "init_z": init_z},
                ],
                remappings=[
                    ("planning/bspline", "/planning/bspline"),
                    ("body_pose", body_pose_topic),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="scan_planner",
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[controllers_yaml, controller_overrides],
                remappings=[
                    ("body_pose", body_pose_topic),
                    ("cmd_vel", cmd_vel_topic),
                ],
            )
        )
        if not is_real:
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_kinematic_sim",
                    name="go2_kinematic_sim",
                    output="screen",
                    parameters=[
                        controllers_yaml,
                        common,
                        {
                            "init_x": init_x,
                            "init_y": init_y,
                            "init_z": init_z,
                            "publish_tf": False,
                        },
                    ],
                    remappings=[
                        ("body_pose", body_pose_topic),
                        ("cmd_vel", cmd_vel_topic),
                    ],
                )
            )

    # --------------------------------------------------------------------------
    # 5. Mode 3 3D 选点桥接 / 参考路径演示发布器
    # --------------------------------------------------------------------------
    if navi_mode == 3:
        if reference_path_file:
            if not os.path.isfile(reference_path_file):
                raise RuntimeError(f"reference_path_file not found: {reference_path_file}")
            actions.append(
                Node(
                    package="scan_planner",
                    executable="reference_path_publisher.py",
                    name="reference_path_publisher",
                    output="screen",
                    parameters=[reference_path_file, common],
                    remappings=[
                        ("body_pose", body_pose_topic),
                        ("initial_path", "/initial_path"),
                    ],
                )
            )
        elif bool(nav_cfg.get("enable_3d_goal_bridge", True)):
            actions.append(
                Node(
                    package="scan_planner",
                    executable="goal_3d_bridge.py",
                    name="goal_3d_bridge",
                    output="screen",
                    parameters=[
                        {
                            **common,
                            "frame_id": map_frame_id,
                            "floor_change_z_threshold": float(
                                nav_cfg.get("floor_change_z_threshold", 0.8)
                            ),
                            "stair_anchors": nav_cfg.get("stair_anchors", []),
                        }
                    ],
                    remappings=[
                        ("body_pose", body_pose_topic),
                        ("clicked_point", "/clicked_point"),
                        ("goal_pose", "/move_base_simple/goal"),
                        ("initial_path", "/initial_path"),
                    ],
                )
            )

    # --------------------------------------------------------------------------
    # 6. RViz2 预置导航界面 (加载 navigation.rviz)
    # --------------------------------------------------------------------------
    if enable_rviz:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(scan_share, "launch", "rviz.launch.py")
                ),
                launch_arguments={
                    "rviz_config": rviz_config,
                    "use_sim_time": str(use_sim_time).lower(),
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    """声明 Step 1 重定位与导航启动参数 / Declare Step 1 launch arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="",
                                  description="运行模式: sim (PC仿真) 或 real (山猫M20 Pro真机)"),
            DeclareLaunchArgument("pcd_map_file", default_value="",
                                  description="Step 0 生成的先验 PCD 地图绝对路径 (sim 默认 <ws>/map.pcd)"),
            DeclareLaunchArgument("navi_mode", default_value="",
                                  description="1=RViz 2D Goal Pose, 3=3D跨层选点/楼梯参考路径"),
            DeclareLaunchArgument("controller_mode", default_value="",
                                  description="closed_loop (输出cmd_vel) 或 open_loop (3D跨层仿真)"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="",
                                  description="覆盖底盘速度指令输出话题 (如 /cmd_vel 或 /cmd_vel_dryrun)"),
            DeclareLaunchArgument("localization_backend", default_value="",
                                  description="真机定位后端: super_lio (主推) / m20_builtin / fast_lio_localization / elevator_lio / external"),
            DeclareLaunchArgument("enable_dual_lidar_fusion", default_value="",
                                  description="是否启用双雷达融合节点 (true=外接双路融合 / false=GOS直连/LIDAR/POINTS)"),
            DeclareLaunchArgument("reference_path_file", default_value="",
                                  description="可选: Mode 3 自动发布的 3D 参考路径 YAML 文件"),
            DeclareLaunchArgument("enable_rviz", default_value="true",
                                  description="是否启动预置 RViz2 导航界面 (true/false)"),
            DeclareLaunchArgument("rviz_config", default_value="navigation.rviz",
                                  description="RViz2 配置文件名 (默认 navigation.rviz)"),
            DeclareLaunchArgument("init_x", default_value="", description="仿真初始 X (m)"),
            DeclareLaunchArgument("init_y", default_value="", description="仿真初始 Y (m)"),
            DeclareLaunchArgument("init_z", default_value="", description="仿真初始 Z (m)"),
            DeclareLaunchArgument("use_sim_time", default_value="false",
                                  description="是否使用仿真时钟 /clock"),
            OpaqueFunction(function=_setup),
        ]
    )

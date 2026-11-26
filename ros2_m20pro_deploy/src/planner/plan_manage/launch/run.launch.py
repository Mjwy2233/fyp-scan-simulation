"""SCAN-Planner 主启动文件：支持电脑端轻量化仿真与真机话题重映射。

Main ROS 2 launch entry point for simulation and real-robot remapping.

节点启动顺序与依赖关系 / Node Roles & Dependency Order:
1. `scan_planner_node` (核心规划器，严禁修改内部算法):
   - 订阅: `body_pose` (机身里程计), `sensor_pose` (传感器位姿), `cloud` / `depth` (局部感知),
           `/move_base_simple/goal` (Mode 1 目标) 或 `/initial_path` (Mode 3 全局参考路径)
   - 发布: `/planning/bspline` (平滑避障 B-spline 轨迹), `/grid_map/occupancy*`, `/optimal_list`
2. `go2_robot_state_publisher` (`robot_state_publisher`):
   - 发布 URDF 静态连杆 TF 与 `/robot_description` 供 RViz2 显示四足模型
3. 轨迹跟踪控制器 (二选一):
   - `open_loop_controller`: 沿 3D B-spline 直接发布 `body_pose` (用于 3D 跨层/斜坡仿真)
   - `closed_loop_controller`: 跟踪 B-spline 并输出 `geometry_msgs/Twist` 速度指令 (`vx, vy, omega`)
     至 `/cmd_vel` (真机) 或 `/quad_0/cmd_vel` (仿真，配合 `go2_kinematic_sim`)
4. `reference_path_publisher.py` (可选，仅 `navi_mode:=3` 且传入 `reference_path_file` 时启动):
   - 等待首帧 `body_pose` 后发布一次 3D 参考路径至 `/initial_path`
5. 仿真辅助栈 (仅 `is_real_world:=false` 时启动):
   - `go2_gait_publisher`: 发布仿真可视化虚拟关节角 `/joint_states`
   - `simulator.launch.py`: 启动全局地图 (`map_pub` 或 `mockamap_node`) + 传感器渲染 (`pcl_render_node`) + `odom_visualization`
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    """将启动参数字符串解析为布尔值 / Parse launch argument string into bool."""
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    """根据启动参数动态组装节点列表与话题重映射 / Build launch actions from runtime args."""
    scan_share = get_package_share_directory("scan_planner")
    go2_share = get_package_share_directory("go2_description")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")

    # 读取运行模式、传感器与控制器参数 / Read mode, sensor, and controller arguments
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    reference_path_file = LaunchConfiguration("reference_path_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))

    # 参数合法性校验 / Validate mutually dependent launch parameters
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )
    if reference_path_file and navi_mode != 3:
        raise RuntimeError("reference_path_file is only valid when navi_mode=3")
    if reference_path_file and not os.path.isfile(reference_path_file):
        raise RuntimeError(
            "reference_path_file must reference an existing ROS 2 parameter YAML"
        )

    # 确定仿真初始位置 (Mode 1 默认在随机地图起点，Mode 2/3 默认在 map.pcd 坡道起点)
    mode_default_init = (-19.0, 1.0, 0.25) if navi_mode == 1 else (-5.5, 5.5, 0.5)
    initial_position = []
    for name, default in zip(("init_x", "init_y", "init_z"), mode_default_init):
        value = LaunchConfiguration(name).perform(context)
        initial_position.append(default if value == "" else float(value))
    init_x, init_y, init_z = initial_position

    # 区分真机与仿真的输入话题及坐标系约定 / Configure topics & frames for real vs sim
    if is_real:
        # 真机默认映射至 Elevator-LIO / FAST-LIO2 接口 (更细粒度配置推荐使用 step1_navigation.launch.py)
        body_pose = "/LIO/odom_vehicle"
        sensor_pose = "/LIO/odom_imu"
        cloud = "/LIO/clouds_lidar"
        depth = "/camera/aligned_depth_to_color/image_raw"
        cloud_is_world = False
        need_extrinsic = True
        intrinsics = {
            "grid_map.cx": 317.19183349609375,
            "grid_map.cy": 256.4806823730469,
            "grid_map.fx": 609.5884399414062,
            "grid_map.fy": 609.22021484375,
        }
    else:
        # 仿真模式映射至 pcl_render_node 与仿真控制器话题，点云已在 world 坐标系下
        body_pose = "/quad_0/body_pose"
        sensor_pose = "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cloud_is_world = True
        need_extrinsic = False
        intrinsics = {}

    common = {"use_sim_time": use_sim_time}
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
    }

    # 节点 1: SCAN-Planner 核心规划器节点 (含 3D 滑动地图、投影 A* 与 B-spline 优化)
    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml] + ([keypoints_file] if keypoints_file else []) + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/initial_path"),
            ],
        )
    ]

    # 节点 2: 机器人模型状态发布器 (提供 /robot_description 供 RViz2 渲染四足模型)
    actions.append(
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
        )
    )

    # 节点 3: 轨迹跟踪控制器 (开环用于 3D 跨层仿真，闭环输出标准 Twist 速度指令用于 2D 仿真与真机)
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
                    ("body_pose", body_pose),
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
                parameters=[controllers_yaml, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("cmd_vel", "/cmd_vel" if is_real else "/quad_0/cmd_vel"),
                ],
            )
        )
        # 闭环仿真模式下额外拉起运动学积分器，将 /quad_0/cmd_vel 积分为 /quad_0/body_pose
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
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/quad_0/cmd_vel"),
                    ],
                )
            )

    # 节点 4 (可选): Mode 3 演示参考路径发布器
    if reference_path_file:
        actions.append(
            Node(
                package="scan_planner",
                executable="reference_path_publisher.py",
                name="reference_path_publisher",
                output="screen",
                parameters=[reference_path_file, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("initial_path", "/initial_path"),
                ],
            )
        )

    # 节点 5 (仅仿真): 虚拟关节动画发布器 + 仿真地图与传感器渲染子 Launch
    if not is_real:
        actions.extend(
            [
                Node(
                    package="scan_planner",
                    executable="go2_gait_publisher",
                    name="go2_gait_publisher",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[("body_pose", body_pose)],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(scan_share, "launch", "simulator.launch.py")
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name)
                        for name in (
                            "is_real_world",
                            "sensor_type",
                            "use_gpu",
                            "use_pcd_map",
                            "pcd_map_file",
                            "map_size_x",
                            "map_size_y",
                            "map_size_z",
                            "use_sim_time",
                        )
                    }.items(),
                ),
            ]
        )
    return actions


def generate_launch_description():
    """声明所有可配置的命令行启动参数 / Declare launch arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false",
                                  description="false=PC仿真 / true=真机话题重映射"),
            DeclareLaunchArgument("navi_mode", default_value="1",
                                  description="1=RViz 2D目标, 2=预设航点, 3=参考路径(跨层)"),
            DeclareLaunchArgument("sensor_type", default_value="lidar",
                                  description="感知输入类型: lidar 或 depth"),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop",
                                  description="closed_loop(输出cmd_vel) 或 open_loop(3D跨层仿真)"),
            DeclareLaunchArgument("keypoints_file", default_value="",
                                  description="Mode 2 使用的航点 YAML 绝对路径"),
            DeclareLaunchArgument("reference_path_file", default_value="",
                                  description="Mode 3 使用的 3D 参考路径 YAML 绝对路径"),
            DeclareLaunchArgument("use_gpu", default_value="false",
                                  description="仿真局部感知是否启用 OpenGL GPU 后端"),
            DeclareLaunchArgument("use_pcd_map", default_value="false",
                                  description="true=加载外部 PCD 点云地图, false=随机生成地图"),
            DeclareLaunchArgument("pcd_map_file", default_value="",
                                  description="use_pcd_map=true 时待加载的先验 PCD 文件路径"),
            DeclareLaunchArgument("map_size_x", default_value="40.0",
                                  description="仿真地图 X 轴尺寸 (m)"),
            DeclareLaunchArgument("map_size_y", default_value="40.0",
                                  description="仿真地图 Y 轴尺寸 (m)"),
            DeclareLaunchArgument("map_size_z", default_value="5.0",
                                  description="仿真地图 Z 轴尺寸 (m)"),
            DeclareLaunchArgument("init_x", default_value="",
                                  description="覆盖仿真初始位置 X (m)"),
            DeclareLaunchArgument("init_y", default_value="",
                                  description="覆盖仿真初始位置 Y (m)"),
            DeclareLaunchArgument("init_z", default_value="",
                                  description="覆盖仿真初始位置 Z (m)"),
            DeclareLaunchArgument("use_sim_time", default_value="false",
                                  description="是否使用 /clock 仿真时钟"),
            OpaqueFunction(function=_setup),
        ]
    )

# `scan_planner` 统一功能包说明

本包整合了四足机器人室内与跨楼层（1F–2F）自主导航所需的全部前端融合、建图、重定位接入、可视化与仿真工具，同时**严格保持原有 `SCAN-Planner` 核心避障/规划算法与底层控制逻辑不变**。

## 目录结构

- `src/`
  - `scan_planner_node.cpp`, `scan_replan_fsm.cpp`, `planner_manager.cpp`：原版核心规划器（未改动）
  - `closed_loop_controller.cpp`, `open_loop_controller.cpp`, `go2_kinematic_sim.cpp`, `go2_gait_publisher.cpp`：原版控制器与仿真节点（未改动）
  - `dual_lidar_fusion_node.cpp`：**新增** 前后双激光雷达时间同步、TF 统一坐标变换、拼接滤波与单 PCD 导出节点
- `launch/`
  - `run.launch.py`, `simulator.launch.py`, `rviz.launch.py`：原版仿真与可视化启动入口（已补全中英文注释）
  - `step0_mapping.launch.py`：**新增** Step 0 手动扫图建图 + 双雷达融合 + rosbag 录制一键启动
  - `step1_navigation.launch.py`：**新增** Step 1 先验地图加载 + 重定位接入 + SCAN-Planner 导航 + `navigation.rviz` 一键启动
- `config/`
  - `planner.yaml`, `simulator.yaml`, `controllers.yaml`, `reference_path.map.yaml`, `keypoints.example.yaml`：原版参数（数值未改，已逐项注释）
  - `dual_lidar_fusion.yaml`, `step0_mapping.yaml`, `step1_navigation.yaml`：**新增** 硬件解耦的真机/仿真切换参数配置
- `rviz/`
  - `default.rviz`：原版视图配置（已从 `launch/` 归位）
  - `navigation.rviz`：**新增** Step 1 专用导航视图（含先验地图、融合点云、占据栅格、轨迹与 `2D Pose Estimate` / `2D Goal Pose` / `Publish Point` 工具）
- `scripts/`
  - `keypoint_recorder.py`：交互式航点录制工具（原 `tools/README.md` 已整合至此）
    ```bash
    ros2 run scan_planner keypoint_recorder.py --odom /LIO/odom_vehicle --output keypoints.yaml
    ```
  - `reference_path_publisher.py`：Mode 3 跨层参考路径一次性发布器
  - `save_map.py`：**新增** Step 0 一键触发保存先验 `.pcd` 地图并自动校验
  - `validate_map.py`：**新增** `.pcd` 地图完整性与 Z 轴跨层高度跨度校验脚本
  - `goal_3d_bridge.py`：**新增** RViz 3D 选点（`/clicked_point`）与 1F–2F 折返楼梯锚点桥接节点
  - `sim_dual_lidar_splitter.py`：**新增** 电脑端仿真双雷达切分器，用于闭环验证 `dual_lidar_fusion_node`

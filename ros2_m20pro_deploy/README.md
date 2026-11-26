# ROS 2 Humble 四足机器人（山猫 M20 Pro + Super-LIO）自主导航与真机部署工程

> **与原仓库内容隔离说明**：
> - 本目录 **`ros2_m20pro_deploy/`** 位于独立分支 **`ros2-m20pro-deploy`**，与本仓库 `main` 分支原有 **ROS 1 Noetic 地形仿真实验**（`maps/`、`patches/`、`results/`、`scripts/` 与根目录 `README.md`）**完全隔离，原有文件改动为 0 行**。
> - 本工程基于上游 ROS 2 移植版 [xiaoqi371317/SCAN-Planner-Ros2](https://github.com/xiaoqi371317/SCAN-Planner-Ros2)（提交 `103bce4`，Apache-2.0，上游说明保留于 [README_UPSTREAM_ROS2.md](README_UPSTREAM_ROS2.md) 与 [NOTICE](NOTICE)）扩展构建，**严格保持原版核心避障规划与底层控制器 C++ 代码零改动**。
> - **导师审批用完整真机部署方案**：请直接审阅 **[DEPLOYMENT_PLAN.md](DEPLOYMENT_PLAN.md)**。

---

## 1. 本次新增上传内容总览

面向 **云深处山猫 M20 Pro**（AOS / NOS / GOS 三主机 RK3588 架构，前后双 96 线速腾 RSAIRY 雷达 + 200 Hz IMU）室内平地与 **1 楼至 2 楼折返楼梯** 自主定位导航，本次完成了以下 5 项核心工程交付：

1. **单包规范化整合与全面中英文注释**：
   - 将全部前端融合、建图、重定位接入、可视化与辅助校验模块统一整合至单一 ROS 2 功能包 `scan_planner`（[src/planner/plan_manage](src/planner/plan_manage)）。
   - 为全部 YAML 参数、Launch 文件、Python 工具与新增 C++ 节点补全中英文注释（原版参数数值与运行逻辑零改动）。
2. **前后双激光雷达点云融合节点（`dual_lidar_fusion_node`）**：
   - 位于 [src/planner/plan_manage/src/dual_lidar_fusion_node.cpp](src/planner/plan_manage/src/dual_lidar_fusion_node.cpp)（C++17）。
   - 实现前后雷达 `ApproximateTime` 时间同步、TF 统一变换至 `base_link`、山猫 M20 机身自遮挡盒裁剪、**标准 32 字节对齐 RoboSense `PointXYZIRT`（保留 `ring` 与逐点 `float64 timestamp`）**输出以支持 `Super-LIO` 运动去畸变，并提供 `~/save_pcd` 按需导出单一 `.pcd` 服务。
3. **采纳导师建议，主定位后端升级为 `Super-LIO`（`ros2` 分支）**：
   - 真机 Step 0 建图与 Step 1 重定位默认接入 [Liansheng-Wang/Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)（`ros2` 分支，具备 $\mathcal{S}^2$ 流形在线重力对齐防楼梯地图歪斜、OctVox+HKNN 适配 ARM RK3588、原生内置 `relocation.py` 先验地图重定位）。
   - 同时在配置中保留 **山猫 M20 Pro 原厂定位（`m20_builtin`）** 与 `FAST-LIO2 / Elevator-LIO` 一键切换，便于 1F–2F 对比实验与兜底。
4. **Step 0 建图 + Step 1 选点导航两步法流水线与 RViz2 预置界面**：
   - **Step 0**：[step0_mapping.launch.py](src/planner/plan_manage/launch/step0_mapping.launch.py) + [save_map.py](src/planner/plan_manage/scripts/save_map.py) + [validate_map.py](src/planner/plan_manage/scripts/validate_map.py)（遥控走完 1F→2F 后一键保存并自动校验 `.pcd` 完整性与 Z 轴高差）。
   - **Step 1**：[step1_navigation.launch.py](src/planner/plan_manage/launch/step1_navigation.launch.py) + [navigation.rviz](src/planner/plan_manage/rviz/navigation.rviz) + [goal_3d_bridge.py](src/planner/plan_manage/scripts/goal_3d_bridge.py)（加载先验地图，在 RViz2 中支持 `2D Pose Estimate` 初始化定位、`2D Goal Pose` 同层选点及 `Publish Point` 3D 跨楼层折返楼梯选点）。
5. **环境隔离一键部署脚本与导师审批文档**：
   - [deploy.sh](deploy.sh)：支持 `--mode sim` / `--mode real`，独立工作空间 `colcon build --symlink-install` 与局部 `source`，零全局环境污染，并内置真机 `--real-confirm` 安全互锁。
   - [DEPLOYMENT_PLAN.md](DEPLOYMENT_PLAN.md)：完整架构图、Topic 列表、**GOS (`10.21.33.104`) ARM64 Docker + `taskset -c 4-7` 绑大核**防污染方案（严禁触碰 AOS/NOS）、硬件核对清单与三级渐进验收流程。

---

## 2. 目录结构与新增/修改明细

```text
ros2_m20pro_deploy/
├── README.md                                        # [新增] 本说明文档（面向导师审阅与快速复现）
├── README_UPSTREAM_ROS2.md                          # [保留] 上游 SCAN-Planner-Ros2 原版说明
├── DEPLOYMENT_PLAN.md                               # [新增] 导师审批用《山猫 M20 Pro 真机部署方案》
├── deploy.sh                                        # [新增] 环境隔离一键构建与 sim/real 启动脚本
├── map.pcd                                          # [沿用] 跨层坡道验证先验点云地图 (46 MB)
├── LICENSE / NOTICE                                 # [保留] Apache-2.0 许可证与上游署名
└── src/
    ├── planner/
    │   ├── plan_env / path_searching / bspline_opt / traj_utils / scan_planner_msgs  # [核心模块：0 行改动]
    │   └── plan_manage/                             # [统一整合单包: scan_planner]
    │       ├── CMakeLists.txt / package.xml         # [更新] 编译 dual_lidar_fusion_node 并安装 scripts/rviz
    │       ├── README.md                            # [新增] 单包结构与工具说明
    │       ├── config/
    │       │   ├── dual_lidar_fusion.yaml           # [新增] 山猫 M20 Pro 双雷达外参、XYZIRT 保留与机身裁剪
    │       │   ├── step0_mapping.yaml               # [新增] Step 0 Super-LIO 建图、rosbag2 与校验配置
    │       │   ├── step1_navigation.yaml            # [新增] Step 1 重定位、安全限幅、话题重映射与楼梯锚点
    │       │   └── planner/simulator/controllers/reference_path.map/keypoints.example.yaml  # [仅补中英文注释，数值零改动]
    │       ├── launch/
    │       │   ├── step0_mapping.launch.py          # [新增] Step 0 手动建图与录包一键启动
    │       │   ├── step1_navigation.launch.py       # [新增] Step 1 先验图重定位与选点导航一键启动
    │       │   └── run.launch.py / simulator.launch.py / rviz.launch.py  # [补注释，保持向后兼容]
    │       ├── rviz/
    │       │   ├── default.rviz                     # [归位] 原版视图（从 launch/ 迁入）
    │       │   └── navigation.rviz                  # [新增] Step 1 专用导航视图（含 Set Goal 工具）
    │       ├── scripts/
    │       │   ├── save_map.py                      # [新增] Step 0 一键存图并自动校验
    │       │   ├── validate_map.py                  # [新增] PCD 点数/NaN/3D 包围盒与 Z 高差校验
    │       │   ├── goal_3d_bridge.py                # [新增] RViz 3D 选点与 1F–2F 折返楼梯锚点桥接
    │       │   ├── sim_dual_lidar_splitter.py       # [新增] PC 仿真双雷达切分器（用于闭环测试融合节点）
    │       │   └── keypoint_recorder.py / reference_path_publisher.py  # [仅补注释，逻辑零改动]
    │       └── src/
    │           ├── dual_lidar_fusion_node.cpp       # [新增] C++17 双雷达同步融合与 XYZIRT 保留节点
    │           └── (其余 7 个原版规划与控制器 .cpp)   # [严格未改动：0 行变更]
    └── simulator/                                   # [原版轻量化仿真器包：0 行改动]
```

### 严格约束核查声明
- **核心避障与规划算法 0 改动**：`plan_env`（3D Sliding Map、双圆柱膨胀）、`path_searching`（投影 A*）、`bspline_opt`（B-spline 轨迹优化）、`scan_planner_node.cpp`、`scan_replan_fsm.cpp`、`planner_manager.cpp` 与上游 `103bce4` 完全一致。
- **底层运动控制 0 改动**：未改动任何步态或底层控制代码，`closed_loop_controller.cpp` 与上游一致，仅通过 Launch/YAML 重映射标准 `geometry_msgs/Twist` 速度话题。
- **原仓库内容 0 改动**：本仓库 `main` 分支的 `maps/`、`patches/`、`results/`、`scripts/`、根 `README.md` 均未修改。

---

## 3. 电脑端一键仿真演示（Ubuntu 22.04 / ROS 2 Humble）

```bash
cd ros2_m20pro_deploy

# 1. 首次在独立工作空间隔离编译（不修改 ~/.bashrc 或全局 Python）
./deploy.sh --mode sim --build --build-only

# 2. 演示 A（默认）：Step 1 闭环仿真 + 双雷达融合 + 先验地图 + RViz 2D Goal 选点
./deploy.sh --mode sim
# -> 在 RViz2 工具栏点击 "2D Goal Pose" 选定终点，实时观察避障轨迹与 /quad_0/cmd_vel 输出

# 3. 演示 B：跨层 3D 坡道一键自动演示（Mode 3 + map.pcd + 双雷达融合）
./deploy.sh --mode sim --step demo3d

# 4. 演示 C：交互式 3D 跨层选点（在 RViz2 点击 "Publish Point" 选中上层目标）
./deploy.sh --mode sim --step step1 --navi-mode 3 --controller open_loop

# 5. 演示 D：Step 0 手动建图、双雷达融合与一键保存校验（仿真演练）
./deploy.sh --mode sim --step step0
# 另一终端执行一键存图与校验：
source /opt/ros/humble/setup.bash && source install/local_setup.bash
ros2 run scan_planner save_map.py --pcd /tmp/scan_planner_maps/building_1f_2f.pcd
```

> 无显示器/SSH 环境可在任意命令后追加 `--no-rviz`。

---

## 4. 自动化编译与仿真验证结果（WSL2 Ubuntu 22.04 / ROS 2 Humble）

| 验证项 | 测试内容 | 实测结果 |
| :--- | :--- | :--- |
| **隔离编译** | `./deploy.sh --mode sim --fast-build --build-only` | **0 警告编译通过**（`-Wall -Wextra -Wpedantic`） |
| **单元与 Launch 测试** | `colcon test --packages-select scan_planner` | **5/5 套件、15/15 用例 100% 通过** |
| **Step 0 双雷达融合 + 存图** | `step0_mapping.launch.py mode:=sim` + `save_map.py` | `/fused_lidar/points` 输出标准 32B `PointXYZIRT`（`x,y,z,intensity,ring,timestamp`）；导出 6,235 点（0 NaN），`validate_map.py` **PASSED** |
| **Step 1 Mode 1 选点闭环** | `./deploy.sh --mode sim --no-rviz` + `/move_base_simple/goal` | 加载 316,493 点先验图，生成 8 条 B-spline，输出 485 帧有效 `Twist`（`max_vx=0.49 m/s`, `max_vyaw=1.00 rad/s`） |
| **Step 1 Mode 3 跨层 3D 选点** | `step1_navigation.launch.py navi_mode:=3` + `/clicked_point` | 串联 26 点 `/initial_path`，规划 15 条 3D B-spline，机身平稳爬升 `+1.392 m` |
| **真机安全互锁** | `./deploy.sh --mode real --skip-build`（无 `--real-confirm`） | 安全拦截（退出码 `3`），未拉起任何底盘控制节点 |

---

## 5. 山猫 M20 Pro 真机部署说明

请导师重点审阅 **[DEPLOYMENT_PLAN.md](DEPLOYMENT_PLAN.md)**，核心要点包括：
- **三主机物理隔离**：严禁触碰 AOS（`10.21.33.103`）与 NOS（`10.21.33.106`），全部部署于 **GOS（`10.21.33.104`）ARM64 ROS 2 Humble Docker 容器**（配合 `taskset -c 4-7` 绑定大核）或外接 Ubuntu 22.04 Humble 笔记本。
- **双路径点云接入**：GOS 上可直连原厂已拼接的 `/LIDAR/POINTS`（`--enable-fusion false`，不改原厂 `send_separately`）；外接主机通过 `rslidar_sdk` 组播收前后两路时开启 `dual_lidar_fusion_node`（`--enable-fusion true`），完整保留 `XYZIRT` 喂给 `Super-LIO`。
- **分级实测验收**：Step 0 扫图验收 → Step 1 原地静态重定位（`/cmd_vel_dryrun` 空跑）→ 1 楼平地短距离限速闭环 → 1F–2F 折返楼梯分段及全程导航。

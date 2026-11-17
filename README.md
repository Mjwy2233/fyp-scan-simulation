# SCAN-Planner 地形规划仿真

本仓库整理了毕业设计中用于复现 SCAN-Planner 规划与开环轨迹跟踪的脚本、合成地图、补丁和结果摘要。完整规划器及 ROS 环境需单独安装。上游使用 Unitree Go2 默认参数，实验尚未验证 M20 的足端接触、步态稳定性或爬楼能力。

| 场景 | 条件 | 当前证据 |
| --- | --- | --- |
| 平地 | 新合成点云 | 地图几何通过；运行对照待做 |
| 直坡 | 10°，长 4 m | 上行一次开环到达；参考路径发布后约 14.43 s，末次三维误差约 5.6 cm；下行和重复试验待做 |
| 楼梯 | 6 级，级高 0.12 m，踏面 0.30 m | 地图几何通过；上下行待做 |

历史结果见 [坡道运行摘要](results/20261103-ramp-up.json)；它来自迁移归档，已去掉原环境路径和单调时钟绝对值。开环的到达条件为模拟机体到目标的三维距离不超过 0.20 m，持续至少 1 s。参考路径用地图真值生成，不能由此推断机器人能自行找楼梯。

## 复现方法

使用 Ubuntu 20.04、ROS Noetic 和 Python 3。按[上游说明](https://github.com/wuyi2121/SCAN-Planner)安装构建依赖（含 Armadillo），另行获取上游工作树。以下从本仓库根目录开始。已有旧 VM 时，先核对提交及补丁状态，避免重复应用。

```bash
SIM_ROOT="$PWD"
git clone https://github.com/wuyi2121/SCAN-Planner.git "$HOME/SCAN-Planner"
git -C "$HOME/SCAN-Planner" checkout --detach 348e8a590a50a5a6bbab8d8c6dcfd171f009be26
git -C "$HOME/SCAN-Planner" apply --check "$SIM_ROOT/patches/planner-refinement.patch"
git -C "$HOME/SCAN-Planner" apply "$SIM_ROOT/patches/planner-refinement.patch"
source /opt/ros/noetic/setup.bash
(cd "$HOME/SCAN-Planner" && catkin_make)
source "$HOME/SCAN-Planner/devel/setup.bash"
python3 scripts/validate_maps.py
python3 scripts/configure_experiments.py --project "$HOME/SCAN-Planner"
```

在两个终端分别加载 ROS 和工作空间的环境后，**两边都执行**：

```bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11381
export ROS_IP=127.0.0.1
```

一个终端运行 `roscore -p 11381`；另一个从本仓库根目录运行：

```bash
python3 scripts/run_pilot.py --scene flat --direction up --limit 60
python3 scripts/run_pilot.py --scene ramp --direction up --limit 60
python3 scripts/run_pilot.py --scene stairs --direction up --limit 60
python3 scripts/run_pilot.py --scene stairs --direction down --limit 60
```

每个场景可单独运行；两次之间确认 `rosnode list` 只剩 `/rosout`。输出保存到 `runs/`，含状态、里程计、参数、bag 与日志。异常或超时返回非零退出码。建议各方向重复三次，报告到达次数、最终三维误差、首次出轨迹的时延、规划超限和重试情况。首次出轨迹时延可从 bag 的首条 `/planning/bspline` 和 `/initial_path` 的记录时间计算。下行以反向参考路径测试；起始机体朝向及转向表现需从记录中单独核对。

`scripts/generate_maps.py` 用于重新生成合成地图；常规复现直接使用随仓库提供的点云，并运行校验脚本。仿真使用 `navi_mode=3`，路径发布时扣除回调会加的机体高度，避免重复加高。生成的 launch 文件随本地 checkout 路径变化。

补丁源自迁移基线，修改了上游 `planner_manager.cpp` 的轨迹细化重试与诊断，适用提交如上。上游代码及其修改受 [Apache 2.0](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/LICENSE) 约束。

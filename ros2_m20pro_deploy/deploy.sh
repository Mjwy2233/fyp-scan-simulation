#!/usr/bin/env bash
# ==============================================================================
# SCAN-Planner ROS 2 环境隔离一键构建与启动部署脚本 (deploy.sh)
# 适配实车：云深处 山猫 M20 Pro + Super-LIO (ros2 分支) + GOS 隔离容器 / 外接主机
#
# 功能说明 / Purpose:
#   - 严格保持环境隔离：仅在当前独立 ROS 2 工作空间内执行 `colcon build --symlink-install`
#     与局部 `source`，绝不修改宿主机 `/opt/ros/*`、全局 Python 环境或 `~/.bashrc`。
#   - 支持 `--mode sim`（电脑端一键仿真演示）与 `--mode real`（山猫 M20 Pro 真机部署，含安全锁）。
#   - 支持 `--step step0`（手动扫图建图）、`--step step1`（重定位选点导航）与
#     `--step demo3d`（跨层坡道 3D 导航一键演示）。
#
# 常用运行示例 / Usage Examples:
#   1. 电脑端一键编译并启动 Step 1 仿真（含双雷达融合 + 先验地图 + RViz 2D Goal 选点）:
#        ./deploy.sh --mode sim
#   2. 电脑端一键启动跨层 3D 坡道演示（Mode 3 + map.pcd + 双雷达融合 + RViz）:
#        ./deploy.sh --mode sim --step demo3d
#   3. 电脑端无界面模式运行仿真（用于无显示器环境或自动化验证）:
#        ./deploy.sh --mode sim --no-rviz
#   4. 电脑端演练 Step 0 手动建图 + 双雷达融合 + 录包:
#        ./deploy.sh --mode sim --step step0
#   5. 山猫 M20 Pro 机载 GOS 容器内（推荐配合 taskset -c 4-7 绑大核）仅编译预检:
#        ./deploy.sh --mode real --build-only
#   6. 山猫 M20 Pro 实车运行（导师审批 DEPLOYMENT_PLAN.md 并确认急停就绪后传入 --real-confirm）:
#        taskset -c 4-7 ./deploy.sh --mode real --step step1 \
#          --slam-backend super_lio --enable-fusion false \
#          --pcd /tmp/scan_planner_maps/building_1f_2f.pcd --real-confirm
# ==============================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${SCRIPT_DIR}"

# 默认参数 / Default options
MODE="sim"
STEP="step1"
BUILD_POLICY="auto"       # auto | all | fast | skip
BUILD_ONLY=false
CHECK_ENV_ONLY=false
ENABLE_RVIZ=true
PCD_FILE=""
NAVI_MODE=""
CONTROLLER_MODE=""
CMD_VEL_TOPIC=""
SLAM_BACKEND=""
ENABLE_FUSION=""
REAL_CONFIRM=false
ROS_DISTRO_PREF="${ROS_DISTRO:-humble}"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPL="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

print_help() {
  cat <<'EOF'
Usage: ./deploy.sh [OPTIONS]

Options:
  --mode <sim|real>           运行模式 (默认: sim)
  --step <step0|step1|demo3d> 启动流水线 (默认: step1)
                                step0  = 双雷达融合 + Super-LIO/SLAM 建图 + rosbag2 录制
                                step1  = 先验地图 + 重定位 + RViz Set Goal 导航
                                demo3d = 仿真跨层 3D 坡道演示 (Mode 3 + map.pcd)
  --build                     强制全量构建工作空间所有包
  --fast-build                仅增量构建 scan_planner 主包
  --skip-build                跳过构建直接启动 (需已完成过构建)
  --build-only                仅完成隔离构建后退出，不拉起节点
  --check-env                 仅执行只读环境与依赖自检
  --no-rviz                   不启动 RViz2 图形界面 (无头模式)
  --pcd <path>                指定先验 PCD 地图绝对路径
  --navi-mode <1|3>           覆盖导航模式 (1=2D Goal Pose, 3=3D 跨层路径)
  --controller <closed_loop|open_loop>  覆盖仿真控制器模式
  --cmd-vel <topic>           覆盖底盘速度指令输出话题 (如 /cmd_vel 或 /cmd_vel_dryrun)
  --slam-backend <name>       覆盖真机 SLAM 后端 (默认 super_lio; 可选 m20_builtin / fast_lio / fast_lio_localization / elevator_lio / external)
  --enable-fusion <true|false> 覆盖双雷达融合节点开关 (GOS 直连原厂 /LIDAR/POINTS 时可设 false)
  --domain-id <int>           局部设置 ROS_DOMAIN_ID (山猫 M20 Pro 默认为 0)
  --real-confirm              真机启动安全确认锁 (未传此参数时 --mode real 不会下发控制节点)
  -h, --help                  显示本帮助信息
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)           MODE="$2"; shift 2 ;;
    --step)           STEP="$2"; shift 2 ;;
    --build|--rebuild) BUILD_POLICY="all"; shift ;;
    --fast-build)     BUILD_POLICY="fast"; shift ;;
    --skip-build)     BUILD_POLICY="skip"; shift ;;
    --build-only)     BUILD_ONLY=true; shift ;;
    --check-env)      CHECK_ENV_ONLY=true; shift ;;
    --no-rviz)        ENABLE_RVIZ=false; shift ;;
    --pcd)            PCD_FILE="$2"; shift 2 ;;
    --navi-mode)      NAVI_MODE="$2"; shift 2 ;;
    --controller)     CONTROLLER_MODE="$2"; shift 2 ;;
    --cmd-vel)        CMD_VEL_TOPIC="$2"; shift 2 ;;
    --slam-backend)   SLAM_BACKEND="$2"; shift 2 ;;
    --enable-fusion)  ENABLE_FUSION="$2"; shift 2 ;;
    --domain-id)      DOMAIN_ID="$2"; shift 2 ;;
    --real-confirm)   REAL_CONFIRM=true; shift ;;
    -h|--help)        print_help; exit 0 ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      print_help
      exit 2
      ;;
  esac
done

if [[ "${MODE}" != "sim" && "${MODE}" != "real" ]]; then
  echo "[ERROR] --mode must be 'sim' or 'real' (got '${MODE}')" >&2
  exit 2
fi
if [[ "${STEP}" != "step0" && "${STEP}" != "step1" && "${STEP}" != "demo3d" ]]; then
  echo "[ERROR] --step must be 'step0', 'step1', or 'demo3d' (got '${STEP}')" >&2
  exit 2
fi

# ------------------------------------------------------------------------------
# 1. 局部加载系统 ROS 2 Underlay（不修改 ~/.bashrc 或任何全局系统文件）
# ------------------------------------------------------------------------------
ROS_SETUP="/opt/ros/${ROS_DISTRO_PREF}/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  for distro in humble foxy jazzy iron; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      ROS_DISTRO_PREF="${distro}"
      ROS_SETUP="/opt/ros/${distro}/setup.bash"
      break
    fi
  done
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[ERROR] No ROS 2 installation found under /opt/ros/." >&2
  exit 1
fi

export ROS_DOMAIN_ID="${DOMAIN_ID}"
export RMW_IMPLEMENTATION="${RMW_IMPL}"
export COLCON_LOG_PATH="${WS_ROOT}/log"

# shellcheck disable=SC1090
source "${ROS_SETUP}"

echo "========================================================================"
echo " SCAN-Planner Isolated Deployment Script (Lynx M20 Pro + Super-LIO)"
echo "   Workspace         : ${WS_ROOT}"
echo "   ROS 2 Distro      : ${ROS_DISTRO_PREF}"
echo "   RMW / Domain ID   : ${RMW_IMPLEMENTATION} / ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "   Mode / Step       : mode=${MODE} | step=${STEP} | rviz=${ENABLE_RVIZ}"
echo "========================================================================"

# ------------------------------------------------------------------------------
# 2. 只读依赖检查 / Read-only environment verification
# ------------------------------------------------------------------------------
check_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERROR] Required command '$1' is not available in PATH." >&2
    return 1
  fi
}

check_cmd ros2
check_cmd colcon
check_cmd python3

if [[ "${CHECK_ENV_ONLY}" == "true" ]]; then
  echo "[OK] ROS 2 underlay, colcon, and python3 verified."
  if [[ -f "${WS_ROOT}/install/local_setup.bash" ]]; then
    echo "[OK] Isolated workspace install overlay exists: ${WS_ROOT}/install"
  else
    echo "[INFO] Workspace has not been built yet; run ./deploy.sh --build-only first."
  fi
  exit 0
fi

# ------------------------------------------------------------------------------
# 3. 独立工作空间编译（优先使用 --symlink-install；不支持符号链接时自动回退）
# ------------------------------------------------------------------------------
build_workspace() {
  local scope="$1"
  local symlink_flag="--symlink-install"

  if ln -s "${WS_ROOT}/README.md" "${WS_ROOT}/.symlink_check_tmp" 2>/dev/null; then
    rm -f "${WS_ROOT}/.symlink_check_tmp"
  else
    echo "[WARN] Filesystem does not support symlinks; falling back to standard install."
    symlink_flag=""
  fi

  cd "${WS_ROOT}"
  if [[ "${scope}" == "fast" ]]; then
    echo "[BUILD] Incrementally building package 'scan_planner'..."
    # shellcheck disable=SC2086
    colcon build ${symlink_flag} \
      --packages-select scan_planner \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  else
    echo "[BUILD] Building full isolated workspace..."
    # shellcheck disable=SC2086
    colcon build ${symlink_flag} \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  fi
}

if [[ "${BUILD_POLICY}" == "all" ]]; then
  build_workspace "all"
elif [[ "${BUILD_POLICY}" == "fast" ]]; then
  build_workspace "fast"
elif [[ "${BUILD_POLICY}" == "auto" ]]; then
  if [[ ! -f "${WS_ROOT}/install/local_setup.bash" ]]; then
    build_workspace "all"
  elif [[ ! -x "${WS_ROOT}/install/scan_planner/lib/scan_planner/dual_lidar_fusion_node" ]]; then
    build_workspace "fast"
  else
    echo "[INFO] Isolated workspace already built (use --fast-build or --build to recompile)."
  fi
fi

# shellcheck disable=SC1091
source "${WS_ROOT}/install/local_setup.bash"

if [[ "${BUILD_ONLY}" == "true" ]]; then
  echo "[OK] Isolated build completed without touching global system/Python environments."
  exit 0
fi

# ------------------------------------------------------------------------------
# 4. 真机安全互锁检查 (防止未经审批或急停未就绪时误触发底盘速度输出)
# ------------------------------------------------------------------------------
if [[ "${MODE}" == "real" && "${REAL_CONFIRM}" != "true" ]]; then
  cat <<'EOF'
[SAFETY INTERLOCK] Real-robot mode (--mode real) requires explicit confirmation!
Before running on Lynx M20 Pro hardware:
  1. Confirm advisor has reviewed & approved DEPLOYMENT_PLAN.md.
  2. Confirm this script runs ONLY on GOS (10.21.33.104) isolated Docker/ws or an external Humble host (NEVER AOS/NOS).
  3. Confirm hardware E-stop & remote controller takeover are in hand.
  4. Verify Super-LIO / M20 Pro LiDAR, IMU, odom & /NAV_CMD bridge configs in:
       src/planner/plan_manage/config/dual_lidar_fusion.yaml
       src/planner/plan_manage/config/step0_mapping.yaml
       src/planner/plan_manage/config/step1_navigation.yaml
  5. Re-run with '--real-confirm' when ready on the robot.
EOF
  exit 3
fi

if [[ "${ENABLE_RVIZ}" == "true" && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "[WARN] Neither DISPLAY nor WAYLAND_DISPLAY is set; disabling RViz2 automatically."
  ENABLE_RVIZ=false
fi

# ------------------------------------------------------------------------------
# 5. 根据 --mode 与 --step 启动对应 ROS 2 流水线
# ------------------------------------------------------------------------------
if [[ "${STEP}" == "step0" ]]; then
  LAUNCH_ARGS=(
    "mode:=${MODE}"
    "enable_rviz:=${ENABLE_RVIZ}"
  )
  [[ -n "${SLAM_BACKEND}" ]] && LAUNCH_ARGS+=("slam_backend:=${SLAM_BACKEND}")
  [[ -n "${ENABLE_FUSION}" ]] && LAUNCH_ARGS+=("enable_dual_lidar_fusion:=${ENABLE_FUSION}")
  [[ -n "${PCD_FILE}" ]] && LAUNCH_ARGS+=("sim_pcd_file:=${PCD_FILE}")
  echo "[LAUNCH] ros2 launch scan_planner step0_mapping.launch.py ${LAUNCH_ARGS[*]}"
  exec ros2 launch scan_planner step0_mapping.launch.py "${LAUNCH_ARGS[@]}"

elif [[ "${STEP}" == "demo3d" ]]; then
  DEMO_PCD="${PCD_FILE:-${WS_ROOT}/map.pcd}"
  DEMO_REF="${WS_ROOT}/src/planner/plan_manage/config/reference_path.map.yaml"
  LAUNCH_ARGS=(
    "mode:=sim"
    "navi_mode:=3"
    "controller_mode:=open_loop"
    "pcd_map_file:=${DEMO_PCD}"
    "reference_path_file:=${DEMO_REF}"
    "enable_dual_lidar_fusion:=true"
    "enable_rviz:=${ENABLE_RVIZ}"
  )
  echo "[LAUNCH] ros2 launch scan_planner step1_navigation.launch.py ${LAUNCH_ARGS[*]}"
  exec ros2 launch scan_planner step1_navigation.launch.py "${LAUNCH_ARGS[@]}"

else
  LAUNCH_ARGS=(
    "mode:=${MODE}"
    "enable_rviz:=${ENABLE_RVIZ}"
  )
  [[ -n "${PCD_FILE}" ]] && LAUNCH_ARGS+=("pcd_map_file:=${PCD_FILE}")
  [[ -n "${NAVI_MODE}" ]] && LAUNCH_ARGS+=("navi_mode:=${NAVI_MODE}")
  [[ -n "${CONTROLLER_MODE}" ]] && LAUNCH_ARGS+=("controller_mode:=${CONTROLLER_MODE}")
  [[ -n "${CMD_VEL_TOPIC}" ]] && LAUNCH_ARGS+=("cmd_vel_topic:=${CMD_VEL_TOPIC}")
  [[ -n "${SLAM_BACKEND}" ]] && LAUNCH_ARGS+=("localization_backend:=${SLAM_BACKEND}")
  [[ -n "${ENABLE_FUSION}" ]] && LAUNCH_ARGS+=("enable_dual_lidar_fusion:=${ENABLE_FUSION}")
  echo "[LAUNCH] ros2 launch scan_planner step1_navigation.launch.py ${LAUNCH_ARGS[*]}"
  exec ros2 launch scan_planner step1_navigation.launch.py "${LAUNCH_ARGS[@]}"
fi

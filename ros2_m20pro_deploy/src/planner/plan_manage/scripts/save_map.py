#!/usr/bin/env python3
"""Step 0 一键地图保存与完整性验收触发脚本。

One-click Step 0 map save trigger (calls SLAM map-save service and/or dual_lidar_fusion
`/dual_lidar_fusion/save_pcd` service, then automatically validates the saved `.pcd` file).

功能 / Purpose:
  - 在手动遥控机器狗走完 1 楼到 2 楼楼道后，在另一终端运行本脚本即可一键落盘先验 `.pcd` 地图。
  - 若当前环境中运行了支持 Trigger 存图服务的 SLAM 节点（如 FAST-LIO2 `/map_save`），会优先尝试触发；
    同时触发 `dual_lidar_fusion` 的 `/dual_lidar_fusion/save_pcd` 导出融合累积点云地图。
  - 保存成功后自动调用 `validate_map.py` 校验点数与三维包围盒，确认地图可用于 Step 1 重定位。

调用服务 / Called Services:
  - `--slam-service` (默认 `/map_save`, `std_srvs/srv/Trigger`, 可选)
  - `--fusion-service` (默认 `/dual_lidar_fusion/save_pcd`, `std_srvs/srv/Trigger`)

运行方式 / Usage:
  ros2 run scan_planner save_map.py --pcd /tmp/scan_planner_maps/building_1f_2f.pcd
"""

import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger

# 复用同包内 validate_map.py 的校验函数 / Reuse validate_map helper in same scripts dir
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
from validate_map import validate_map  # noqa: E402


class MapSaverClient(Node):
    """负责调用 SLAM 与 dual_lidar_fusion 存图服务的轻量客户端节点。"""

    def __init__(self):
        super().__init__("step0_save_map_client")

    def call_trigger_service(self, service_name, timeout_sec):
        """等待并调用指定的 std_srvs/srv/Trigger 服务，返回 (called_ok, message)。"""
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, f"Service '{service_name}' not available within {timeout_sec:.1f}s"

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=max(5.0, timeout_sec * 3.0))
        if not future.done():
            return False, f"Service '{service_name}' call timed out"

        response = future.result()
        if response is None:
            return False, f"Service '{service_name}' returned empty response"
        return bool(response.success), response.message


def main():
    """命令行入口：依次触发存图服务并执行自动 PCD 质量校验。"""
    parser = argparse.ArgumentParser(description="One-click Step 0 PCD map saver & validator.")
    parser.add_argument(
        "--slam-service",
        default="/map_save",
        help="Optional SLAM Trigger service name (default: /map_save)",
    )
    parser.add_argument(
        "--fusion-service",
        default="/dual_lidar_fusion/save_pcd",
        help="Dual-LiDAR fusion save_pcd service name (default: /dual_lidar_fusion/save_pcd)",
    )
    parser.add_argument(
        "--pcd",
        default="/tmp/scan_planner_maps/building_1f_2f.pcd",
        help="Expected saved PCD file path for post-save validation",
    )
    parser.add_argument("--timeout", type=float, default=2.0, help="Service wait timeout (sec)")
    parser.add_argument("--min-points", type=int, default=1000, help="Validation min points")
    parser.add_argument("--min-z-span", type=float, default=0.0, help="Validation min Z span (m)")
    parser.add_argument("--skip-validate", action="store_true", help="Skip PCD validation step")
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init(args=sys.argv)
    client = MapSaverClient()
    saved_any = False

    try:
        # 1. 若外部 SLAM 存图服务在线，优先触发 SLAM 导出完整地图
        if args.slam_service:
            ok, msg = client.call_trigger_service(args.slam_service, timeout_sec=args.timeout)
            if ok:
                client.get_logger().info(f"[SLAM save] SUCCESS: {msg}")
                saved_any = True
            else:
                client.get_logger().info(f"[SLAM save] Skipped/Unavailable: {msg}")

        # 2. 触发 dual_lidar_fusion 节点导出融合/累积点云 PCD
        if args.fusion_service:
            ok, msg = client.call_trigger_service(args.fusion_service, timeout_sec=args.timeout)
            if ok:
                client.get_logger().info(f"[Fusion save] SUCCESS: {msg}")
                saved_any = True
            else:
                client.get_logger().warning(f"[Fusion save] Failed/Unavailable: {msg}")
    finally:
        client.destroy_node()
        rclpy.try_shutdown()

    if not saved_any:
        print(
            "[ERROR] Neither SLAM nor dual_lidar_fusion save service succeeded. "
            "Make sure step0_mapping.launch.py is running.",
            file=sys.stderr,
        )
        sys.exit(1)

    if args.skip_validate:
        sys.exit(0)

    if not os.path.isfile(args.pcd):
        print(
            f"[WARN] Trigger succeeded, but expected file '{args.pcd}' was not found. "
            "Check your configured pcd_save_path.",
            file=sys.stderr,
        )
        sys.exit(1)

    passed, _ = validate_map(
        args.pcd, min_points=args.min_points, min_z_span=args.min_z_span
    )
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()

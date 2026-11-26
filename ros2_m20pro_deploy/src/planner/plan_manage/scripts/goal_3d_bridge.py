#!/usr/bin/env python3
"""RViz 3D 选点与 1F-2F 跨楼层折返楼梯路径桥接节点。

Interactive 3D goal & 1F-2F switchback staircase route bridge for SCAN-Planner Mode 3.

功能 / Purpose:
  - 解决原版 `navi_mode:=1` 仅支持同层 2D 目标（Z 固定为初始高度）的局限，在不修改任何
    SCAN-Planner 核心 C++ 算法的前提下，让操作员在 RViz2 中直接选点触发跨楼层 3D 导航：
    1. 使用 RViz2 工具栏 **Publish Point** (`/clicked_point`) 直接点击先验地图的 1 楼或 2 楼任意
       3D 点位（自动携带真实三维 Z 坐标）。
    2. 或使用 **2D Goal Pose** (`/move_base_simple/goal`) 选定平面终点（默认保持当前高度或指定高度）。
  - 当检测到起点与终点高差超过 `floor_change_z_threshold` 时，自动在当前机身位置与目标点之间
    按上楼/下楼方向串接配置的 1F-2F 折返楼梯锚点 (`stair_anchors`)，生成连续 3D `nav_msgs/Path`
    发布至 `/initial_path`，直接驱动原版 `scan_planner_node` (`navi_mode:=3`) 进行局部避障规划！

输入话题 / Subscribed Topics:
  - `body_pose` (`nav_msgs/msg/Odometry`, SensorDataQoS): 当前机器狗全局定位位姿
  - `clicked_point` (默认 `/clicked_point`, `geometry_msgs/msg/PointStamped`): RViz 3D 选点
  - `goal_pose` (默认 `/move_base_simple/goal`, `geometry_msgs/msg/PoseStamped`): RViz 2D Goal Pose

输出话题 / Published Topics:
  - `initial_path` (默认 `/initial_path`, `nav_msgs/msg/Path`, Reliable + Transient Local):
    输出给 SCAN-Planner Mode 3 的三维引导路径

运行方式 / Usage:
  由 `step1_navigation.launch.py` 在 `navi_mode:=3` 且 `enable_3d_goal_bridge:=true` 时自动启动。
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


def parse_anchors(flat_values):
    """将扁平数组 [x0, y0, z0, ...] 解析为三维锚点列表 / Parse flat xyz anchor array."""
    if not flat_values or len(flat_values) % 3 != 0:
        return []
    pts = []
    for i in range(0, len(flat_values), 3):
        pt = (float(flat_values[i]), float(flat_values[i + 1]), float(flat_values[i + 2]))
        if all(math.isfinite(v) for v in pt):
            pts.append(pt)
    return pts


def interpolate_segment(p0, p1, step_m=0.5):
    """按给定间距在两三维点之间均匀插值（不含起点，含终点）/ Interpolate 3D segment."""
    dx, dy, dz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 1e-3:
        return [p1]
    steps = max(1, int(math.ceil(dist / max(0.1, step_m))))
    return [
        (p0[0] + dx * k / steps, p0[1] + dy * k / steps, p0[2] + dz * k / steps)
        for k in range(1, steps + 1)
    ]


class Goal3DBridge(Node):
    """监听 RViz 3D/2D 选点并构造连续 3D /initial_path 引导路径的 ROS 2 节点。"""

    def __init__(self):
        super().__init__("goal_3d_bridge")
        self.frame_id = self.declare_parameter("frame_id", "world").value
        self.body_height = float(self.declare_parameter("body_height", 0.4).value)
        self.floor_change_z_threshold = float(
            self.declare_parameter("floor_change_z_threshold", 0.8).value
        )
        self.waypoint_spacing = float(self.declare_parameter("waypoint_spacing", 0.5).value)
        self.accept_2d_goal = bool(self.declare_parameter("accept_2d_goal", True).value)
        raw_anchors = self.declare_parameter("stair_anchors", [0.0]).value
        self.stair_anchors = parse_anchors(raw_anchors if len(raw_anchors) >= 3 else [])

        self.latest_body_pos = None

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, "initial_path", path_qos)
        self.odom_sub = self.create_subscription(
            Odometry, "body_pose", self.odom_callback, qos_profile_sensor_data
        )
        self.clicked_sub = self.create_subscription(
            PointStamped, "clicked_point", self.clicked_point_callback, 10
        )
        if self.accept_2d_goal:
            self.goal_2d_sub = self.create_subscription(
                PoseStamped, "goal_pose", self.goal_2d_callback, 10
            )

        self.get_logger().info(
            f"Goal3DBridge ready (frame='{self.frame_id}', stair_anchors={len(self.stair_anchors)}, "
            f"floor_z_thresh={self.floor_change_z_threshold:.2f}m). "
            "Use RViz 'Publish Point' for 3D multi-floor goals or '2D Goal Pose' for planar goals."
        )

    def odom_callback(self, msg):
        """记录当前机器狗机身三维位置 / Cache current robot body position."""
        p = msg.pose.pose.position
        self.latest_body_pos = (float(p.x), float(p.y), float(p.z))

    def clicked_point_callback(self, msg):
        """响应 RViz Publish Point (/clicked_point)：以真实点击的 3D 地面坐标作为终点。"""
        goal_ground = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        self.publish_connected_path(goal_ground, source="RViz 3D PublishPoint")

    def goal_2d_callback(self, msg):
        """响应 RViz 2D Goal Pose (/move_base_simple/goal)：默认沿当前地面高度生成路径。"""
        if self.latest_body_pos is None:
            self.get_logger().warning("Ignored 2D goal: waiting for body_pose odometry")
            return
        cur_ground_z = self.latest_body_pos[2] - self.body_height
        # 若消息本身携带显著非零 Z（例如命令行手动下发 3D PoseStamped），保留该 Z
        goal_z = float(msg.pose.position.z) if abs(msg.pose.position.z) > 1e-3 else cur_ground_z
        goal_ground = (float(msg.pose.position.x), float(msg.pose.position.y), goal_z)
        self.publish_connected_path(goal_ground, source="RViz 2D GoalPose")

    def publish_connected_path(self, goal_ground, source):
        """从当前地面位置出发，按需串接 1F-2F 楼梯折返锚点并发布 /initial_path。"""
        if self.latest_body_pos is None:
            self.get_logger().warning(f"Ignored {source}: no body_pose received yet")
            return

        # SCAN-Planner Mode 3 会在 /initial_path 的 z 坐标上自动叠加 body_height，故起点换算为地面高度
        start_ground = (
            self.latest_body_pos[0],
            self.latest_body_pos[1],
            self.latest_body_pos[2] - self.body_height,
        )
        dz = goal_ground[2] - start_ground[2]

        control_nodes = [start_ground]
        if abs(dz) >= self.floor_change_z_threshold and len(self.stair_anchors) >= 2:
            # 上楼按锚点正序，下楼按锚点逆序，仅保留高程位于起终点之间的楼梯锚点
            ordered = list(self.stair_anchors) if dz > 0.0 else list(reversed(self.stair_anchors))
            z_lo = min(start_ground[2], goal_ground[2]) - 0.25
            z_hi = max(start_ground[2], goal_ground[2]) + 0.25
            filtered = [pt for pt in ordered if z_lo <= pt[2] <= z_hi]
            control_nodes.extend(filtered)
            self.get_logger().info(
                f"[{source}] Cross-floor goal detected (dz={dz:+.2f}m); "
                f"routing through {len(filtered)} stair anchor(s)"
            )
        control_nodes.append(goal_ground)

        # 生成连续均匀插值的三维引导路径
        dense_points = [control_nodes[0]]
        for idx in range(len(control_nodes) - 1):
            dense_points.extend(
                interpolate_segment(control_nodes[idx], control_nodes[idx + 1], self.waypoint_spacing)
            )

        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, z in dense_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f"[{source}] Published /initial_path ({len(path_msg.poses)} pts): "
            f"({start_ground[0]:.2f}, {start_ground[1]:.2f}, {start_ground[2]:.2f}) -> "
            f"({goal_ground[0]:.2f}, {goal_ground[1]:.2f}, {goal_ground[2]:.2f})"
        )


def main(args=None):
    """节点入口函数 / ROS 2 node entry point."""
    rclpy.init(args=args)
    node = Goal3DBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

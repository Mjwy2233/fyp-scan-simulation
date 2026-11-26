#!/usr/bin/env python3
"""3D 参考路径一次性发布节点：在里程计与规划器订阅者就绪后向 /initial_path 发布全局参考路径。

Publish one configured 3D reference path after odometry and planner subscriber are ready.

功能 / Purpose:
  - 配合 SCAN-Planner `navi_mode:=3`（参考路径引导模式），在 1F-2F 跨楼层/坡道/楼梯
    场景下发布预先规划或录制的 3D 地面参考路径。
  - 使用 Transient Local QoS 发布一次，避免重复触发规划器重初始化。

输入话题 / Subscribed Topics:
  - `body_pose` (`nav_msgs/msg/Odometry`, SensorDataQoS): 等待首帧机身位姿就绪

输出话题 / Published Topics:
  - `initial_path` (`nav_msgs/msg/Path`, Reliable + Transient Local): 三维参考路径

ROS 2 参数 / Parameters:
  - `frame_id` (string, 默认 `world`): 路径所属全局坐标系
  - `waypoints` (double[], 必填): 扁平化 `[x0, y0, z0, x1, y1, z1, ...]`，至少 2 个三维点
  - `publish_delay_sec` (double, 默认 `0.5`): 收到首帧里程计后的等待时间(s)

运行方式 / Usage:
  ros2 run scan_planner reference_path_publisher.py --ros-args \
    --params-file config/reference_path.map.yaml \
    -r body_pose:=/quad_0/body_pose -r initial_path:=/initial_path
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


def parse_waypoints(values):
    """校验并将扁平数组解析为 (x, y, z) 元组列表 / Validate and group flat xyz array."""
    if values is None or len(values) < 6 or len(values) % 3 != 0:
        raise ValueError("waypoints must contain at least two x,y,z triples")
    numbers = [float(value) for value in values]
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError("waypoints must contain only finite values")
    return [tuple(numbers[index : index + 3]) for index in range(0, len(numbers), 3)]


def build_path_message(waypoints, frame_id, stamp):
    """将 (x, y, z) 点序列封装为 nav_msgs/Path 消息 / Build ROS 2 Path message."""
    message = Path()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    for x, y, z in waypoints:
        pose = PoseStamped()
        message.poses.append(pose)
        pose.header = message.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
    return message


class ReferencePathPublisher(Node):
    """等待里程计就绪与下游订阅建立后，单次发布 3D 全局参考路径的 ROS 2 节点。"""

    def __init__(self):
        super().__init__("reference_path_publisher")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("waypoints", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("publish_delay_sec", 0.5)

        self.frame_id = self.get_parameter("frame_id").value
        self.waypoints = parse_waypoints(self.get_parameter("waypoints").value)
        self.publish_delay_sec = float(self.get_parameter("publish_delay_sec").value)
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        if not math.isfinite(self.publish_delay_sec) or self.publish_delay_sec < 0.0:
            raise ValueError("publish_delay_sec must be finite and non-negative")

        # 使用 Transient Local 持久性，确保晚启动的订阅者（如 RViz）仍可获取该路径
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(Path, "initial_path", path_qos)
        self.odom_subscription = self.create_subscription(
            Odometry, "body_pose", self.odom_callback, qos_profile_sensor_data
        )
        self.first_odom_time = None
        self.published = False
        self.timer = self.create_timer(0.05, self.try_publish)
        self.get_logger().info(
            f"Waiting to publish {len(self.waypoints)} reference-path points"
        )

    def odom_callback(self, _message):
        """记录首帧机身里程计到达时刻 / Record arrival time of first odometry frame."""
        if self.first_odom_time is None:
            self.first_odom_time = self.get_clock().now()

    def try_publish(self):
        """检查里程计、订阅者与延时条件均满足后发布一次路径并停止定时器。"""
        if self.published or self.first_odom_time is None:
            return
        if self.path_publisher.get_subscription_count() == 0:
            return
        elapsed = (self.get_clock().now() - self.first_odom_time).nanoseconds / 1e9
        if elapsed < self.publish_delay_sec:
            return

        message = build_path_message(
            self.waypoints, self.frame_id, self.get_clock().now().to_msg()
        )
        self.path_publisher.publish(message)
        self.published = True
        self.timer.cancel()
        self.get_logger().info(
            f"Published reference path on initial_path with {len(message.poses)} points"
        )


def main(args=None):
    """节点入口函数 / ROS 2 node entry point."""
    rclpy.init(args=args)
    node = None
    try:
        node = ReferencePathPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            rclpy.logging.get_logger("reference_path_publisher").error(str(error))
        if isinstance(error, ValueError):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

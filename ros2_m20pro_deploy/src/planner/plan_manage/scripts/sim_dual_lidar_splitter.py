#!/usr/bin/env python3
"""仿真双激光雷达点云切分适配器：将仿真全局点云切分为前/后雷达局部点云供融合节点闭环验证。

Simulation dual-LiDAR splitter adapter for verifying `dual_lidar_fusion_node` end-to-end
in PC simulation without real hardware.

功能 / Purpose:
  - 订阅仿真器发布的全局系局部感知点云 (`/quad_0/cloud`) 与机身位姿 (`/quad_0/body_pose`)。
  - 将点云先反变换至机器狗本体坐标系 (`base_link`)，按前/后半球（带少量视场重叠）拆分为两路，
    并进一步按 YAML 外参逆变换至前雷达系 (`lidar_front_link`) 与后雷达系 (`lidar_rear_link`) 发布。
  - 同时发布静态恒等变换 `base -> base_link`，衔接仿真 `world -> base` TF 树，从而在电脑端
    100% 复现真机“前/后双雷达原始点云 -> TF 变换与时间同步融合 -> SLAM/SCAN-Planner”的完整链路。

输入话题 / Subscribed Topics:
  - `sim_cloud` (默认 `/quad_0/cloud`, `sensor_msgs/msg/PointCloud2`, frame_id=`world`)
  - `body_pose` (默认 `/quad_0/body_pose`, `nav_msgs/msg/Odometry`)

输出话题 / Published Topics:
  - `front_topic` (默认 `/lidar_front/points`, `sensor_msgs/msg/PointCloud2`, frame_id=`lidar_front_link`)
  - `rear_topic`  (默认 `/lidar_rear/points`,  `sensor_msgs/msg/PointCloud2`, frame_id=`lidar_rear_link`)
  - `/tf_static` (`sim_base_frame` -> `target_frame`, 默认 `base` -> `base_link`)

运行方式 / Usage:
  由 `step0_mapping.launch.py` 或 `step1_navigation.launch.py` 在 `mode:=sim` 下自动拉起。
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


def quat_to_rot_matrix(qx, qy, qz, qw):
    """四元数 (x, y, z, w) 转 3x3 旋转矩阵 / Quaternion to 3x3 rotation matrix."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def rpy_to_rot_matrix(roll, pitch, yaw):
    """欧拉角 Z-Y-X (roll, pitch, yaw) 转 3x3 旋转矩阵 / RPY to 3x3 rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def pack_xyz_cloud(points_nx3, frame_id, stamp):
    """将 Nx3 float32 NumPy 数组快速打包为 sensor_msgs/PointCloud2 消息。"""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(points_nx3.shape[0])
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = np.ascontiguousarray(points_nx3, dtype="<f4").tobytes()
    return msg


class SimDualLidarSplitter(Node):
    """将仿真局部点云切分为前/后两路传感器坐标系点云的测试适配节点。"""

    def __init__(self):
        super().__init__("sim_dual_lidar_splitter")
        self.sim_cloud_topic = self.declare_parameter("sim_cloud_topic", "/quad_0/cloud").value
        self.body_pose_topic = self.declare_parameter("body_pose_topic", "/quad_0/body_pose").value
        self.front_topic = self.declare_parameter("front_topic", "/lidar_front/points").value
        self.rear_topic = self.declare_parameter("rear_topic", "/lidar_rear/points").value
        self.front_frame_id = self.declare_parameter("front_frame_id", "lidar_front_link").value
        self.rear_frame_id = self.declare_parameter("rear_frame_id", "lidar_rear_link").value
        self.sim_base_frame = self.declare_parameter("sim_base_frame", "base").value
        self.target_frame = self.declare_parameter("target_frame", "base_link").value
        self.overlap_x = float(self.declare_parameter("overlap_x", 0.25).value)

        front_ext = self.declare_parameter(
            "front_extrinsic", [0.32028, 0.0, -0.013, 0.0, 0.0, 0.0]
        ).value
        rear_ext = self.declare_parameter(
            "rear_extrinsic", [-0.32028, 0.0, -0.013, 0.0, 0.0, math.pi]
        ).value

        # 预计算 base_link -> sensor 的逆变换，确保经 dual_lidar_fusion 还原后无几何误差
        self.front_r_inv, self.front_t = self._make_inverse_extrinsic(front_ext)
        self.rear_r_inv, self.rear_t = self._make_inverse_extrinsic(rear_ext)

        # 若仿真 TF 使用 base 而融合节点使用 base_link，广播恒等静态 TF 连接二者
        if self.sim_base_frame != self.target_frame:
            self.static_tf = StaticTransformBroadcaster(self)
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = self.sim_base_frame
            tf_msg.child_frame_id = self.target_frame
            tf_msg.transform.rotation.w = 1.0
            self.static_tf.sendTransform(tf_msg)

        self.latest_pos = None
        self.latest_rot = None

        self.front_pub = self.create_publisher(PointCloud2, self.front_topic, qos_profile_sensor_data)
        self.rear_pub = self.create_publisher(PointCloud2, self.rear_topic, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, self.body_pose_topic, self.odom_callback, qos_profile_sensor_data
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.sim_cloud_topic, self.cloud_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"SimDualLidarSplitter ready: {self.sim_cloud_topic} -> "
            f"{self.front_topic} ({self.front_frame_id}) + {self.rear_topic} ({self.rear_frame_id})"
        )

    @staticmethod
    def _make_inverse_extrinsic(ext):
        """从 [x, y, z, r, p, y] 构造 sensor<-base 的旋转转置与平移向量。"""
        t = np.array([ext[0], ext[1], ext[2]], dtype=np.float64)
        rot = rpy_to_rot_matrix(ext[3], ext[4], ext[5])
        return rot.T, t

    def odom_callback(self, msg):
        """缓存仿真机身位姿用于 world -> base_link 反变换。"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        self.latest_rot = quat_to_rot_matrix(q.x, q.y, q.z, q.w)

    def cloud_callback(self, msg):
        """解包仿真点云、按前后半球切分并分别变换至前/后雷达局部坐标系发布。"""
        if self.latest_pos is None or self.latest_rot is None:
            return
        total = int(msg.width) * int(msg.height)
        if total == 0:
            return

        # 提取 x, y, z 字节偏移量，零拷贝视图解析为 Nx3 点集
        field_offsets = {f.name: f.offset for f in msg.fields}
        if not all(k in field_offsets for k in ("x", "y", "z")):
            return
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": ["<f4", "<f4", "<f4"],
                "offsets": [field_offsets["x"], field_offsets["y"], field_offsets["z"]],
                "itemsize": msg.point_step,
            }
        )
        raw = np.frombuffer(msg.data, dtype=dtype, count=total)
        pts_world = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float64)
        valid = np.all(np.isfinite(pts_world), axis=1)
        pts_world = pts_world[valid]
        if pts_world.shape[0] == 0:
            return

        # world -> base_link: p_b = R_wb^T * (p_w - t_wb)
        pts_body = (pts_world - self.latest_pos) @ self.latest_rot

        # 按本体 X 轴切分为前/后两路（保留 overlap_x 重叠区模拟真实 360°/半球重叠视场）
        front_mask = pts_body[:, 0] >= -self.overlap_x
        rear_mask = pts_body[:, 0] <= self.overlap_x

        pts_front_body = pts_body[front_mask]
        pts_rear_body = pts_body[rear_mask]

        # base_link -> lidar_front_link / lidar_rear_link: p_s = R_bs^T * (p_b - t_bs)
        pts_front_sensor = (pts_front_body - self.front_t) @ self.front_r_inv.T
        pts_rear_sensor = (pts_rear_body - self.rear_t) @ self.rear_r_inv.T

        stamp = msg.header.stamp
        self.front_pub.publish(pack_xyz_cloud(pts_front_sensor, self.front_frame_id, stamp))
        self.rear_pub.publish(pack_xyz_cloud(pts_rear_sensor, self.rear_frame_id, stamp))


def main(args=None):
    """节点入口函数 / ROS 2 node entry point."""
    rclpy.init(args=args)
    node = SimDualLidarSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

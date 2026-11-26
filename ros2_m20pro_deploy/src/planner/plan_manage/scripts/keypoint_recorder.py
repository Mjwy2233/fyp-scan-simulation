#!/usr/bin/env python3
"""交互式航点记录工具：实时订阅机器狗里程计并将关键位置保存为 SCAN-Planner Mode 2 参数 YAML。

Interactive keypoint recorder for SCAN-Planner Mode 2 (preset waypoint sequence).

功能 / Purpose:
  - 在手动遥控机器狗走过 1F-2F 走廊或楼梯时，通过键盘快捷记录当前 3D 位姿坐标。
  - 输出符合 ROS 2 参数格式的 YAML 文件 (`fsm.waypoints: [x0, y0, z0, ...]`)，
    可直接传递给 `run.launch.py navi_mode:=2 keypoints_file:=<path>`。

输入话题 / Subscribed Topics:
  - `--odom` (默认 `/LIO/odom_vehicle`, `nav_msgs/msg/Odometry`): 机器狗当前定位里程计

输出文件 / Output File:
  - `--output` (默认 `./keypoints.yaml`): ROS 2 参数格式航点 YAML 文件

运行方式 / Usage:
  ros2 run scan_planner keypoint_recorder.py --odom /LIO/odom_vehicle --output keypoints.yaml
  (按键说明: Enter/Space/a=记录当前点, r=替换, d=删除, u=撤销, l=列出, s=保存, q=保存并退出)
"""

import argparse
import math
import os
import select
import sys
import termios
import tty

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args


DEFAULT_OUTPUT = os.path.join(os.getcwd(), "keypoints.yaml")


def format_float(value):
    """格式化浮点数并去除尾部多余的 0 / Format float concisely without trailing zeros."""
    text = f"{float(value):.6f}".rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def atomic_write(path, content):
    """原子写文件：先写临时文件再 rename，防止中断导致 YAML 损坏 / Atomic file write."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        handle.write(content)
    os.replace(temporary, path)


class KeypointRecorder(Node):
    """监听里程计并提供终端交互式航点增删改存的 ROS 2 节点。"""

    def __init__(self, odom_topic, output_path):
        super().__init__("keypoint_recorder")
        self.odom_topic = odom_topic
        self.output_path = os.path.abspath(output_path)
        self.latest_odom = None
        self.waypoints = []
        # 使用 SensorDataQoS (Best Effort) 兼容各类 LIO/SLAM 里程计发布端
        self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)

    def odom_callback(self, message):
        """缓存最新一帧里程计消息 / Cache latest odometry message."""
        self.latest_odom = message

    def current_point(self):
        """提取当前有效的三维坐标 (x, y, z) / Extract valid finite (x, y, z) from odometry."""
        if self.latest_odom is None:
            return None
        position = self.latest_odom.pose.pose.position
        point = (float(position.x), float(position.y), float(position.z))
        return point if all(math.isfinite(value) for value in point) else None

    def record_current(self):
        """追加当前机身位置到航点列表 / Append current robot position to waypoint list."""
        point = self.current_point()
        if point is None:
            self.get_logger().warning("No valid odometry received yet")
            return
        self.waypoints.append(point)
        self.get_logger().info(f"Recorded waypoint {len(self.waypoints)}: {point}")

    def replace_current(self, index):
        """用当前机身位置替换指定编号(1-based)的航点 / Replace waypoint at 1-based index."""
        point = self.current_point()
        if point is None or not 1 <= index <= len(self.waypoints):
            self.get_logger().warning("Invalid waypoint index or odometry")
            return
        self.waypoints[index - 1] = point

    def delete_waypoint(self, index):
        """删除指定编号(1-based)的航点 / Delete waypoint at 1-based index."""
        if 1 <= index <= len(self.waypoints):
            self.waypoints.pop(index - 1)
        else:
            self.get_logger().warning("Invalid waypoint index")

    def build_yaml(self):
        """生成供 scan_planner_node 直接加载的扁平化 ROS 2 参数 YAML 内容。"""
        values = [format_float(value) for point in self.waypoints for value in point]
        return (
            "scan_planner_node:\n"
            "  ros__parameters:\n"
            f"    fsm.waypoints: [{', '.join(values)}]\n"
        )

    def save(self):
        """将当前航点列表原子保存至目标 YAML 路径 / Persist waypoints to YAML file."""
        atomic_write(self.output_path, self.build_yaml())
        self.get_logger().info(f"Saved {len(self.waypoints)} waypoint(s) to {self.output_path}")

    def prompt_index(self, settings, action):
        """临时恢复终端行缓冲模式以读取用户输入编号 / Prompt user for a 1-based index."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            return int(input(f"{action} waypoint index (1-{len(self.waypoints)}): ").strip())
        except ValueError:
            return -1
        finally:
            tty.setcbreak(sys.stdin.fileno())

    def run(self):
        """主循环：非阻塞轮询 ROS 回调与终端单键输入 / Main loop for ROS spin + key input."""
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard control requires a TTY")
        print("Enter/Space/a: add, r: replace, d: delete, u: undo, l: list, s: save, q: save+quit")
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                if not ready:
                    continue
                key = sys.stdin.read(1).lower()
                if key in ("\n", "\r", " ", "a"):
                    self.record_current()
                elif key == "r":
                    self.replace_current(self.prompt_index(settings, "Replace"))
                elif key == "d":
                    self.delete_waypoint(self.prompt_index(settings, "Delete"))
                elif key == "u" and self.waypoints:
                    self.waypoints.pop()
                elif key == "l":
                    print("\n".join(f"{i}: {point}" for i, point in enumerate(self.waypoints, 1)))
                elif key == "s":
                    self.save()
                elif key in ("q", "\x03"):
                    self.save()
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    """命令行入口：解析参数并启动航点记录节点 / CLI entry point."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom", default="/LIO/odom_vehicle")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init(args=sys.argv)
    recorder = KeypointRecorder(arguments.odom, arguments.output)
    try:
        recorder.run()
    except (KeyboardInterrupt, RuntimeError) as error:
        recorder.get_logger().error(str(error))
    finally:
        recorder.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

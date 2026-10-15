#!/usr/bin/env python3
"""诊断用：每秒打印 /odom 位置与 /cmd_vel 速度，判断车是否真的在动。"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Probe(Node):
    def __init__(self):
        super().__init__("probe")
        self.odom = None
        self.cmd = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(1.0, self.on_timer)

    def on_odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y)

    def on_timer(self):
        print(f"odom=({self.odom}) cmd=({self.cmd})")


def main():
    rclpy.init()
    node = Probe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

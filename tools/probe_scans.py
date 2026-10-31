#!/usr/bin/env python3
# 临时诊断：订阅 5 层激光，打印桩/箱子方向的角度-距离
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

TOPICS = ["/scan_low1", "/scan_low2", "/scan_mid", "/scan_high1", "/scan_high2"]
DEG = {"下桩45": 45.0, "箱46": 46.0, "箱43": 43.0, "上桩66": 66.0, "上桩66.8": 66.8,
       "上桩67.5": 67.5, "上桩68": 68.0}

class Probe(Node):
    def __init__(self):
        super().__init__("probe")
        self.msgs = {}
        for t in TOPICS:
            self.create_subscription(LaserScan, t, lambda m, t=t: self.msgs.update({t: m}), 10)
        self.timer = self.create_timer(3.0, self.dump)

    def dump(self):
        if len(self.msgs) < len(TOPICS):
            print("waiting...", len(self.msgs))
            return
        for t in TOPICS:
            m = self.msgs[t]
            n = len(m.ranges)
            inc = 2 * math.pi / n
            out = []
            for name, deg in DEG.items():
                i = int((math.radians(deg) + math.pi) / inc) % n
                r = m.ranges[i]
                out.append(f"{name}={r:.2f}" if r < 100 else f"{name}=inf")
            print(t, " | ".join(out))

rclpy.init()
node = Probe()
rclpy.spin(node)

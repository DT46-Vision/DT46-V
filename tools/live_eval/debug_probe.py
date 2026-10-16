#!/usr/bin/env python3
"""采集 /detector/armors_debug_info 的四角点 -> <前缀>_debug.csv（离线复算 PnP / 分支用）。

用法: ROS_LOG_DIR=<可写> FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 debug_probe.py <秒> <前缀>
"""
import sys, time, csv
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rm_interfaces.msg import ArmorsDebugMsg

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
PREFIX = sys.argv[2] if len(sys.argv) > 2 else "dbg"

class P(Node):
    def __init__(self):
        super().__init__("dbg_probe"); self.rows = []
        self.create_subscription(ArmorsDebugMsg, "/detector/armors_debug_info", self.cb, qos_profile_sensor_data)
    def cb(self, msg):
        s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for a in msg.armors_debug:
            self.rows.append([time.time(), s, a.armor_id, a.color,
                              a.l_light_up_dx, a.l_light_up_dy, a.l_light_down_dx, a.l_light_down_dy,
                              a.r_light_up_dx, a.r_light_up_dy, a.r_light_down_dx, a.r_light_down_dy])

rclpy.init(); n = P()
end = time.time() + DUR
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node(); rclpy.shutdown()
out = f"{PREFIX}_debug.csv"
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_recv", "stamp", "armor_id", "color", "l1u_x", "l1u_y", "l1d_x", "l1d_y", "l2u_x", "l2u_y", "l2d_x", "l2d_y"])
    w.writerows(n.rows)
print("saved", out, "rows", len(n.rows))

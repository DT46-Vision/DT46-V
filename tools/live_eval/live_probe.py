#!/usr/bin/env python3
"""实车采集：/detector/armors_info + /imu/rpy + /tracker/gimbal_control -> 3 个 CSV。

用法:  ROS_LOG_DIR=<可写目录> FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 live_probe.py <秒> <前缀>
注意:  在容器/沙箱里必须设 FASTDDS_BUILTIN_TRANSPORTS=UDPv4，否则共享内存被挡、订阅收不到数据。
"""
import sys, time, csv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Vector3Stamped
from rm_interfaces.msg import ArmorsMsg, GimbalControl

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
PREFIX = sys.argv[2] if len(sys.argv) > 2 else "probe"

def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

def sensor_qos():
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)

class Probe(Node):
    def __init__(self):
        super().__init__("live_probe")
        self.armors, self.imu, self.gimbal = [], [], []
        self.create_subscription(ArmorsMsg, "/detector/armors_info", self.cb_armors, sensor_qos())
        self.create_subscription(Vector3Stamped, "/imu/rpy", self.cb_imu, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(GimbalControl, "/tracker/gimbal_control", self.cb_gimbal, sensor_qos())

    def cb_armors(self, msg):
        now = time.time(); s = stamp_s(msg)
        for a in msg.armors:
            self.armors.append([now, s, a.armor_id, a.dx, a.dy, a.dz, a.yaw])

    def cb_imu(self, msg):
        self.imu.append([time.time(), stamp_s(msg), msg.vector.x, msg.vector.y, msg.vector.z])

    def cb_gimbal(self, msg):
        self.gimbal.append([time.time(), stamp_s(msg), msg.yaw, msg.pitch, msg.can_fire])

def write(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(header); w.writerows(rows)
    print("[file]", path, "rows", len(rows))

rclpy.init(); n = Probe()
print("[probe] running for %.0fs ..." % DUR)
end = time.time() + DUR
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node(); rclpy.shutdown()

write(f"{PREFIX}armors.csv", ["t_recv", "stamp", "armor_id", "dx", "dy", "dz", "yaw"], n.armors)
write(f"{PREFIX}imu.csv", ["t_recv", "stamp", "roll", "pitch", "yaw"], n.imu)
write(f"{PREFIX}gimbal.csv", ["t_recv", "stamp", "yaw", "pitch", "can_fire"], n.gimbal)

# 现场判断用的摘要
if len(n.armors) > 1:
    st = [r[1] for r in n.armors]
    dt = [b - a for a, b in zip(st, st[1:])]
    dt.sort()
    print("[armors] n=%d  dt p50=%.1fms p95=%.1fms max=%.1fms  >30ms=%.1f%%" % (
        len(st), dt[len(dt)//2]*1000, dt[int(len(dt)*0.95)]*1000, dt[-1]*1000,
        100.0*sum(1 for d in dt if d > 0.03)/len(dt)))
    ids = {}
    for r in n.armors: ids[r[2]] = ids.get(r[2], 0) + 1
    print("[armors] 各 id 帧数:", ids)
if len(n.gimbal) > 1:
    lat = [r[0] - r[1] for r in n.gimbal]
    print("[gimbal] n=%d  recv-stamp 延迟 mean=%.2fms  can_fire 帧数=%d" % (
        len(lat), 1000*sum(lat)/len(lat), sum(1 for r in n.gimbal if r[4])))
if len(n.imu) > 1:
    st = [r[1] for r in n.imu]; dt = [b - a for a, b in zip(st, st[1:])]
    print("[imu] n=%d  平均周期=%.1fms (≈%.0fHz)" % (len(st), 1000*sum(dt)/len(dt), 1.0/(sum(dt)/len(dt))))

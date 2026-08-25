import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from rm_interfaces.msg import EnemyCenter


class EnemyTfNode(Node):

    def __init__(self):
        super().__init__('enemy_tf_node')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.pub_pose = self.create_publisher(
            PoseStamped, '/tracker/enemy_pose', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            EnemyCenter, '/tracker/enemy_datas', self.enemy_cb, qos)

        self.get_logger().info('enemy_tf_node started')
    # 回调函数
    def enemy_cb(self, msg: EnemyCenter):
        if not msg.tracked:
            return

        now = self.get_clock().now().to_msg()

        # ---- tf: map -> enemy ----
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'camera'
        t.child_frame_id = 'enemy'
        t.transform.translation.x = float(msg.x)
        t.transform.translation.y = float(msg.y)
        t.transform.translation.z = float(msg.z)
        q = self.yaw_to_quaternion(msg.enemy_yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # ---- PoseStamped: /tracker/enemy_pose ----
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'camera'
        pose.pose.position.x = float(msg.x)
        pose.pose.position.y = float(msg.y)
        pose.pose.position.z = float(msg.z)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.pub_pose.publish(pose)

    @staticmethod
    def yaw_to_quaternion(yaw: float):
        """yaw (rad) -> (x, y, z, w) 四元数，绕 z 轴旋转"""
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main(args=None):
    rclpy.init(args=args)
    node = EnemyTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

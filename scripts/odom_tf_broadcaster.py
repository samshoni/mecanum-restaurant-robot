#!/usr/bin/env python3
"""
Subscribes to /model/mecanum_bot/odometry (activates the lazy ros_gz_bridge),
republishes as /odom, and broadcasts the odom→base_footprint TF transform.

Combining /odom publishing and TF broadcasting in one node avoids the fragile
/model/mecanum_bot/tf bridge + ign_tf_relay approach that was failing due to
ros_gz_bridge lazy-activation timing. The odometry message already contains
everything needed for the TF (position + orientation of base_footprint in odom).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

RELIABLE_QOS = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
BEST_EFFORT_QOS = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

IGN_TOPIC = '/model/mecanum_bot/odometry'


class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self._received = False
        self._pub = self.create_publisher(Odometry, '/odom', RELIABLE_QOS)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Subscribe with both QoS profiles — bridge may publish with either
        self.create_subscription(Odometry, IGN_TOPIC, self._cb, RELIABLE_QOS)
        self.create_subscription(Odometry, IGN_TOPIC, self._cb, BEST_EFFORT_QOS)
        self.create_timer(5.0, self._watchdog)
        self.get_logger().info(
            f'odom_tf_broadcaster ready — {IGN_TOPIC} → /odom + odom→base_footprint TF')

    def _watchdog(self):
        if not self._received:
            self.get_logger().warn(
                f'odom_tf_broadcaster: still waiting for data on {IGN_TOPIC} '
                '— check that Gazebo is running and ros_gz_bridge is up')

    def _cb(self, msg: Odometry):
        if not self._received:
            self._received = True
            self.get_logger().info(
                'odom_tf_broadcaster: first odometry received — /odom and TF live')

        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        # Publish /odom topic for Nav2 and other consumers
        self._pub.publish(msg)

        # Broadcast odom → base_footprint TF from odometry pose
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomTFBroadcaster())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

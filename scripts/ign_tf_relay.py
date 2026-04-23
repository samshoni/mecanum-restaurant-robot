#!/usr/bin/env python3
"""
Subscribes to /model/mecanum_bot/tf (actual Ignition bridge topic, no remapping)
so the lazy bridge activates, then republishes to /tf with guaranteed correct
frame IDs: odom → base_footprint.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage

RELIABLE_QOS = QoSProfile(depth=100,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE)
BEST_EFFORT_QOS = QoSProfile(depth=100,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)

IGN_TOPIC = '/model/mecanum_bot/tf'


class IgnTFRelay(Node):
    def __init__(self):
        super().__init__('ign_tf_relay')
        self._received = False
        self._pub = self.create_publisher(TFMessage, '/tf', RELIABLE_QOS)
        # Subscribe with BOTH QoS profiles — bridge may use either
        self.create_subscription(TFMessage, IGN_TOPIC, self._cb, RELIABLE_QOS)
        self.create_subscription(TFMessage, IGN_TOPIC, self._cb, BEST_EFFORT_QOS)
        # Diagnostic: warn every 5 s until first message
        self.create_timer(5.0, self._watchdog)
        self.get_logger().info(f'ign_tf_relay ready — {IGN_TOPIC} → /tf')

    def _watchdog(self):
        if not self._received:
            self.get_logger().warn(
                f'ign_tf_relay: still waiting for data on {IGN_TOPIC} '
                '— bridge may not have activated yet')

    def _cb(self, msg: TFMessage):
        if not self._received:
            self._received = True
            raw_ids = [(t.header.frame_id, t.child_frame_id) for t in msg.transforms]
            self.get_logger().info(
                f'ign_tf_relay: first TF received, raw frame IDs: {raw_ids}')
        out = TFMessage()
        for t in msg.transforms:
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            out.transforms.append(t)
        self._pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(IgnTFRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

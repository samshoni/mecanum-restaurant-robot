#!/usr/bin/env python3
"""
Subscribes to /ign_clock (raw Ignition sim clock bridged from Gazebo) and
republishes to /clock only when time moves forward — preventing backward
jumps from clearing SLAM's TF buffer.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock

RELIABLE_QOS = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
BEST_EFFORT_QOS = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


class ClockRelay(Node):
    def __init__(self):
        # Must NOT use sim_time — this node IS the sim_time source
        super().__init__('clock_relay',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL,
                                 False)
                         ])
        self._last_ns: int = -1
        self._pub = self.create_publisher(Clock, '/clock', RELIABLE_QOS)
        # Subscribe with both QoS profiles to match whatever the bridge uses
        self.create_subscription(Clock, '/ign_clock', self._cb, RELIABLE_QOS)
        self.create_subscription(Clock, '/ign_clock', self._cb, BEST_EFFORT_QOS)
        self.get_logger().info('clock_relay ready — /ign_clock → /clock (monotonic filter)')

    def _cb(self, msg: Clock):
        t_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if t_ns >= self._last_ns:
            self._last_ns = t_ns
            self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ClockRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

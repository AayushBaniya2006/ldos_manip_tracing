#!/usr/bin/env python3
"""
msg_flood_node.py - High-rate message publisher for DDS load testing

Publishes messages at a configurable rate to stress the DDS middleware
and test system performance under message contention.

Usage:
    ros2 run ldos_harness msg_flood_node.py --rate 1000 --topic /flood --payload-size 1024
"""

import argparse
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String


class MsgFloodNode(Node):
    """High-rate message publisher for load testing."""

    def __init__(
        self,
        topic: str,
        rate_hz: float,
        payload_size: int,
        duration: Optional[float] = None
    ):
        super().__init__('msg_flood_node')

        self.rate_hz = rate_hz
        self.payload_size = payload_size
        self.duration = duration
        self.start_time = time.time()
        self.msg_count = 0

        # QoS for best-effort high-rate publishing
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(String, topic, qos)

        # Pre-generate payload
        self.payload = 'X' * payload_size

        # Timer for publishing
        timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'MsgFlood started: topic={topic}, rate={rate_hz}Hz, payload={payload_size}B'
        )

    def timer_callback(self):
        """Publish a message."""
        # Check duration limit
        if self.duration is not None:
            elapsed = time.time() - self.start_time
            if elapsed >= self.duration:
                self.get_logger().info(
                    f'Duration reached. Published {self.msg_count} messages'
                )
                raise SystemExit(0)

        # Publish message
        msg = String()
        msg.data = f'{self.msg_count}:{self.payload}'
        self.publisher.publish(msg)
        self.msg_count += 1

        # Periodic stats
        if self.msg_count % (self.rate_hz * 10) == 0:  # Every 10 seconds
            elapsed = time.time() - self.start_time
            actual_rate = self.msg_count / elapsed if elapsed > 0 else 0
            self.get_logger().info(
                f'Published {self.msg_count} msgs, actual rate: {actual_rate:.1f} Hz'
            )


def main():
    parser = argparse.ArgumentParser(description='DDS Message Flood Node')
    parser.add_argument('--topic', default='/flood_topic', help='Topic to publish to')
    parser.add_argument('--rate', type=float, default=1000, help='Publish rate in Hz')
    parser.add_argument('--payload-size', type=int, default=1024, help='Payload size in bytes')
    parser.add_argument('--duration', type=float, default=None, help='Duration in seconds (None=infinite)')

    args = parser.parse_args()

    rclpy.init()

    try:
        node = MsgFloodNode(
            topic=args.topic,
            rate_hz=args.rate,
            payload_size=args.payload_size,
            duration=args.duration
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

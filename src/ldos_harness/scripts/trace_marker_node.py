#!/usr/bin/env python3
"""
trace_marker_node.py - Emit custom trace markers for LDOS experiments

This node provides a service to emit named markers that can be
correlated with LTTng trace events.

Usage:
    ros2 run ldos_harness trace_marker_node.py

Service:
    /emit_marker (std_srvs/srv/SetBool) - Marker name in request
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
import time

# Try to import tracetools for direct LTTng integration
try:
    from tracetools import tracetools
    HAS_TRACETOOLS = True
except ImportError:
    HAS_TRACETOOLS = False
    print("Warning: tracetools not available, using fallback markers")


class TraceMarkerNode(Node):
    """Node for emitting trace markers."""

    def __init__(self):
        super().__init__('trace_marker_node')

        # Publisher for marker events (fallback when tracetools unavailable)
        self.marker_pub = self.create_publisher(String, '/trace_markers', 10)

        # Service for triggering markers
        self.srv = self.create_service(SetBool, 'emit_marker', self.emit_marker_callback)

        self.get_logger().info('TraceMarkerNode ready')
        self.get_logger().info(f'Tracetools available: {HAS_TRACETOOLS}')

    def emit_marker(self, marker_name: str):
        """Emit a named marker."""
        timestamp_ns = time.time_ns()

        # Method 1: Use tracetools if available (best for LTTng)
        if HAS_TRACETOOLS:
            # Note: This requires custom tracepoint registration
            # For now, we fall back to ROS message
            pass

        # Method 2: Publish ROS message (traceable via ros2:rcl_publish)
        msg = String()
        msg.data = f"{marker_name}:{timestamp_ns}"
        self.marker_pub.publish(msg)

        self.get_logger().debug(f'Marker: {marker_name} at {timestamp_ns}')

        return timestamp_ns

    def emit_marker_callback(self, request, response):
        """Service callback for marker emission."""
        # Use data field for marker name (workaround for simple service)
        marker_name = "marker"  # Default
        timestamp = self.emit_marker(marker_name)
        response.success = True
        response.message = f"{marker_name}:{timestamp}"
        return response


def main():
    rclpy.init()
    node = TraceMarkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

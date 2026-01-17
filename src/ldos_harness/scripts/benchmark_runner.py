#!/usr/bin/env python3
"""
benchmark_runner.py - Scripted benchmark for MoveIt 2 Panda manipulation

Sends a fixed pose goal to MoveIt, measures timing markers, and logs events.
No GUI required - fully automated.

Usage:
    ros2 run ldos_harness benchmark_runner.py --trial-id baseline_001 --output-dir /path/to/results

Exit codes:
    0 - Success
    1 - Planning failed
    2 - Execution failed
    3 - Timeout
    4 - ROS/system error
"""

import argparse
import json
import os
import signal
import sys
import threading
import time
import traceback
from dataclasses import dataclass, asdict, field
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any, List

import psutil
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    WorkspaceParameters,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


@dataclass
class TimingMarker:
    """A single timing event marker with dual timestamps."""
    name: str
    wall_time_ns: int      # Wall clock nanoseconds (time.time_ns())
    ros_time_ns: int       # ROS clock nanoseconds (node.get_clock().now())
    wall_time: float       # Wall clock seconds (for backwards compat)
    # Legacy field for compatibility
    timestamp_ns: int = 0  # Deprecated, use wall_time_ns

    def __post_init__(self):
        # For backwards compatibility, set timestamp_ns from wall_time_ns
        if self.timestamp_ns == 0:
            self.timestamp_ns = self.wall_time_ns


@dataclass
class TrialResult:
    """Complete result of a benchmark trial."""
    trial_id: str
    scenario: str
    status: str  # success, planning_failed, execution_failed, timeout, error
    error_message: Optional[str] = None

    # Timestamps (wall clock, seconds since epoch)
    t_goal_sent: float = 0.0
    t_planning_start: float = 0.0
    t_planning_end: float = 0.0
    t_execution_start: float = 0.0
    t_execution_end: float = 0.0

    # Timing accuracy tracking - indicates source of each timestamp
    # Sources: "feedback" (from MoveGroup state callback - most accurate),
    #          "result" (from action result planning_time field),
    #          "estimated" (from goal send time - conservative fallback)
    timing_sources: Dict[str, str] = field(default_factory=dict)

    # Derived metrics (milliseconds)
    planning_latency_ms: float = 0.0
    execution_latency_ms: float = 0.0
    total_latency_ms: float = 0.0

    # Planning details
    planner_id: str = ""
    planning_time_requested: float = 0.0
    planning_time_actual: float = 0.0
    trajectory_points: int = 0
    trajectory_duration_s: float = 0.0

    # Goal pose
    goal_pose: Dict[str, float] = field(default_factory=dict)

    # Markers
    markers: List[Dict[str, Any]] = field(default_factory=list)

    # Trace correlation - unique ID for matching with LTTng traces
    trace_correlation_id: str = ""

    # CPU utilization (Phase 1: measured during trial execution)
    cpu_percent_mean: float = 0.0
    cpu_percent_max: float = 0.0
    cpu_percent_samples: List[float] = field(default_factory=list)
    memory_percent: float = 0.0

    def compute_derived_metrics(self):
        """Compute latency metrics from timestamps."""
        if self.t_planning_start > 0 and self.t_planning_end > 0:
            self.planning_latency_ms = (self.t_planning_end - self.t_planning_start) * 1000
        if self.t_execution_start > 0 and self.t_execution_end > 0:
            self.execution_latency_ms = (self.t_execution_end - self.t_execution_start) * 1000
        if self.t_goal_sent > 0 and self.t_execution_end > 0:
            self.total_latency_ms = (self.t_execution_end - self.t_goal_sent) * 1000


class BenchmarkRunner(Node):
    """ROS 2 node for running MoveIt benchmarks."""

    def __init__(self, trial_id: str, scenario: str, output_dir: Path,
                 action_name: str = '/move_action'):
        super().__init__('benchmark_runner')
        self.trial_id = trial_id
        self.scenario = scenario
        self.output_dir = output_dir
        self.action_name = action_name
        self.result = TrialResult(trial_id=trial_id, scenario=scenario, status="pending")
        self.markers: List[TimingMarker] = []

        # Generate unique trace correlation ID for LTTng trace matching
        # Format: trial_id + timestamp in microseconds for uniqueness
        self._trace_correlation_id = f"{trial_id}_{int(time.time() * 1e6)}"
        self.result.trace_correlation_id = self._trace_correlation_id

        # Track whether we got timing from feedback (more accurate) vs estimation
        self._got_planning_feedback = False
        self._got_execution_feedback = False

        # Callback group for action client
        self._cb_group = ReentrantCallbackGroup()

        # MoveGroup action client
        self._move_group_client = ActionClient(
            self,
            MoveGroup,
            action_name,
            callback_group=self._cb_group
        )

        # Wait for action server
        self.get_logger().info(f'Waiting for MoveGroup action server on "{action_name}"...')
        if not self._move_group_client.wait_for_server(timeout_sec=60.0):
            self.result.status = "error"
            self.result.error_message = f"MoveGroup action server '{action_name}' not available after 60s"
            raise RuntimeError(self.result.error_message)

        self.get_logger().info('MoveGroup action server connected')

        # Goal configuration (from experiment_config.yaml defaults)
        self.goal_pose = Pose()
        self.goal_pose.position.x = 0.4
        self.goal_pose.position.y = 0.2
        self.goal_pose.position.z = 0.5
        self.goal_pose.orientation.w = 0.0
        self.goal_pose.orientation.x = 0.707
        self.goal_pose.orientation.y = 0.707
        self.goal_pose.orientation.z = 0.0

        self.planning_time = 5.0
        self.planner_id = "RRTConnect"

    def add_marker(self, name: str):
        """Record a timing marker with both wall clock and ROS clock timestamps."""
        wall_now = time.time()
        ros_now = self.get_clock().now()
        marker = TimingMarker(
            name=name,
            wall_time_ns=int(wall_now * 1e9),
            ros_time_ns=ros_now.nanoseconds,
            wall_time=wall_now
        )
        self.markers.append(marker)
        self.get_logger().debug(f'Marker: {name} wall={wall_now:.6f}s ros={ros_now.nanoseconds}ns')

    def _start_cpu_sampling(self, sample_rate_hz: float = 10.0):
        """Start background CPU sampling at specified rate."""
        self._cpu_samples: List[float] = []
        self._cpu_sampling_active = True
        self._cpu_sample_interval = 1.0 / sample_rate_hz

        def sample_cpu():
            # Prime psutil (first call always returns 0)
            psutil.cpu_percent(interval=None)
            while self._cpu_sampling_active:
                cpu = psutil.cpu_percent(interval=None)
                self._cpu_samples.append(cpu)
                time.sleep(self._cpu_sample_interval)

        self._cpu_thread = threading.Thread(target=sample_cpu, daemon=True)
        self._cpu_thread.start()
        self.get_logger().debug(f'CPU sampling started at {sample_rate_hz}Hz')

    def _stop_cpu_sampling(self):
        """Stop CPU sampling and compute statistics."""
        self._cpu_sampling_active = False
        if hasattr(self, '_cpu_thread'):
            self._cpu_thread.join(timeout=1.0)

        if self._cpu_samples:
            self.result.cpu_percent_samples = self._cpu_samples
            self.result.cpu_percent_mean = sum(self._cpu_samples) / len(self._cpu_samples)
            self.result.cpu_percent_max = max(self._cpu_samples)
            self.get_logger().info(
                f'CPU: mean={self.result.cpu_percent_mean:.1f}%, '
                f'max={self.result.cpu_percent_max:.1f}%, '
                f'samples={len(self._cpu_samples)}'
            )

        # Capture memory at end of trial
        self.result.memory_percent = psutil.virtual_memory().percent
        self.get_logger().debug(f'Memory: {self.result.memory_percent:.1f}%')

    def run_benchmark(self, timeout: float = 60.0) -> TrialResult:
        """Execute the benchmark trial."""
        self.get_logger().info(f'Starting benchmark trial: {self.trial_id}')
        self.get_logger().info(f'Trace correlation ID: {self._trace_correlation_id}')
        self.add_marker('benchmark_start')

        # Add trace correlation marker for LTTng trace matching
        self.add_marker(f'trace_correlation:{self._trace_correlation_id}')

        # Start CPU sampling (10Hz background thread)
        self._start_cpu_sampling(sample_rate_hz=10.0)

        try:
            # Create motion plan request
            goal_msg = self._create_move_goal()

            # Send goal - record goal send time, but DON'T set planning_start yet
            # Planning start will be captured more accurately from feedback callback
            # when MoveGroup transitions to "PLANNING" state
            self.add_marker('goal_sent')
            self.result.t_goal_sent = time.time()
            # Store conservative estimate in case feedback never arrives
            self._t_goal_sent_backup = time.time()

            send_goal_future = self._move_group_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback
            )

            # Wait for goal acceptance
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=timeout)

            if not send_goal_future.done():
                self.result.status = "timeout"
                self.result.error_message = "Goal send timed out"
                return self._finalize_result()

            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.result.status = "error"
                self.result.error_message = "Goal rejected by MoveGroup"
                return self._finalize_result()

            self.add_marker('goal_accepted')
            # Note: t_planning_start already set at goal send time

            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

            if not result_future.done():
                self.result.status = "timeout"
                self.result.error_message = "Execution timed out"
                return self._finalize_result()

            result = result_future.result().result
            self.result.t_execution_end = time.time()
            self.result.timing_sources['t_execution_end'] = 'result'
            self.add_marker('execution_complete')

            # Process result
            error_code = result.error_code.val

            if error_code == 1:  # MoveItErrorCodes.SUCCESS
                self.result.status = "success"
                self.result.planning_time_actual = result.planning_time

                # Use timing from feedback if available (most accurate)
                # Otherwise fall back to estimation from result.planning_time
                if not self._got_planning_feedback:
                    # Fallback: estimate planning start from goal send
                    self.result.t_planning_start = self._t_goal_sent_backup
                    self.result.timing_sources['t_planning_start'] = 'estimated'
                    self.get_logger().warn(
                        'Planning start not captured from feedback, using goal send time (less accurate)'
                    )

                if 't_planning_end' not in self.result.timing_sources:
                    # Fallback: compute planning end from start + planning_time
                    self.result.t_planning_end = self.result.t_planning_start + result.planning_time
                    self.result.timing_sources['t_planning_end'] = 'result'
                    self.get_logger().info(
                        f'Planning end computed from result.planning_time: {result.planning_time:.3f}s'
                    )

                if 't_execution_start' not in self.result.timing_sources:
                    # Fallback: execution starts when planning ends
                    self.result.t_execution_start = self.result.t_planning_end
                    self.result.timing_sources['t_execution_start'] = 'estimated'
                    self.get_logger().warn(
                        'Execution start not captured from feedback, using planning end time (less accurate)'
                    )

                if result.planned_trajectory.joint_trajectory.points:
                    pts = result.planned_trajectory.joint_trajectory.points
                    self.result.trajectory_points = len(pts)
                    self.result.trajectory_duration_s = pts[-1].time_from_start.sec + \
                        pts[-1].time_from_start.nanosec * 1e-9

            elif error_code == -1:  # PLANNING_FAILED
                self.result.status = "planning_failed"
                self.result.error_message = f"Planning failed with code {error_code}"
            else:
                self.result.status = "execution_failed"
                self.result.error_message = f"MoveIt error code: {error_code}"

        except Exception as e:
            self.result.status = "error"
            self.result.error_message = f"Exception: {str(e)}\n{traceback.format_exc()}"

        return self._finalize_result()

    def _create_move_goal(self) -> MoveGroup.Goal:
        """Create MoveGroup action goal."""
        goal = MoveGroup.Goal()

        # Motion plan request
        goal.request.group_name = "panda_arm"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = self.planning_time
        goal.request.planner_id = self.planner_id
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        # Workspace parameters
        goal.request.workspace_parameters.header.frame_id = "panda_link0"
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = -1.0
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.0

        # Goal constraints (pose goal)
        constraints = Constraints()
        constraints.name = "benchmark_goal"

        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "panda_link0"
        position_constraint.link_name = "panda_link8"
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        # Bounding volume (small sphere around target)
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm tolerance
        bounding_volume.primitives.append(primitive)

        primitive_pose = Pose()
        primitive_pose.position = self.goal_pose.position
        primitive_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(primitive_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "panda_link0"
        orientation_constraint.link_name = "panda_link8"
        orientation_constraint.orientation = self.goal_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        goal.request.goal_constraints.append(constraints)

        # Planning options
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0

        # Store goal pose in result
        self.result.goal_pose = {
            "x": self.goal_pose.position.x,
            "y": self.goal_pose.position.y,
            "z": self.goal_pose.position.z,
            "qw": self.goal_pose.orientation.w,
            "qx": self.goal_pose.orientation.x,
            "qy": self.goal_pose.orientation.y,
            "qz": self.goal_pose.orientation.z,
        }
        self.result.planner_id = self.planner_id
        self.result.planning_time_requested = self.planning_time

        return goal

    def _feedback_callback(self, feedback_msg):
        """Handle feedback from MoveGroup.

        This callback provides the most accurate timing for planning/execution
        transitions since it's triggered when MoveGroup actually changes state,
        rather than estimating from goal send/result times.
        """
        feedback = feedback_msg.feedback
        state = feedback.state
        self.get_logger().debug(f'MoveGroup state: {state}')

        # Record state transitions as markers with high-resolution timestamps
        if state == "PLANNING":
            self.add_marker('planning_started_feedback')
            # Capture actual planning start time from feedback (most accurate)
            if self.result.t_planning_start == 0.0:
                self.result.t_planning_start = time.time()
                self.result.timing_sources['t_planning_start'] = 'feedback'
                self._got_planning_feedback = True
                self.get_logger().info(
                    f'Planning start captured from feedback at {self.result.t_planning_start:.6f}'
                )

        elif state == "MONITOR":
            self.add_marker('execution_started_feedback')
            # Capture planning end / execution start from feedback (most accurate)
            if self.result.t_planning_end == 0.0:
                self.result.t_planning_end = time.time()
                self.result.timing_sources['t_planning_end'] = 'feedback'
                self.get_logger().info(
                    f'Planning end captured from feedback at {self.result.t_planning_end:.6f}'
                )
            if self.result.t_execution_start == 0.0:
                self.result.t_execution_start = time.time()
                self.result.timing_sources['t_execution_start'] = 'feedback'
                self._got_execution_feedback = True
                self.get_logger().info(
                    f'Execution start captured from feedback at {self.result.t_execution_start:.6f}'
                )

    def _finalize_result(self) -> TrialResult:
        """Finalize and save the trial result."""
        self.add_marker('benchmark_end')

        # Stop CPU sampling and compute statistics
        self._stop_cpu_sampling()

        # Compute derived metrics
        self.result.compute_derived_metrics()

        # Add markers to result
        self.result.markers = [asdict(m) for m in self.markers]

        # Save result
        self._save_result()

        return self.result

    def _save_result(self):
        """Save trial result to JSON file."""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        result_file = self.output_dir / f"{self.trial_id}_result.json"

        with open(result_file, 'w') as f:
            json.dump(asdict(self.result), f, indent=2)

        self.get_logger().info(f'Result saved to: {result_file}')


def main():
    parser = argparse.ArgumentParser(description='LDOS Benchmark Runner')
    parser.add_argument('--trial-id', required=True, help='Unique trial identifier')
    parser.add_argument('--scenario', default='baseline', help='Scenario name (baseline, cpu_load, msg_load)')
    parser.add_argument('--output-dir', required=True, help='Directory for output files')
    parser.add_argument('--timeout', type=float, default=60.0, help='Total timeout in seconds')
    parser.add_argument('--goal-x', type=float, default=0.4, help='Goal position X')
    parser.add_argument('--goal-y', type=float, default=0.2, help='Goal position Y')
    parser.add_argument('--goal-z', type=float, default=0.5, help='Goal position Z')
    parser.add_argument('--action-name', default='/move_action',
                        help='MoveGroup action name (default: /move_action)')

    args = parser.parse_args()

    # Initialize ROS
    rclpy.init()

    exit_code = 0
    node = None

    try:
        output_dir = Path(args.output_dir)
        node = BenchmarkRunner(
            trial_id=args.trial_id,
            scenario=args.scenario,
            output_dir=output_dir,
            action_name=args.action_name
        )

        # Override goal pose if specified
        node.goal_pose.position.x = args.goal_x
        node.goal_pose.position.y = args.goal_y
        node.goal_pose.position.z = args.goal_z

        # Run benchmark
        result = node.run_benchmark(timeout=args.timeout)

        # Set exit code based on result
        if result.status == "success":
            exit_code = 0
        elif result.status == "planning_failed":
            exit_code = 1
        elif result.status == "execution_failed":
            exit_code = 2
        elif result.status == "timeout":
            exit_code = 3
        else:
            exit_code = 4

        # Print summary
        print(f"\n=== Trial {args.trial_id} ===")
        print(f"Status: {result.status}")
        if result.status == "success":
            print(f"Planning latency: {result.planning_latency_ms:.2f} ms")
            print(f"Execution latency: {result.execution_latency_ms:.2f} ms")
            print(f"Total latency: {result.total_latency_ms:.2f} ms")
            print(f"Trajectory points: {result.trajectory_points}")
        elif result.error_message:
            print(f"Error: {result.error_message}")

        # Always print CPU metrics (captured regardless of success/failure)
        if result.cpu_percent_samples:
            print(f"CPU utilization: mean={result.cpu_percent_mean:.1f}%, max={result.cpu_percent_max:.1f}%")
            print(f"Memory utilization: {result.memory_percent:.1f}%")

    except Exception as e:
        print(f"FATAL: {e}", file=sys.stderr)
        traceback.print_exc()
        exit_code = 4

    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
analyze_trace.py - Extract metrics from LTTng traces for LDOS experiments

This script processes LTTng trace data to extract:
- Callback durations
- Pub/sub latencies (where possible)
- Scheduler events
- Control loop timing

Usage:
    python3 analyze_trace.py --trace-dir /path/to/trace --output-dir /path/to/output

Requirements:
    pip install babeltrace2 pandas matplotlib
"""

import argparse
import json
import os
import sys
from collections import defaultdict
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple
import warnings

def check_babeltrace2_import():
    """Check for babeltrace2 with helpful error messages."""
    try:
        import bt2
        return bt2
    except ImportError as e:
        import subprocess

        error_lines = [
            "=" * 60,
            "ERROR: babeltrace2 Python bindings not found",
            "=" * 60,
            "",
            f"Import error: {e}",
            "",
            "This is required for LTTng trace analysis.",
            "",
            "Installation options:",
            "",
            "  Option 1 - System package (Ubuntu/Debian):",
            "    sudo apt install python3-babeltrace2",
            "",
            "  Option 2 - pip (may require build dependencies):",
            "    pip install babeltrace2",
            "",
        ]

        # Check if we're in a virtualenv
        in_venv = hasattr(sys, 'real_prefix') or (
            hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix
        )
        if in_venv:
            venv_path = sys.prefix
            error_lines.extend([
                f"  NOTE: You are in a virtualenv: {venv_path}",
                "",
                "  Option 3 - If using virtualenv without system packages:",
                "    Recreate venv with system site-packages access:",
                "    python3 -m venv --system-site-packages .venv",
                "    source .venv/bin/activate",
                "",
                "  Or try running with system Python:",
                "    deactivate",
                "    /usr/bin/python3 analyze_trace.py ...",
                "",
            ])

        # Check if system package exists but not accessible
        try:
            result = subprocess.run(
                ['dpkg', '-s', 'python3-babeltrace2'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                error_lines.extend([
                    "  INFO: python3-babeltrace2 is installed on the system",
                    "  but not accessible to this Python interpreter.",
                    f"  Current Python: {sys.executable}",
                    "",
                ])
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass  # Not on Debian-based system or command failed

        for line in error_lines:
            print(line, file=sys.stderr)
        sys.exit(1)


bt2 = check_babeltrace2_import()

import pandas as pd

# Suppress pandas warnings
warnings.filterwarnings('ignore', category=FutureWarning)


@dataclass
class CallbackEvent:
    """A callback start/end event pair."""
    callback_ptr: int
    start_ns: int
    end_ns: Optional[int] = None
    duration_ns: Optional[int] = None
    vpid: int = 0
    vtid: int = 0
    procname: str = ""


@dataclass
class TraceMetrics:
    """Aggregated metrics from a trace."""
    trace_path: str
    start_time_ns: int = 0
    end_time_ns: int = 0
    duration_s: float = 0.0

    # Callback metrics
    total_callbacks: int = 0
    callback_durations_ns: List[int] = field(default_factory=list)
    callback_duration_mean_ms: float = 0.0
    callback_duration_std_ms: float = 0.0
    callback_duration_p50_ms: float = 0.0
    callback_duration_p95_ms: float = 0.0
    callback_duration_p99_ms: float = 0.0
    callback_duration_max_ms: float = 0.0

    # Per-callback type statistics
    callback_stats: Dict[str, Dict[str, float]] = field(default_factory=dict)

    # Control loop jitter metrics (Phase 2: T3 objective)
    control_loop_period_mean_ms: float = 0.0
    control_loop_jitter_ms: float = 0.0  # std dev of period
    control_loop_deadline_violations: int = 0
    control_loop_samples: int = 0

    # Control loop execution time metrics (for WCET analysis)
    control_loop_exec_mean_ms: float = 0.0
    control_loop_exec_p95_ms: float = 0.0
    control_loop_exec_p99_ms: float = 0.0
    control_loop_exec_max_ms: float = 0.0  # WCET approximation
    control_loop_expected_period_ms: float = 2.0  # Expected period (default 500Hz)

    # Executor contention metrics (Phase 3)
    executor_spinlock_ratio: float = 0.0  # Ratio of short waits (<100μs)
    executor_wait_mean_ms: float = 0.0
    executor_wait_max_ms: float = 0.0
    executor_wait_samples: int = 0

    # Message latency metrics (Phase 4)
    message_latency_mean_ms: float = 0.0
    message_latency_p95_ms: float = 0.0
    message_latency_max_ms: float = 0.0
    message_latency_samples: int = 0
    message_latency_stats: Dict[str, Dict[str, float]] = field(default_factory=dict)

    # Scheduler metrics
    context_switches: int = 0
    wakeups: int = 0

    # ROS 2 specific
    publishers_created: int = 0
    subscriptions_created: int = 0
    nodes_created: int = 0

    def compute_aggregates(self):
        """Compute aggregate statistics from raw data."""
        if not self.callback_durations_ns:
            return

        durations_ms = [d / 1e6 for d in self.callback_durations_ns]
        n = len(durations_ms)

        # Warn if sample size is low for reliable percentiles
        if n < 100:
            print(f"WARNING: Only {n} callbacks recorded. Percentile statistics may be unreliable.")

        sorted_durations = sorted(durations_ms)

        self.callback_duration_mean_ms = sum(durations_ms) / n
        self.callback_duration_max_ms = max(durations_ms)

        # Safe percentile calculation with bounds checking
        self.callback_duration_p50_ms = sorted_durations[min(n // 2, n - 1)]
        self.callback_duration_p95_ms = sorted_durations[min(int(n * 0.95), n - 1)]
        self.callback_duration_p99_ms = sorted_durations[min(int(n * 0.99), n - 1)]

        # Compute std dev
        mean = self.callback_duration_mean_ms
        variance = sum((d - mean) ** 2 for d in durations_ms) / n
        self.callback_duration_std_ms = variance ** 0.5


class TraceAnalyzer:
    """Analyze LTTng traces from ROS 2 applications."""

    # Strict patterns for control loop callbacks - specifically the joint trajectory controller
    # These are the actual update callbacks from ros2_control that run at the control rate
    # The order matters: more specific patterns are checked first
    CONTROL_LOOP_PATTERNS_STRICT = [
        # Primary: joint_trajectory_controller update callbacks
        'joint_trajectory_controller',
        'JointTrajectoryController',
        # Specific controller update methods
        'panda_arm_controller',
        'controller_manager::ControllerManager::update',
        'update_and_write_commands',
        # ros2_control internal update
        'ros2_control::ControllerManager',
    ]

    # Patterns to EXCLUDE even if they match (too generic or not real-time critical)
    CONTROL_LOOP_EXCLUDE_PATTERNS = [
        'joint_state_broadcaster',  # Not a control callback, just state publishing
        'state_publisher',
        'robot_state_publisher',
        'tf_broadcaster',
        'parameter',  # Parameter callbacks
        'service',    # Service callbacks
        'timer',      # Generic timers (not control loop)
    ]

    # Expected control loop period in ms
    # ros2_control default is 500 Hz = 2ms (from panda_controllers.yaml: update_rate: 500)
    CONTROL_LOOP_PERIOD_MS = 2.0  # 500Hz for ros2_control

    # Deadline threshold multiplier for violation detection
    # Period > 2x expected indicates a missed control deadline
    DEADLINE_THRESHOLD_MULTIPLIER = 2.0

    def __init__(self, trace_path: str, control_rate_hz: float = 500.0):
        """Initialize trace analyzer.

        Args:
            trace_path: Path to LTTng trace directory
            control_rate_hz: Expected control loop rate in Hz (default: 500 Hz from ros2_control)
        """
        self.trace_path = Path(trace_path)
        self.metrics = TraceMetrics(trace_path=str(self.trace_path))

        # Configure control loop timing from rate
        self.CONTROL_LOOP_PERIOD_MS = 1000.0 / control_rate_hz

        # State tracking - use stack per thread for proper nested callback handling
        self._callback_stacks: Dict[int, List[CallbackEvent]] = {}  # vtid -> stack of active callbacks
        self._callback_names: Dict[int, str] = {}  # callback_ptr -> name
        self._callback_nodes: Dict[int, str] = {}  # callback_ptr -> node name

        # Control loop jitter tracking (Phase 2)
        self._control_loop_timestamps: List[int] = []  # Start timestamps for control callbacks
        self._control_loop_callback_ptrs: set = set()  # Identified control loop callbacks
        self._control_loop_durations_ns: List[int] = []  # Durations of control callbacks

        # Per-controller tracking for more granular analysis
        self._per_controller_timestamps: Dict[str, List[int]] = {}  # controller_name -> timestamps

        # Executor contention tracking (Phase 3)
        self._executor_wait_start: Dict[int, int] = {}  # vtid -> wait_for_work start timestamp
        self._executor_wait_durations_ns: List[int] = []  # All wait durations

        # Message latency tracking (Phase 4)
        self._publish_timestamps: Dict[int, int] = {}  # message_ptr -> publish timestamp
        self._message_latencies_ns: List[int] = []  # All message latencies

        # Data loss detection counters
        self._unmatched_callback_ends: int = 0
        self._malformed_events: int = 0

    def analyze(self) -> TraceMetrics:
        """Process the trace and extract metrics."""
        print(f"Analyzing trace: {self.trace_path}")

        # Find the trace data (may be in subdirectory)
        trace_data_path = self._find_trace_data()
        if not trace_data_path:
            print(f"ERROR: No trace data found in {self.trace_path}")
            return self.metrics

        print(f"Trace data path: {trace_data_path}")

        # Create message iterator
        try:
            msg_it = bt2.TraceCollectionMessageIterator(str(trace_data_path))
        except Exception as e:
            print(f"ERROR: Failed to open trace: {e}")
            return self.metrics

        event_count = 0
        first_ts = None
        last_ts = None

        for msg in msg_it:
            if type(msg) is not bt2._EventMessageConst:
                continue

            event = msg.event
            event_name = event.name
            ts_ns = msg.default_clock_snapshot.ns_from_origin

            if first_ts is None:
                first_ts = ts_ns
            last_ts = ts_ns

            event_count += 1

            # Process different event types
            if event_name == 'ros2:callback_start':
                self._handle_callback_start(event, ts_ns)
            elif event_name == 'ros2:callback_end':
                self._handle_callback_end(event, ts_ns)
            elif event_name == 'ros2:rclcpp_callback_register':
                self._handle_callback_register(event)
            elif event_name == 'ros2:rcl_node_init':
                self.metrics.nodes_created += 1
            elif event_name == 'ros2:rcl_publisher_init':
                self.metrics.publishers_created += 1
            elif event_name == 'ros2:rcl_subscription_init':
                self.metrics.subscriptions_created += 1
            elif event_name == 'sched_switch':
                self.metrics.context_switches += 1
            elif event_name == 'sched_wakeup':
                self.metrics.wakeups += 1
            # Executor events (Phase 3)
            elif event_name == 'ros2:rclcpp_executor_wait_for_work':
                self._handle_executor_wait_start(event, ts_ns)
            elif event_name == 'ros2:rclcpp_executor_get_next_ready':
                self._handle_executor_wait_end(event, ts_ns)
            # Message latency events (Phase 4)
            elif event_name in ('ros2:rcl_publish', 'ros2:rclcpp_publish'):
                self._handle_message_publish(event, ts_ns)
            elif event_name in ('ros2:rcl_take', 'ros2:rclcpp_take'):
                self._handle_message_take(event, ts_ns)

        # Finalize
        if first_ts and last_ts:
            self.metrics.start_time_ns = first_ts
            self.metrics.end_time_ns = last_ts
            self.metrics.duration_s = (last_ts - first_ts) / 1e9

        self.metrics.total_callbacks = len(self.metrics.callback_durations_ns)
        self.metrics.compute_aggregates()

        # Compute control loop jitter (Phase 2)
        self._compute_jitter()

        # Compute executor contention (Phase 3)
        self._compute_executor_contention()

        # Compute message latency (Phase 4)
        self._compute_message_latency()

        print(f"Processed {event_count} events")

        # Document which event types are actually analyzed
        ANALYZED_EVENTS = [
            'ros2:callback_start', 'ros2:callback_end', 'ros2:rclcpp_callback_register',
            'ros2:rcl_node_init', 'ros2:rcl_publisher_init', 'ros2:rcl_subscription_init',
            'sched_switch', 'sched_wakeup'
        ]
        print(f"Note: Analyzing {len(ANALYZED_EVENTS)} event types. Other enabled events are ignored.")

        # Warn if trace appears empty or has no callback events
        if event_count == 0:
            print("WARNING: Trace contains no events. Check if tracing was properly enabled.")
        elif self.metrics.total_callbacks == 0:
            print("WARNING: No callback events found in trace. ROS 2 tracepoints may not be enabled.")
        print(f"Trace duration: {self.metrics.duration_s:.2f}s")
        print(f"Callbacks: {self.metrics.total_callbacks}")

        # Report data loss indicators
        if self._unmatched_callback_ends > 0:
            print(f"WARNING: {self._unmatched_callback_ends} callback_end events had no matching start (data loss)")
        if self._malformed_events > 0:
            print(f"WARNING: {self._malformed_events} malformed events skipped")

        return self.metrics

    def _find_trace_data(self) -> Optional[Path]:
        """Find the actual trace data directory."""
        # LTTng traces typically have a ust/uid or kernel subdirectory
        candidates = [
            self.trace_path,
            self.trace_path / "ust",
            self.trace_path / "kernel",
        ]

        # Also check for uid subdirectories
        ust_path = self.trace_path / "ust"
        if ust_path.exists():
            for uid_dir in ust_path.iterdir():
                if uid_dir.is_dir():
                    candidates.append(uid_dir)
                    for subdir in uid_dir.iterdir():
                        if subdir.is_dir():
                            candidates.append(subdir)

        # Find first valid trace
        for candidate in candidates:
            if candidate.exists():
                # Check for metadata file (indicates valid trace)
                if (candidate / "metadata").exists():
                    return candidate
                # Check subdirectories
                for subdir in candidate.rglob("metadata"):
                    return subdir.parent

        return self.trace_path if self.trace_path.exists() else None

    def _handle_callback_start(self, event, ts_ns: int):
        """Handle callback_start event using stack-based approach for nested callbacks."""
        try:
            callback_ptr = event['callback']
        except (KeyError, TypeError) as e:
            self._malformed_events += 1
            return

        try:
            vtid = event.common_context_field.get('vtid', 0) if event.common_context_field else 0
            vpid = event.common_context_field.get('vpid', 0) if event.common_context_field else 0
            procname = str(event.common_context_field.get('procname', '')) if event.common_context_field else ''
        except Exception:
            vtid, vpid, procname = 0, 0, ''

        # Initialize stack for this thread if needed
        if vtid not in self._callback_stacks:
            self._callback_stacks[vtid] = []

        # Push callback onto thread's stack
        self._callback_stacks[vtid].append(CallbackEvent(
            callback_ptr=callback_ptr,
            start_ns=ts_ns,
            vpid=vpid,
            vtid=vtid,
            procname=procname
        ))

        # Track control loop callback timestamps for jitter analysis (Phase 2)
        if callback_ptr in self._control_loop_callback_ptrs:
            self._control_loop_timestamps.append(ts_ns)

            # Also track per-controller timestamps for granular analysis
            symbol = self._callback_names.get(callback_ptr, 'unknown')
            for pattern, timestamps in self._per_controller_timestamps.items():
                if pattern.lower() in symbol.lower():
                    timestamps.append(ts_ns)
                    break

    def _handle_callback_end(self, event, ts_ns: int):
        """Handle callback_end event using stack-based approach for nested callbacks."""
        try:
            callback_ptr = event['callback']
        except (KeyError, TypeError):
            self._malformed_events += 1
            return

        try:
            vtid = event.common_context_field.get('vtid', 0) if event.common_context_field else 0
        except Exception:
            vtid = 0

        if vtid not in self._callback_stacks or not self._callback_stacks[vtid]:
            # Data loss detection: callback_end without matching start
            self._unmatched_callback_ends += 1
            if self._unmatched_callback_ends == 1 or self._unmatched_callback_ends % 100 == 0:
                print(f"WARNING: {self._unmatched_callback_ends} unmatched callback_end events "
                      "(possible buffer overflow or partial trace)")
            return

        # Find the most recent matching callback on the stack
        # In most cases, it's the top of stack, but handle out-of-order ends gracefully
        stack = self._callback_stacks[vtid]
        for i in range(len(stack) - 1, -1, -1):
            if stack[i].callback_ptr == callback_ptr:
                cb_event = stack.pop(i)
                cb_event.end_ns = ts_ns
                cb_event.duration_ns = ts_ns - cb_event.start_ns
                self.metrics.callback_durations_ns.append(cb_event.duration_ns)

                # Track control loop callback durations for WCET analysis
                if callback_ptr in self._control_loop_callback_ptrs:
                    self._control_loop_durations_ns.append(cb_event.duration_ns)
                break

    def _handle_callback_register(self, event):
        """Handle callback registration event with strict control loop identification."""
        callback_ptr = event['callback']
        symbol = str(event.get('symbol', 'unknown'))
        self._callback_names[callback_ptr] = symbol

        # Extract node name if available (from symbol demangling)
        # Typical format: "NodeName::callback_name" or "namespace::NodeName::method"
        node_name = self._extract_node_name(symbol)
        if node_name:
            self._callback_nodes[callback_ptr] = node_name

        # Check if this is a control loop callback (Phase 2)
        # Use strict filtering to avoid false positives
        symbol_lower = symbol.lower()

        # First check exclusion patterns - if matched, definitely not a control callback
        for exclude_pattern in self.CONTROL_LOOP_EXCLUDE_PATTERNS:
            if exclude_pattern.lower() in symbol_lower:
                return  # Excluded, not a control loop callback

        # Check strict control loop patterns
        is_control_callback = False
        matched_pattern = None
        for pattern in self.CONTROL_LOOP_PATTERNS_STRICT:
            if pattern.lower() in symbol_lower:
                is_control_callback = True
                matched_pattern = pattern
                break

        if is_control_callback:
            self._control_loop_callback_ptrs.add(callback_ptr)
            # Track per-controller for granular analysis
            controller_key = matched_pattern or 'unknown_controller'
            if controller_key not in self._per_controller_timestamps:
                self._per_controller_timestamps[controller_key] = []

    def _extract_node_name(self, symbol: str) -> Optional[str]:
        """Extract node name from callback symbol.

        Handles common symbol formats:
        - "NodeName::callback_method"
        - "namespace::NodeName::method"
        - "void NodeName::method()"
        """
        # Remove return type prefix if present
        if symbol.startswith('void '):
            symbol = symbol[5:]

        # Split by :: and try to identify node name
        parts = symbol.split('::')
        if len(parts) >= 2:
            # Usually the class name is the second-to-last before the method
            # e.g., "controller_manager::ControllerManager::update"
            for i, part in enumerate(parts[:-1]):
                # Node/class names typically start with uppercase or contain "controller"
                if part and (part[0].isupper() or 'controller' in part.lower()):
                    return part
        return None

    def _compute_jitter(self):
        """Compute control loop jitter from inter-arrival times (Phase 2: T3 objective).

        Jitter is defined as the standard deviation of the period between
        consecutive control loop callback invocations.

        Uses strict callback identification to ensure only actual controller
        update callbacks are measured (not state broadcasters or other callbacks).
        """
        # Report which callbacks were identified as control loop
        print(f"  Control loop analysis (T3):")
        print(f"    Expected period: {self.CONTROL_LOOP_PERIOD_MS:.2f}ms ({1000.0/self.CONTROL_LOOP_PERIOD_MS:.0f} Hz)")
        print(f"    Identified {len(self._control_loop_callback_ptrs)} control loop callback type(s):")

        for cb_ptr in self._control_loop_callback_ptrs:
            symbol = self._callback_names.get(cb_ptr, 'unknown')
            # Truncate long symbols for readability
            if len(symbol) > 60:
                symbol = symbol[:57] + '...'
            print(f"      - {symbol}")

        if len(self._control_loop_timestamps) < 2:
            print(f"    WARNING: Insufficient samples ({len(self._control_loop_timestamps)} callbacks)")
            print(f"    Possible causes:")
            print(f"      - Controller not active during trace")
            print(f"      - Callback symbol not matching patterns")
            print(f"      - Trace buffer overflow lost events")
            return

        # Sort timestamps (they should already be sorted, but ensure it)
        timestamps = sorted(self._control_loop_timestamps)

        # Compute inter-arrival times (periods)
        periods_ns = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps) - 1)]
        periods_ms = [p / 1e6 for p in periods_ns]

        n = len(periods_ms)
        self.metrics.control_loop_samples = n

        # Filter out outliers for mean calculation (periods > 10x expected are likely gaps, not jitter)
        # But keep them for violation counting
        outlier_threshold_ms = self.CONTROL_LOOP_PERIOD_MS * 10.0
        filtered_periods = [p for p in periods_ms if p < outlier_threshold_ms]
        outlier_count = n - len(filtered_periods)

        if filtered_periods:
            # Compute mean period from non-outlier samples
            mean_period_ms = sum(filtered_periods) / len(filtered_periods)
            self.metrics.control_loop_period_mean_ms = mean_period_ms

            # Compute jitter (standard deviation of period) from non-outlier samples
            variance = sum((p - mean_period_ms) ** 2 for p in filtered_periods) / len(filtered_periods)
            jitter_ms = variance ** 0.5
            self.metrics.control_loop_jitter_ms = jitter_ms
        else:
            mean_period_ms = sum(periods_ms) / n
            self.metrics.control_loop_period_mean_ms = mean_period_ms
            variance = sum((p - mean_period_ms) ** 2 for p in periods_ms) / n
            jitter_ms = variance ** 0.5
            self.metrics.control_loop_jitter_ms = jitter_ms

        # Count deadline violations (period > 2x expected, indicating missed deadline)
        deadline_threshold_ms = self.CONTROL_LOOP_PERIOD_MS * self.DEADLINE_THRESHOLD_MULTIPLIER
        violations = sum(1 for p in periods_ms if p > deadline_threshold_ms)
        self.metrics.control_loop_deadline_violations = violations

        # Report findings
        print(f"    Samples: {n} periods ({outlier_count} outliers filtered for mean/jitter)")
        print(f"    Measured period: mean={mean_period_ms:.3f}ms (expected: {self.CONTROL_LOOP_PERIOD_MS:.2f}ms)")
        print(f"    Jitter (std dev): {jitter_ms:.3f}ms")
        print(f"    Deadline violations (>{deadline_threshold_ms:.1f}ms): {violations}")

        # Provide interpretation
        if abs(mean_period_ms - self.CONTROL_LOOP_PERIOD_MS) > self.CONTROL_LOOP_PERIOD_MS * 0.2:
            print(f"    WARNING: Measured period differs significantly from expected")
            print(f"             This may indicate wrong callbacks being measured")
            print(f"             or controller running at different rate")

        if violations > 0:
            violation_rate = violations / n * 100
            print(f"    WARNING: {violation_rate:.1f}% deadline violations detected")
            if violation_rate > 1.0:
                print(f"             System may be overloaded or controller under-provisioned")

        # Report per-controller breakdown if available
        if self._per_controller_timestamps:
            print(f"    Per-controller breakdown:")
            for controller, tstamps in self._per_controller_timestamps.items():
                if len(tstamps) >= 2:
                    c_periods = [(tstamps[i+1] - tstamps[i]) / 1e6 for i in range(len(tstamps) - 1)]
                    c_mean = sum(c_periods) / len(c_periods)
                    c_var = sum((p - c_mean) ** 2 for p in c_periods) / len(c_periods)
                    c_jitter = c_var ** 0.5
                    print(f"      {controller}: {len(c_periods)} samples, mean={c_mean:.3f}ms, jitter={c_jitter:.3f}ms")

        # Compute control loop execution time (WCET approximation)
        self._compute_control_loop_execution_time()

    def _compute_control_loop_execution_time(self):
        """Compute control loop callback execution time statistics.

        This provides WCET (Worst-Case Execution Time) approximation for
        schedulability analysis.
        """
        if not self._control_loop_durations_ns:
            print(f"    Execution time: no duration samples")
            return

        durations_ms = [d / 1e6 for d in self._control_loop_durations_ns]
        n = len(durations_ms)

        # Compute statistics
        self.metrics.control_loop_exec_mean_ms = sum(durations_ms) / n
        self.metrics.control_loop_exec_max_ms = max(durations_ms)
        self.metrics.control_loop_expected_period_ms = self.CONTROL_LOOP_PERIOD_MS

        # Compute percentiles
        sorted_durations = sorted(durations_ms)
        p95_idx = min(int(n * 0.95), n - 1)
        p99_idx = min(int(n * 0.99), n - 1)
        self.metrics.control_loop_exec_p95_ms = sorted_durations[p95_idx]
        self.metrics.control_loop_exec_p99_ms = sorted_durations[p99_idx]

        # Report findings
        print(f"    Execution time (WCET analysis):")
        print(f"      Samples: {n}")
        print(f"      Mean: {self.metrics.control_loop_exec_mean_ms:.3f}ms")
        print(f"      P95:  {self.metrics.control_loop_exec_p95_ms:.3f}ms")
        print(f"      P99:  {self.metrics.control_loop_exec_p99_ms:.3f}ms")
        print(f"      Max (WCET approx): {self.metrics.control_loop_exec_max_ms:.3f}ms")

        # Check if WCET exceeds period (schedulability issue)
        if self.metrics.control_loop_exec_max_ms > self.CONTROL_LOOP_PERIOD_MS:
            print(f"      WARNING: Max execution time ({self.metrics.control_loop_exec_max_ms:.3f}ms) "
                  f"exceeds period ({self.CONTROL_LOOP_PERIOD_MS:.2f}ms)")
            print(f"               Control loop may miss deadlines under load")

        # Compute utilization
        utilization = self.metrics.control_loop_exec_mean_ms / self.CONTROL_LOOP_PERIOD_MS
        print(f"      Utilization: {utilization:.1%} (mean exec / period)")

    def _handle_executor_wait_start(self, event, ts_ns: int):
        """Handle executor wait_for_work start event (Phase 3)."""
        try:
            vtid = event.common_context_field.get('vtid', 0) if event.common_context_field else 0
        except Exception:
            vtid = 0
        self._executor_wait_start[vtid] = ts_ns

    def _handle_executor_wait_end(self, event, ts_ns: int):
        """Handle executor get_next_ready event (Phase 3)."""
        try:
            vtid = event.common_context_field.get('vtid', 0) if event.common_context_field else 0
        except Exception:
            vtid = 0

        if vtid in self._executor_wait_start:
            start_ts = self._executor_wait_start.pop(vtid)
            duration_ns = ts_ns - start_ts
            if duration_ns > 0:
                self._executor_wait_durations_ns.append(duration_ns)

    def _compute_executor_contention(self):
        """Compute executor contention metrics (Phase 3).

        Spinlock ratio: proportion of waits < 100μs indicates busy-waiting/contention.
        High spinlock ratio = executor is frequently polling, not blocking.
        """
        if not self._executor_wait_durations_ns:
            print(f"  Executor contention: no wait_for_work events found")
            return

        durations_ns = self._executor_wait_durations_ns
        durations_ms = [d / 1e6 for d in durations_ns]

        n = len(durations_ms)
        self.metrics.executor_wait_samples = n

        # Mean and max wait time
        self.metrics.executor_wait_mean_ms = sum(durations_ms) / n
        self.metrics.executor_wait_max_ms = max(durations_ms)

        # Spinlock ratio: waits < 100μs (0.1ms) indicate busy-waiting
        SPINLOCK_THRESHOLD_MS = 0.1
        short_waits = sum(1 for d in durations_ms if d < SPINLOCK_THRESHOLD_MS)
        self.metrics.executor_spinlock_ratio = short_waits / n

        # Report findings
        print(f"  Executor contention analysis (Phase 3):")
        print(f"    Wait time: mean={self.metrics.executor_wait_mean_ms:.3f}ms, max={self.metrics.executor_wait_max_ms:.3f}ms")
        print(f"    Samples: {n}, Spinlock ratio: {self.metrics.executor_spinlock_ratio:.2%}")

        if self.metrics.executor_spinlock_ratio > 0.3:
            print(f"    WARNING: High spinlock ratio indicates executor contention")

    def _handle_message_publish(self, event, ts_ns: int):
        """Handle message publish event (Phase 4)."""
        try:
            # Try different field names used by ros2_tracing
            msg_ptr = None
            for field_name in ('message', 'msg', 'message_pointer'):
                try:
                    msg_ptr = event[field_name]
                    break
                except (KeyError, TypeError):
                    continue

            if msg_ptr is not None:
                self._publish_timestamps[msg_ptr] = ts_ns
        except Exception:
            pass  # Silently ignore malformed events

    def _handle_message_take(self, event, ts_ns: int):
        """Handle message take event (Phase 4)."""
        try:
            msg_ptr = None
            for field_name in ('message', 'msg', 'message_pointer'):
                try:
                    msg_ptr = event[field_name]
                    break
                except (KeyError, TypeError):
                    continue

            if msg_ptr is not None and msg_ptr in self._publish_timestamps:
                publish_ts = self._publish_timestamps.pop(msg_ptr)
                latency_ns = ts_ns - publish_ts
                if latency_ns > 0:
                    self._message_latencies_ns.append(latency_ns)
        except Exception:
            pass  # Silently ignore malformed events

    def _compute_message_latency(self):
        """Compute message pub→sub latency statistics (Phase 4)."""
        if not self._message_latencies_ns:
            print(f"  Message latency: no matched publish→take pairs found")
            print(f"    (Pending publishes: {len(self._publish_timestamps)})")
            return

        latencies_ns = self._message_latencies_ns
        latencies_ms = [l / 1e6 for l in latencies_ns]

        n = len(latencies_ms)
        self.metrics.message_latency_samples = n

        # Compute statistics
        self.metrics.message_latency_mean_ms = sum(latencies_ms) / n
        self.metrics.message_latency_max_ms = max(latencies_ms)

        # P95
        sorted_latencies = sorted(latencies_ms)
        p95_idx = min(int(n * 0.95), n - 1)
        self.metrics.message_latency_p95_ms = sorted_latencies[p95_idx]

        # Report findings
        print(f"  Message latency analysis (Phase 4):")
        print(f"    Matched pairs: {n}")
        print(f"    Latency: mean={self.metrics.message_latency_mean_ms:.3f}ms, "
              f"p95={self.metrics.message_latency_p95_ms:.3f}ms, "
              f"max={self.metrics.message_latency_max_ms:.3f}ms")

        # Warn about unmatched publishes (potential message loss)
        if self._publish_timestamps:
            print(f"    Note: {len(self._publish_timestamps)} unmatched publishes "
                  "(messages not yet received or trace ended)")


def aggregate_results(result_dir: Path, output_path: Path):
    """Aggregate multiple trial results into a summary CSV."""
    print(f"\nAggregating results from: {result_dir}")

    rows = []
    for result_file in result_dir.glob("*_result.json"):
        try:
            with open(result_file) as f:
                data = json.load(f)
            rows.append(data)
        except Exception as e:
            print(f"Warning: Failed to load {result_file}: {e}")

    if not rows:
        print("No result files found")
        return

    df = pd.DataFrame(rows)

    # Select key columns for summary
    key_cols = [
        'trial_id', 'scenario', 'status',
        'planning_latency_ms', 'execution_latency_ms', 'total_latency_ms',
        'trajectory_points', 'trajectory_duration_s', 'planning_time_actual'
    ]
    available_cols = [c for c in key_cols if c in df.columns]
    summary_df = df[available_cols]

    summary_df.to_csv(output_path, index=False)
    print(f"Summary saved to: {output_path}")

    # Print statistics
    if 'status' in df.columns:
        print(f"\nStatus counts:")
        print(df['status'].value_counts())

    for col in ['planning_latency_ms', 'execution_latency_ms', 'total_latency_ms']:
        if col in df.columns:
            successful = df[df['status'] == 'success'][col]
            if len(successful) > 0:
                print(f"\n{col} (successful trials):")
                print(f"  Mean: {successful.mean():.2f}")
                print(f"  Std:  {successful.std():.2f}")
                print(f"  Min:  {successful.min():.2f}")
                print(f"  Max:  {successful.max():.2f}")


def main():
    parser = argparse.ArgumentParser(description='LDOS Trace Analyzer')
    parser.add_argument('--trace-dir', help='Path to LTTng trace directory')
    parser.add_argument('--result-dir', help='Path to result JSON files')
    parser.add_argument('--output-dir', required=True, help='Output directory for analysis')
    parser.add_argument('--aggregate-only', action='store_true',
                        help='Only aggregate existing results, skip trace analysis')
    parser.add_argument('--control-rate-hz', type=float, default=500.0,
                        help='Expected control loop rate in Hz (default: 500 Hz for ros2_control)')

    args = parser.parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Analyze trace if provided
    if args.trace_dir and not args.aggregate_only:
        analyzer = TraceAnalyzer(args.trace_dir, control_rate_hz=args.control_rate_hz)
        metrics = analyzer.analyze()

        # Save metrics
        metrics_file = output_dir / "trace_metrics.json"
        # Convert to dict without the large list
        metrics_dict = asdict(metrics)
        metrics_dict.pop('callback_durations_ns', None)  # Too large for JSON
        with open(metrics_file, 'w') as f:
            json.dump(metrics_dict, f, indent=2)
        print(f"Trace metrics saved to: {metrics_file}")

    # Aggregate results if provided
    if args.result_dir:
        result_dir = Path(args.result_dir)
        summary_path = output_dir / "summary.csv"
        aggregate_results(result_dir, summary_path)


if __name__ == '__main__':
    main()

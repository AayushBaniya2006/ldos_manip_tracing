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

    def __init__(self, trace_path: str):
        self.trace_path = Path(trace_path)
        self.metrics = TraceMetrics(trace_path=str(self.trace_path))

        # State tracking - use stack per thread for proper nested callback handling
        self._callback_stacks: Dict[int, List[CallbackEvent]] = {}  # vtid -> stack of active callbacks
        self._callback_names: Dict[int, str] = {}  # callback_ptr -> name

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

        # Finalize
        if first_ts and last_ts:
            self.metrics.start_time_ns = first_ts
            self.metrics.end_time_ns = last_ts
            self.metrics.duration_s = (last_ts - first_ts) / 1e9

        self.metrics.total_callbacks = len(self.metrics.callback_durations_ns)
        self.metrics.compute_aggregates()

        print(f"Processed {event_count} events")

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
                break

    def _handle_callback_register(self, event):
        """Handle callback registration event."""
        callback_ptr = event['callback']
        symbol = str(event.get('symbol', 'unknown'))
        self._callback_names[callback_ptr] = symbol


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

    args = parser.parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Analyze trace if provided
    if args.trace_dir and not args.aggregate_only:
        analyzer = TraceAnalyzer(args.trace_dir)
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

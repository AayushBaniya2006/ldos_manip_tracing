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

try:
    import bt2
except ImportError:
    print("ERROR: babeltrace2 not installed. Install with: pip install babeltrace2")
    print("Or: sudo apt install python3-babeltrace2")
    sys.exit(1)

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
        if self.callback_durations_ns:
            durations_ms = [d / 1e6 for d in self.callback_durations_ns]
            self.callback_duration_mean_ms = sum(durations_ms) / len(durations_ms)
            self.callback_duration_p50_ms = sorted(durations_ms)[len(durations_ms) // 2]
            self.callback_duration_p95_ms = sorted(durations_ms)[int(len(durations_ms) * 0.95)]
            self.callback_duration_p99_ms = sorted(durations_ms)[int(len(durations_ms) * 0.99)]
            self.callback_duration_max_ms = max(durations_ms)

            # Compute std dev
            mean = self.callback_duration_mean_ms
            variance = sum((d - mean) ** 2 for d in durations_ms) / len(durations_ms)
            self.callback_duration_std_ms = variance ** 0.5


class TraceAnalyzer:
    """Analyze LTTng traces from ROS 2 applications."""

    def __init__(self, trace_path: str):
        self.trace_path = Path(trace_path)
        self.metrics = TraceMetrics(trace_path=str(self.trace_path))

        # State tracking
        self._active_callbacks: Dict[Tuple[int, int], CallbackEvent] = {}  # (vtid, callback_ptr) -> event
        self._callback_names: Dict[int, str] = {}  # callback_ptr -> name

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
        print(f"Trace duration: {self.metrics.duration_s:.2f}s")
        print(f"Callbacks: {self.metrics.total_callbacks}")

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
        """Handle callback_start event."""
        callback_ptr = event['callback']
        vtid = event.common_context_field.get('vtid', 0) if event.common_context_field else 0
        vpid = event.common_context_field.get('vpid', 0) if event.common_context_field else 0
        procname = str(event.common_context_field.get('procname', '')) if event.common_context_field else ''

        key = (vtid, callback_ptr)
        self._active_callbacks[key] = CallbackEvent(
            callback_ptr=callback_ptr,
            start_ns=ts_ns,
            vpid=vpid,
            vtid=vtid,
            procname=procname
        )

    def _handle_callback_end(self, event, ts_ns: int):
        """Handle callback_end event."""
        callback_ptr = event['callback']
        vtid = event.common_context_field.get('vtid', 0) if event.common_context_field else 0

        key = (vtid, callback_ptr)
        if key in self._active_callbacks:
            cb_event = self._active_callbacks.pop(key)
            cb_event.end_ns = ts_ns
            cb_event.duration_ns = ts_ns - cb_event.start_ns
            self.metrics.callback_durations_ns.append(cb_event.duration_ns)

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

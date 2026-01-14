#!/usr/bin/env python3
"""
e2e_path_analyzer.py - End-to-End Path Analysis for ROS 2 Manipulation Stacks

This script analyzes LTTng traces to:
1. Identify callback chains forming end-to-end paths
2. Compute latency breakdown per callback
3. Find bottleneck callbacks
4. Generate path diagrams and reports

Reference: PiCAS paper (callback chains), Message Flow Analysis paper

Usage:
    python3 e2e_path_analyzer.py --trace-dir traces/baseline_001 --output analysis/output/paths
    python3 e2e_path_analyzer.py --trace-dir traces/ --objectives configs/objectives.yaml

Requirements:
    pip install babeltrace2 pandas pyyaml matplotlib
"""

import argparse
import json
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import warnings

try:
    import bt2
except ImportError:
    print("ERROR: babeltrace2 not installed. Install with: pip install babeltrace2")
    sys.exit(1)

import pandas as pd
import yaml

warnings.filterwarnings('ignore', category=FutureWarning)


# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class CallbackInfo:
    """Information about a single callback execution."""
    callback_ptr: int
    symbol: str
    node_name: str
    start_ns: int
    end_ns: int
    duration_ns: int
    vtid: int
    vpid: int
    procname: str

    @property
    def duration_ms(self) -> float:
        return self.duration_ns / 1e6


@dataclass
class PublishEvent:
    """A message publish event."""
    timestamp_ns: int
    publisher_handle: int
    topic_name: str
    node_name: str
    message_ptr: Optional[int] = None


@dataclass
class SubscriptionEvent:
    """A subscription take event."""
    timestamp_ns: int
    subscription_handle: int
    topic_name: str
    node_name: str
    message_ptr: Optional[int] = None


@dataclass
class CallbackChain:
    """A sequence of callbacks forming an end-to-end path instance."""
    chain_id: str
    path_type: str  # P1_planning, P2_control, etc.
    start_ns: int
    end_ns: int
    callbacks: List[CallbackInfo] = field(default_factory=list)
    publishes: List[PublishEvent] = field(default_factory=list)

    @property
    def total_latency_ns(self) -> int:
        return self.end_ns - self.start_ns

    @property
    def total_latency_ms(self) -> float:
        return self.total_latency_ns / 1e6

    @property
    def callback_latency_ns(self) -> int:
        return sum(cb.duration_ns for cb in self.callbacks)

    @property
    def callback_latency_ms(self) -> float:
        return self.callback_latency_ns / 1e6

    def get_bottleneck(self) -> Optional[CallbackInfo]:
        """Return the callback consuming most time."""
        if not self.callbacks:
            return None
        return max(self.callbacks, key=lambda cb: cb.duration_ns)


@dataclass
class PathStatistics:
    """Aggregated statistics for an end-to-end path."""
    path_type: str
    path_name: str
    n_instances: int

    # Latency stats (ms)
    latency_mean_ms: float = 0.0
    latency_std_ms: float = 0.0
    latency_p50_ms: float = 0.0
    latency_p95_ms: float = 0.0
    latency_p99_ms: float = 0.0
    latency_min_ms: float = 0.0
    latency_max_ms: float = 0.0

    # Callback breakdown
    callback_breakdown: Dict[str, float] = field(default_factory=dict)  # callback_name -> avg_ms

    # Bottleneck info
    bottleneck_callback: str = ""
    bottleneck_percentage: float = 0.0

    # Deadline analysis
    deadline_ms: Optional[float] = None
    deadline_violations: int = 0
    violation_rate: float = 0.0


# =============================================================================
# PATH DEFINITIONS (loaded from objectives.yaml)
# =============================================================================

DEFAULT_PATH_PATTERNS = {
    "P1_planning": {
        "name": "Planning Path",
        "callback_patterns": [
            r".*goal_callback.*",
            r".*planning.*",
            r".*MoveGroupInterface.*",
            r".*plan_execution.*",
        ],
        "node_patterns": [
            r"move_group.*",
        ],
        "deadline_ms": 5000,
    },
    "P2_control": {
        "name": "Control Loop",
        "callback_patterns": [
            r".*controller.*update.*",
            r".*JointTrajectoryController.*",
            r".*ControllerManager.*",
        ],
        "node_patterns": [
            r"controller_manager.*",
            r".*_controller.*",
        ],
        "deadline_ms": 2,
    },
    "P3_feedback": {
        "name": "State Feedback",
        "callback_patterns": [
            r".*joint_state.*",
            r".*JointStateBroadcaster.*",
            r".*robot_state.*",
            r".*tf.*",
        ],
        "node_patterns": [
            r"joint_state_broadcaster.*",
            r"robot_state_publisher.*",
        ],
        "deadline_ms": 10,
    },
    "P4_collision": {
        "name": "Collision Checking",
        "callback_patterns": [
            r".*collision.*",
            r".*planning_scene.*",
            r".*fcl.*",
        ],
        "node_patterns": [
            r"move_group.*",
        ],
        "deadline_ms": 50,
    },
}


# =============================================================================
# ANALYZER CLASS
# =============================================================================

class E2EPathAnalyzer:
    """Analyze end-to-end paths in ROS 2 traces."""

    def __init__(self, trace_path: str, objectives_path: Optional[str] = None):
        self.trace_path = Path(trace_path)
        self.objectives_path = objectives_path

        # Load path definitions
        self.path_patterns = self._load_path_patterns()

        # State tracking during analysis
        self._callback_names: Dict[int, str] = {}  # callback_ptr -> symbol
        self._node_handles: Dict[int, str] = {}    # node_handle -> node_name
        self._publisher_topics: Dict[int, str] = {}  # publisher_handle -> topic
        self._subscription_topics: Dict[int, str] = {}  # subscription_handle -> topic

        # Collected data
        self.callbacks: List[CallbackInfo] = []
        self.publishes: List[PublishEvent] = []
        self.subscriptions: List[SubscriptionEvent] = []

        # Identified chains
        self.chains: Dict[str, List[CallbackChain]] = defaultdict(list)
        self.path_stats: Dict[str, PathStatistics] = {}

    def _load_path_patterns(self) -> Dict:
        """Load path patterns from objectives.yaml or use defaults."""
        if self.objectives_path and Path(self.objectives_path).exists():
            with open(self.objectives_path) as f:
                objectives = yaml.safe_load(f)

            patterns = {}
            for path_id, path_def in objectives.get('paths', {}).items():
                if path_def.get('enabled', True):
                    patterns[path_id] = {
                        'name': path_def.get('name', path_id),
                        'callback_patterns': [
                            cb.get('callback_pattern', '.*')
                            for cb in path_def.get('chain', [])
                        ],
                        'node_patterns': [
                            cb.get('node', '.*')
                            for cb in path_def.get('chain', [])
                        ],
                        'deadline_ms': path_def.get('timing', {}).get('deadline_ms'),
                    }
            return patterns if patterns else DEFAULT_PATH_PATTERNS
        return DEFAULT_PATH_PATTERNS

    def analyze(self) -> Dict[str, PathStatistics]:
        """Run full analysis pipeline."""
        print(f"Analyzing trace: {self.trace_path}")

        # Step 1: Parse trace
        self._parse_trace()

        # Step 2: Identify callback chains
        self._identify_chains()

        # Step 3: Compute statistics
        self._compute_statistics()

        return self.path_stats

    def _parse_trace(self):
        """Parse LTTng trace and extract events."""
        trace_data_path = self._find_trace_data()
        if not trace_data_path:
            print(f"ERROR: No trace data found in {self.trace_path}")
            return

        print(f"Trace data path: {trace_data_path}")

        try:
            msg_it = bt2.TraceCollectionMessageIterator(str(trace_data_path))
        except Exception as e:
            print(f"ERROR: Failed to open trace: {e}")
            return

        active_callbacks: Dict[Tuple[int, int], Tuple[int, int, int, str]] = {}  # (vtid, ptr) -> (start, vpid, vtid, procname)
        event_count = 0

        for msg in msg_it:
            if type(msg) is not bt2._EventMessageConst:
                continue

            event = msg.event
            event_name = event.name
            ts_ns = msg.default_clock_snapshot.ns_from_origin
            event_count += 1

            # Extract common context
            ctx = event.common_context_field
            vtid = ctx.get('vtid', 0) if ctx else 0
            vpid = ctx.get('vpid', 0) if ctx else 0
            procname = str(ctx.get('procname', '')) if ctx else ''

            if event_name == 'ros2:rclcpp_callback_register':
                callback_ptr = event['callback']
                symbol = str(event.get('symbol', f'callback_{callback_ptr:x}'))
                self._callback_names[callback_ptr] = symbol

            elif event_name == 'ros2:rcl_node_init':
                handle = event['node_handle']
                name = str(event.get('node_name', f'node_{handle}'))
                self._node_handles[handle] = name

            elif event_name == 'ros2:rcl_publisher_init':
                handle = event['publisher_handle']
                topic = str(event.get('topic_name', f'topic_{handle}'))
                self._publisher_topics[handle] = topic

            elif event_name == 'ros2:rcl_subscription_init':
                handle = event['subscription_handle']
                topic = str(event.get('topic_name', f'topic_{handle}'))
                self._subscription_topics[handle] = topic

            elif event_name == 'ros2:callback_start':
                callback_ptr = event['callback']
                key = (vtid, callback_ptr)
                active_callbacks[key] = (ts_ns, vpid, vtid, procname)

            elif event_name == 'ros2:callback_end':
                callback_ptr = event['callback']
                key = (vtid, callback_ptr)

                if key in active_callbacks:
                    start_ns, vpid_start, vtid_start, procname_start = active_callbacks.pop(key)
                    symbol = self._callback_names.get(callback_ptr, f'callback_{callback_ptr:x}')

                    # Infer node name from symbol
                    node_name = self._infer_node_name(symbol, procname)

                    cb_info = CallbackInfo(
                        callback_ptr=callback_ptr,
                        symbol=symbol,
                        node_name=node_name,
                        start_ns=start_ns,
                        end_ns=ts_ns,
                        duration_ns=ts_ns - start_ns,
                        vtid=vtid_start,
                        vpid=vpid_start,
                        procname=procname_start
                    )
                    self.callbacks.append(cb_info)

            elif event_name in ('ros2:rcl_publish', 'ros2:rclcpp_publish'):
                pub_handle = event.get('publisher_handle', 0)
                topic = self._publisher_topics.get(pub_handle, f'topic_{pub_handle}')
                msg_ptr = event.get('message', None)

                pub_event = PublishEvent(
                    timestamp_ns=ts_ns,
                    publisher_handle=pub_handle,
                    topic_name=topic,
                    node_name=procname,
                    message_ptr=msg_ptr
                )
                self.publishes.append(pub_event)

        print(f"Processed {event_count} events")
        print(f"Callbacks: {len(self.callbacks)}")
        print(f"Publishes: {len(self.publishes)}")

    def _find_trace_data(self) -> Optional[Path]:
        """Find the actual trace data directory."""
        candidates = [self.trace_path]

        ust_path = self.trace_path / "ust"
        if ust_path.exists():
            candidates.append(ust_path)
            for uid_dir in ust_path.iterdir():
                if uid_dir.is_dir():
                    candidates.append(uid_dir)
                    for subdir in uid_dir.iterdir():
                        if subdir.is_dir():
                            candidates.append(subdir)

        for candidate in candidates:
            if candidate.exists():
                if (candidate / "metadata").exists():
                    return candidate
                for subdir in candidate.rglob("metadata"):
                    return subdir.parent

        return self.trace_path if self.trace_path.exists() else None

    def _infer_node_name(self, symbol: str, procname: str) -> str:
        """Infer node name from callback symbol or process name."""
        # Try to extract from symbol (e.g., "move_group::MoveGroupExe::goalCallback")
        parts = symbol.split('::')
        if len(parts) >= 2:
            return parts[0]
        return procname

    def _identify_chains(self):
        """Identify callback chains based on path patterns."""
        print("\nIdentifying callback chains...")

        # Sort callbacks by start time
        sorted_callbacks = sorted(self.callbacks, key=lambda cb: cb.start_ns)

        for path_id, path_def in self.path_patterns.items():
            callback_patterns = [re.compile(p, re.IGNORECASE) for p in path_def.get('callback_patterns', [])]
            node_patterns = [re.compile(p, re.IGNORECASE) for p in path_def.get('node_patterns', [])]

            # Find callbacks matching this path
            matching_callbacks = []
            for cb in sorted_callbacks:
                # Check callback pattern match
                cb_match = any(p.search(cb.symbol) for p in callback_patterns) if callback_patterns else True
                # Check node pattern match
                node_match = any(p.search(cb.node_name) for p in node_patterns) if node_patterns else True

                if cb_match or node_match:
                    matching_callbacks.append(cb)

            if not matching_callbacks:
                continue

            # Group into chains based on time proximity
            chains = self._group_into_chains(matching_callbacks, path_id, path_def)
            self.chains[path_id] = chains

            print(f"  {path_id} ({path_def['name']}): {len(chains)} instances, {len(matching_callbacks)} callbacks")

    def _group_into_chains(
        self,
        callbacks: List[CallbackInfo],
        path_id: str,
        path_def: Dict
    ) -> List[CallbackChain]:
        """Group callbacks into chains based on temporal proximity."""
        if not callbacks:
            return []

        chains = []
        current_chain = None

        # Grouping threshold: 100ms for planning, 10ms for control
        threshold_ns = 100_000_000  # 100ms default
        if 'control' in path_id.lower():
            threshold_ns = 10_000_000  # 10ms for control loops

        for cb in callbacks:
            if current_chain is None:
                # Start new chain
                chain_id = f"{path_id}_{len(chains):04d}"
                current_chain = CallbackChain(
                    chain_id=chain_id,
                    path_type=path_id,
                    start_ns=cb.start_ns,
                    end_ns=cb.end_ns,
                    callbacks=[cb]
                )
            elif cb.start_ns - current_chain.end_ns < threshold_ns:
                # Add to current chain
                current_chain.callbacks.append(cb)
                current_chain.end_ns = max(current_chain.end_ns, cb.end_ns)
            else:
                # Save current chain and start new one
                chains.append(current_chain)
                chain_id = f"{path_id}_{len(chains):04d}"
                current_chain = CallbackChain(
                    chain_id=chain_id,
                    path_type=path_id,
                    start_ns=cb.start_ns,
                    end_ns=cb.end_ns,
                    callbacks=[cb]
                )

        if current_chain:
            chains.append(current_chain)

        return chains

    def _compute_statistics(self):
        """Compute statistics for each path type."""
        print("\nComputing path statistics...")

        for path_id, chains in self.chains.items():
            if not chains:
                continue

            path_def = self.path_patterns.get(path_id, {})
            latencies_ms = [chain.total_latency_ms for chain in chains]

            # Basic stats
            latencies_sorted = sorted(latencies_ms)
            n = len(latencies_sorted)

            stats = PathStatistics(
                path_type=path_id,
                path_name=path_def.get('name', path_id),
                n_instances=n,
                latency_mean_ms=sum(latencies_ms) / n,
                latency_min_ms=latencies_sorted[0],
                latency_max_ms=latencies_sorted[-1],
                # Safe percentile calculation with bounds checking
                latency_p50_ms=latencies_sorted[min(n // 2, n - 1)],
                latency_p95_ms=latencies_sorted[min(int(n * 0.95), n - 1)],
                latency_p99_ms=latencies_sorted[min(int(n * 0.99), n - 1)],
            )

            # Std dev
            mean = stats.latency_mean_ms
            variance = sum((l - mean) ** 2 for l in latencies_ms) / n
            stats.latency_std_ms = variance ** 0.5

            # Callback breakdown
            callback_totals: Dict[str, List[float]] = defaultdict(list)
            for chain in chains:
                for cb in chain.callbacks:
                    callback_totals[cb.symbol].append(cb.duration_ms)

            stats.callback_breakdown = {
                symbol: sum(durations) / len(durations)
                for symbol, durations in callback_totals.items()
            }

            # Find bottleneck
            if stats.callback_breakdown:
                bottleneck = max(stats.callback_breakdown.items(), key=lambda x: x[1])
                stats.bottleneck_callback = bottleneck[0]
                total_callback_time = sum(stats.callback_breakdown.values())
                if total_callback_time > 0:
                    stats.bottleneck_percentage = (bottleneck[1] / total_callback_time) * 100

            # Deadline analysis
            deadline_ms = path_def.get('deadline_ms')
            if deadline_ms:
                stats.deadline_ms = deadline_ms
                stats.deadline_violations = sum(1 for l in latencies_ms if l > deadline_ms)
                stats.violation_rate = stats.deadline_violations / n

            self.path_stats[path_id] = stats

            print(f"  {path_id}: mean={stats.latency_mean_ms:.2f}ms, "
                  f"p95={stats.latency_p95_ms:.2f}ms, "
                  f"violations={stats.deadline_violations}/{n}")

    def generate_report(self, output_dir: Path) -> Path:
        """Generate analysis report."""
        output_dir.mkdir(parents=True, exist_ok=True)

        # Generate markdown report
        report_path = output_dir / "e2e_path_report.md"

        lines = [
            "# End-to-End Path Analysis Report\n",
            f"\n**Trace:** `{self.trace_path}`\n",
            f"**Total callbacks analyzed:** {len(self.callbacks)}\n",
            "\n## Path Summary\n",
            "\n| Path | Name | Instances | Mean (ms) | P95 (ms) | Max (ms) | Deadline | Violations |\n",
            "|------|------|-----------|-----------|----------|----------|----------|------------|\n",
        ]

        for path_id, stats in self.path_stats.items():
            deadline_str = f"{stats.deadline_ms}ms" if stats.deadline_ms else "-"
            violation_str = f"{stats.deadline_violations} ({stats.violation_rate*100:.1f}%)" if stats.deadline_ms else "-"
            lines.append(
                f"| {path_id} | {stats.path_name} | {stats.n_instances} | "
                f"{stats.latency_mean_ms:.2f} | {stats.latency_p95_ms:.2f} | "
                f"{stats.latency_max_ms:.2f} | {deadline_str} | {violation_str} |\n"
            )

        lines.append("\n## Detailed Analysis\n")

        for path_id, stats in self.path_stats.items():
            lines.append(f"\n### {path_id}: {stats.path_name}\n")
            lines.append(f"\n- **Instances:** {stats.n_instances}\n")
            lines.append(f"- **Latency (ms):** mean={stats.latency_mean_ms:.2f}, "
                        f"std={stats.latency_std_ms:.2f}, p50={stats.latency_p50_ms:.2f}, "
                        f"p95={stats.latency_p95_ms:.2f}, max={stats.latency_max_ms:.2f}\n")

            if stats.deadline_ms:
                lines.append(f"- **Deadline:** {stats.deadline_ms}ms\n")
                lines.append(f"- **Violations:** {stats.deadline_violations} ({stats.violation_rate*100:.1f}%)\n")

            if stats.bottleneck_callback:
                lines.append(f"- **Bottleneck:** `{stats.bottleneck_callback}` ({stats.bottleneck_percentage:.1f}% of callback time)\n")

            if stats.callback_breakdown:
                lines.append("\n**Callback Breakdown (avg ms):**\n")
                lines.append("\n| Callback | Avg (ms) |\n")
                lines.append("|----------|----------|\n")
                for cb_name, avg_ms in sorted(stats.callback_breakdown.items(), key=lambda x: -x[1])[:10]:
                    short_name = cb_name[:60] + "..." if len(cb_name) > 60 else cb_name
                    lines.append(f"| `{short_name}` | {avg_ms:.3f} |\n")

        with open(report_path, 'w') as f:
            f.writelines(lines)

        print(f"\nReport saved to: {report_path}")

        # Save JSON data
        json_path = output_dir / "e2e_path_data.json"
        data = {
            'trace_path': str(self.trace_path),
            'n_callbacks': len(self.callbacks),
            'paths': {
                path_id: {
                    'name': stats.path_name,
                    'n_instances': stats.n_instances,
                    'latency_mean_ms': stats.latency_mean_ms,
                    'latency_std_ms': stats.latency_std_ms,
                    'latency_p50_ms': stats.latency_p50_ms,
                    'latency_p95_ms': stats.latency_p95_ms,
                    'latency_p99_ms': stats.latency_p99_ms,
                    'latency_max_ms': stats.latency_max_ms,
                    'deadline_ms': stats.deadline_ms,
                    'deadline_violations': stats.deadline_violations,
                    'violation_rate': stats.violation_rate,
                    'bottleneck_callback': stats.bottleneck_callback,
                    'bottleneck_percentage': stats.bottleneck_percentage,
                    'callback_breakdown': stats.callback_breakdown,
                }
                for path_id, stats in self.path_stats.items()
            }
        }

        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"Data saved to: {json_path}")

        # Save CSV summary
        csv_path = output_dir / "e2e_path_summary.csv"
        rows = []
        for path_id, stats in self.path_stats.items():
            rows.append({
                'path_id': path_id,
                'path_name': stats.path_name,
                'n_instances': stats.n_instances,
                'latency_mean_ms': stats.latency_mean_ms,
                'latency_std_ms': stats.latency_std_ms,
                'latency_p50_ms': stats.latency_p50_ms,
                'latency_p95_ms': stats.latency_p95_ms,
                'latency_p99_ms': stats.latency_p99_ms,
                'latency_max_ms': stats.latency_max_ms,
                'deadline_ms': stats.deadline_ms,
                'deadline_violations': stats.deadline_violations,
                'violation_rate': stats.violation_rate,
            })

        if rows:
            pd.DataFrame(rows).to_csv(csv_path, index=False)
            print(f"Summary CSV saved to: {csv_path}")

        return report_path

    def generate_mermaid_diagram(self, output_path: Path):
        """Generate Mermaid diagram of identified paths."""
        lines = ["```mermaid", "graph LR"]

        for path_id, stats in self.path_stats.items():
            if not stats.callback_breakdown:
                continue

            # Add subgraph for each path
            lines.append(f"  subgraph {path_id}[\"{stats.path_name}\"]")

            # Add callbacks as nodes
            callbacks = list(stats.callback_breakdown.keys())[:5]  # Top 5 callbacks
            for i, cb in enumerate(callbacks):
                short_name = cb.split('::')[-1][:20] if '::' in cb else cb[:20]
                avg_ms = stats.callback_breakdown[cb]
                lines.append(f"    {path_id}_{i}[\"{short_name}<br/>{avg_ms:.2f}ms\"]")

            # Connect callbacks
            for i in range(len(callbacks) - 1):
                lines.append(f"    {path_id}_{i} --> {path_id}_{i+1}")

            lines.append("  end")

        lines.append("```")

        with open(output_path, 'w') as f:
            f.write('\n'.join(lines))

        print(f"Mermaid diagram saved to: {output_path}")


# =============================================================================
# CLI
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='End-to-End Path Analyzer for ROS 2 Traces',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Analyze a single trace
  python3 e2e_path_analyzer.py --trace-dir traces/baseline_001 --output analysis/output/paths

  # Use custom objectives
  python3 e2e_path_analyzer.py --trace-dir traces/ --objectives configs/objectives.yaml

  # Analyze multiple traces
  for trace in traces/baseline_*; do
    python3 e2e_path_analyzer.py --trace-dir "$trace" --output "analysis/output/$(basename $trace)"
  done
        """
    )

    parser.add_argument('--trace-dir', required=True,
                        help='Path to LTTng trace directory')
    parser.add_argument('--output', '-o', default='analysis/output/paths',
                        help='Output directory for reports')
    parser.add_argument('--objectives',
                        help='Path to objectives.yaml for path definitions')
    parser.add_argument('--format', choices=['all', 'md', 'json', 'csv'], default='all',
                        help='Output format (default: all)')

    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Run analysis
    analyzer = E2EPathAnalyzer(args.trace_dir, args.objectives)
    stats = analyzer.analyze()

    if not stats:
        print("No paths identified. Check trace data and path patterns.")
        return 1

    # Generate outputs
    analyzer.generate_report(output_dir)
    analyzer.generate_mermaid_diagram(output_dir / "path_diagram.md")

    print("\n=== Analysis Complete ===")
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""
find_breaking_point.py - Automatically find system breaking points via binary search

This script performs a binary search to find the parameter value at which
the system starts failing (success rate drops below threshold).

This is more efficient than a full parameter sweep when you just need to
find the breaking point, not characterize the full degradation curve.

Usage:
    python3 find_breaking_point.py --param cpu_load_percent --low 50 --high 100 --threshold 0.9
    python3 find_breaking_point.py --param num_publishers --low 1 --high 100 --threshold 0.95

Requirements:
    pip install pandas pyyaml
"""

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import pandas as pd
import yaml


# =============================================================================
# CONFIGURATION
# =============================================================================

SCRIPT_DIR = Path(__file__).parent
WS_ROOT = SCRIPT_DIR.parent
CONFIG_FILE = WS_ROOT / "configs" / "experiment_config.yaml"


# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class TrialResult:
    """Result of running trials at a specific parameter value."""
    param_value: float
    n_trials: int
    n_success: int
    success_rate: float
    mean_latency_ms: float
    p95_latency_ms: float
    max_latency_ms: float


@dataclass
class SearchResult:
    """Result of binary search."""
    param_name: str
    threshold: float
    breaking_point: Optional[float]
    last_stable_value: float
    first_breaking_value: Optional[float]
    trials: List[TrialResult]
    search_time_s: float


# =============================================================================
# CONFIGURATION MANAGEMENT
# =============================================================================

def load_config() -> dict:
    """Load experiment configuration."""
    with open(CONFIG_FILE) as f:
        return yaml.safe_load(f)


def save_config(config: dict):
    """Save experiment configuration."""
    with open(CONFIG_FILE, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)


def update_config_param(param_name: str, value: float):
    """Update a parameter in the configuration file."""
    config = load_config()

    # Handle nested parameters
    parts = param_name.split('.')
    target = config

    for part in parts[:-1]:
        if part in target:
            target = target[part]
        else:
            # Search in common sections
            for section in ['load_scenarios', 'benchmark', 'tracing']:
                if section in config and part in config[section]:
                    target = config[section][part]
                    break

    final_key = parts[-1]

    # Try to find and update the parameter
    if final_key in target:
        target[final_key] = value
    else:
        # Search in nested dicts
        updated = False
        for key, val in config.items():
            if isinstance(val, dict):
                if final_key in val:
                    config[key][final_key] = value
                    updated = True
                    break
                for subkey, subval in val.items():
                    if isinstance(subval, dict) and final_key in subval:
                        config[key][subkey][final_key] = value
                        updated = True
                        break

        if not updated:
            raise ValueError(f"Parameter {param_name} not found in config")

    save_config(config)


def get_scenario_for_param(param_name: str) -> str:
    """Determine which scenario to use for a parameter."""
    cpu_params = {'cpu_load_percent', 'cpu_workers'}
    msg_params = {'num_publishers', 'rate_hz', 'payload_size_bytes'}

    if param_name in cpu_params:
        return 'cpu_load'
    elif param_name in msg_params:
        return 'msg_load'
    else:
        return 'baseline'


# =============================================================================
# EXPERIMENT EXECUTION
# =============================================================================

def run_trials(
    param_name: str,
    param_value: float,
    num_trials: int = 20,
    scenario: Optional[str] = None
) -> TrialResult:
    """Run trials at a specific parameter value."""
    print(f"\n  Running {num_trials} trials at {param_name}={param_value}")

    # Update configuration
    update_config_param(param_name, param_value)

    # Determine scenario
    if scenario is None:
        scenario = get_scenario_for_param(param_name)

    # Create output directory
    output_dir = WS_ROOT / "results" / f"binsearch_{param_name}_{param_value}"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Run experiment suite
    run_script = WS_ROOT / "scripts" / "run_experiment_suite.sh"

    try:
        result = subprocess.run(
            [str(run_script), str(num_trials), scenario],
            capture_output=True,
            text=True,
            timeout=3600,  # 1 hour timeout
            cwd=str(WS_ROOT)
        )

        if result.returncode != 0:
            print(f"  Warning: Experiment script returned non-zero: {result.returncode}")
            print(f"  stderr: {result.stderr[-500:] if result.stderr else 'none'}")
    except subprocess.TimeoutExpired:
        print(f"  Warning: Experiment timed out")
    except Exception as e:
        print(f"  Warning: Failed to run experiment: {e}")

    # Move results
    scenario_results = WS_ROOT / "results" / scenario
    if scenario_results.exists():
        import shutil
        if output_dir.exists():
            shutil.rmtree(output_dir)
        shutil.move(str(scenario_results), str(output_dir))

    # Analyze results
    return analyze_trial_results(output_dir, param_value)


def analyze_trial_results(result_dir: Path, param_value: float) -> TrialResult:
    """Analyze trial results from a directory."""
    # Find result files
    json_files = list(result_dir.glob("*_result.json"))

    if not json_files:
        print(f"  Warning: No result files found in {result_dir}")
        return TrialResult(
            param_value=param_value,
            n_trials=0,
            n_success=0,
            success_rate=0.0,
            mean_latency_ms=float('inf'),
            p95_latency_ms=float('inf'),
            max_latency_ms=float('inf')
        )

    # Load all results
    rows = []
    for json_file in json_files:
        try:
            with open(json_file) as f:
                data = json.load(f)
            rows.append(data)
        except Exception as e:
            print(f"  Warning: Failed to load {json_file}: {e}")

    if not rows:
        return TrialResult(
            param_value=param_value,
            n_trials=len(json_files),
            n_success=0,
            success_rate=0.0,
            mean_latency_ms=float('inf'),
            p95_latency_ms=float('inf'),
            max_latency_ms=float('inf')
        )

    df = pd.DataFrame(rows)
    n_trials = len(df)
    n_success = len(df[df.get('status', 'success') == 'success']) if 'status' in df.columns else n_trials

    # Compute latency stats (only for successful trials)
    successful = df[df.get('status', 'success') == 'success'] if 'status' in df.columns else df
    latency_col = 'total_latency_ms'

    if latency_col in successful.columns and len(successful) > 0:
        latencies = successful[latency_col].dropna()
        mean_latency = latencies.mean() if len(latencies) > 0 else float('inf')
        p95_latency = latencies.quantile(0.95) if len(latencies) > 0 else float('inf')
        max_latency = latencies.max() if len(latencies) > 0 else float('inf')
    else:
        mean_latency = float('inf')
        p95_latency = float('inf')
        max_latency = float('inf')

    result = TrialResult(
        param_value=param_value,
        n_trials=n_trials,
        n_success=n_success,
        success_rate=n_success / n_trials if n_trials > 0 else 0.0,
        mean_latency_ms=mean_latency,
        p95_latency_ms=p95_latency,
        max_latency_ms=max_latency
    )

    print(f"  Results: {n_success}/{n_trials} success ({result.success_rate*100:.1f}%), "
          f"mean={mean_latency:.1f}ms, p95={p95_latency:.1f}ms")

    return result


# =============================================================================
# BINARY SEARCH
# =============================================================================

def binary_search_breaking_point(
    param_name: str,
    low: float,
    high: float,
    threshold: float = 0.9,
    precision: float = 1.0,
    trials_per_test: int = 20,
) -> SearchResult:
    """
    Binary search to find the parameter value where success rate drops below threshold.

    Args:
        param_name: Name of parameter to sweep
        low: Lower bound of search range
        high: Upper bound of search range
        threshold: Success rate threshold (default 0.9 = 90%)
        precision: Stop when high - low < precision
        trials_per_test: Number of trials to run at each test point

    Returns:
        SearchResult with breaking point and all trial data
    """
    print(f"\n{'='*60}")
    print(f"Binary Search for Breaking Point")
    print(f"{'='*60}")
    print(f"Parameter: {param_name}")
    print(f"Range: [{low}, {high}]")
    print(f"Threshold: {threshold*100:.0f}% success rate")
    print(f"Precision: {precision}")
    print(f"Trials per test: {trials_per_test}")
    print(f"{'='*60}\n")

    start_time = time.time()
    all_trials = []

    last_stable = low
    first_breaking = None

    iteration = 0
    while high - low > precision:
        iteration += 1
        mid = (low + high) / 2

        print(f"\nIteration {iteration}: Testing {param_name}={mid:.1f} (range: [{low:.1f}, {high:.1f}])")

        result = run_trials(param_name, mid, trials_per_test)
        all_trials.append(result)

        if result.success_rate >= threshold:
            # Still stable, search higher
            print(f"  -> Stable ({result.success_rate*100:.1f}% >= {threshold*100:.0f}%), searching higher")
            low = mid
            last_stable = mid
        else:
            # Breaking, search lower
            print(f"  -> Breaking ({result.success_rate*100:.1f}% < {threshold*100:.0f}%), searching lower")
            high = mid
            if first_breaking is None:
                first_breaking = mid

    # Final determination
    breaking_point = first_breaking if first_breaking else None

    elapsed = time.time() - start_time

    print(f"\n{'='*60}")
    print(f"Search Complete")
    print(f"{'='*60}")
    print(f"Last stable value: {last_stable}")
    print(f"First breaking value: {first_breaking}")
    print(f"Breaking point: {breaking_point}")
    print(f"Search time: {elapsed/60:.1f} minutes")
    print(f"Total trials: {sum(t.n_trials for t in all_trials)}")

    return SearchResult(
        param_name=param_name,
        threshold=threshold,
        breaking_point=breaking_point,
        last_stable_value=last_stable,
        first_breaking_value=first_breaking,
        trials=all_trials,
        search_time_s=elapsed
    )


# =============================================================================
# REPORTING
# =============================================================================

def save_results(result: SearchResult, output_dir: Path):
    """Save search results."""
    output_dir.mkdir(parents=True, exist_ok=True)

    # Save JSON
    json_path = output_dir / f"breaking_point_{result.param_name}.json"
    data = {
        'param_name': result.param_name,
        'threshold': result.threshold,
        'breaking_point': result.breaking_point,
        'last_stable_value': result.last_stable_value,
        'first_breaking_value': result.first_breaking_value,
        'search_time_s': result.search_time_s,
        'trials': [asdict(t) for t in result.trials]
    }
    with open(json_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"\nSaved: {json_path}")

    # Save CSV
    csv_path = output_dir / f"breaking_point_{result.param_name}.csv"
    rows = [asdict(t) for t in result.trials]
    pd.DataFrame(rows).to_csv(csv_path, index=False)
    print(f"Saved: {csv_path}")

    # Generate report
    report_path = output_dir / f"breaking_point_{result.param_name}_report.md"
    lines = [
        f"# Breaking Point Analysis: {result.param_name}\n",
        f"\n## Summary\n",
        f"\n- **Parameter:** {result.param_name}\n",
        f"- **Success threshold:** {result.threshold*100:.0f}%\n",
        f"- **Breaking point:** {result.breaking_point}\n",
        f"- **Last stable value:** {result.last_stable_value}\n",
        f"- **Search time:** {result.search_time_s/60:.1f} minutes\n",
        f"\n## Search History\n",
        f"\n| Value | Trials | Success Rate | Mean (ms) | P95 (ms) |\n",
        f"|-------|--------|--------------|-----------|----------|\n",
    ]

    for trial in sorted(result.trials, key=lambda t: t.param_value):
        lines.append(
            f"| {trial.param_value:.1f} | {trial.n_trials} | "
            f"{trial.success_rate*100:.1f}% | {trial.mean_latency_ms:.1f} | "
            f"{trial.p95_latency_ms:.1f} |\n"
        )

    lines.append(f"\n## Recommendations\n")
    if result.breaking_point:
        safe_margin = result.last_stable_value * 0.9
        lines.append(f"\n- **Safe operating value:** < {safe_margin:.1f}\n")
        lines.append(f"- **Maximum recommended:** {result.last_stable_value:.1f}\n")
        lines.append(f"- **Avoid values above:** {result.breaking_point:.1f}\n")
    else:
        lines.append(f"\n- System remained stable throughout tested range\n")
        lines.append(f"- Consider testing higher values\n")

    with open(report_path, 'w') as f:
        f.writelines(lines)
    print(f"Saved: {report_path}")


# =============================================================================
# CLI
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Find system breaking point via binary search',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Find CPU load breaking point
  python3 find_breaking_point.py --param cpu_load_percent --low 50 --high 100

  # Find message publisher breaking point with 95% threshold
  python3 find_breaking_point.py --param num_publishers --low 1 --high 100 --threshold 0.95

  # Faster search with fewer trials
  python3 find_breaking_point.py --param cpu_load_percent --low 50 --high 100 --trials 10
        """
    )

    parser.add_argument('--param', '-p', required=True,
                        help='Parameter name to sweep')
    parser.add_argument('--low', '-l', type=float, required=True,
                        help='Lower bound of search range')
    parser.add_argument('--high', '-H', type=float, required=True,
                        help='Upper bound of search range')
    parser.add_argument('--threshold', '-t', type=float, default=0.9,
                        help='Success rate threshold (default: 0.9 = 90%%)')
    parser.add_argument('--precision', type=float, default=1.0,
                        help='Search precision (default: 1.0)')
    parser.add_argument('--trials', type=int, default=20,
                        help='Trials per test point (default: 20)')
    parser.add_argument('--output', '-o', default='analysis/output',
                        help='Output directory')

    args = parser.parse_args()

    # Backup original config
    original_config = load_config()

    try:
        result = binary_search_breaking_point(
            param_name=args.param,
            low=args.low,
            high=args.high,
            threshold=args.threshold,
            precision=args.precision,
            trials_per_test=args.trials
        )

        save_results(result, Path(args.output))

        print(f"\n{'='*60}")
        print(f"RESULT: Breaking point for {args.param} = {result.breaking_point}")
        print(f"{'='*60}")

        return 0 if result.breaking_point else 1

    finally:
        # Restore original config
        save_config(original_config)
        print("\nRestored original configuration")


if __name__ == '__main__':
    sys.exit(main())

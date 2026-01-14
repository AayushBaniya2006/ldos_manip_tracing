#!/usr/bin/env python3
"""
sweep_analysis.py - Analyze and visualize parameter sweep results

This script processes results from parameter_sweep.sh to:
1. Load results from multiple sweep levels
2. Plot latency vs parameter curves with confidence intervals
3. Identify "knee" points where performance degrades
4. Generate comparison reports and LaTeX tables

Usage:
    python3 sweep_analysis.py --param cpu_load_percent --results results/cpu_*pct --output analysis/output/
    python3 sweep_analysis.py --results results/sweep_* --output sweep_report/

Requirements:
    pip install pandas matplotlib numpy scipy
"""

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Set matplotlib backend BEFORE importing pyplot
# This ensures compatibility with headless CloudLab nodes
import matplotlib
if not os.environ.get('DISPLAY'):
    matplotlib.use('Agg')  # Non-interactive backend for headless operation

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

# Try to import seaborn for nicer plots
try:
    import seaborn as sns
    sns.set_theme(style="whitegrid")
    HAS_SEABORN = True
except ImportError:
    HAS_SEABORN = False


# =============================================================================
# CONFIGURATION
# =============================================================================

COLORS = {
    'primary': '#2196F3',
    'secondary': '#FF9800',
    'success': '#4CAF50',
    'warning': '#FFC107',
    'error': '#F44336',
}

METRICS = [
    'planning_latency_ms',
    'execution_latency_ms',
    'total_latency_ms',
]

# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class SweepPoint:
    """Data for one sweep level."""
    param_value: float
    n_trials: int
    n_success: int
    success_rate: float

    # Latency statistics for each metric
    latency_stats: Dict[str, Dict[str, float]]  # metric -> {mean, std, p50, p95, p99, min, max}


@dataclass
class SweepResults:
    """Complete sweep results."""
    param_name: str
    param_values: List[float]
    points: List[SweepPoint]

    # Computed analysis
    knee_point: Optional[float] = None
    breaking_point: Optional[float] = None


# =============================================================================
# ANALYSIS FUNCTIONS
# =============================================================================

def load_sweep_results(
    result_dirs: List[str],
    param_name: Optional[str] = None
) -> SweepResults:
    """Load results from multiple sweep directories."""

    points = []
    param_values = []

    for result_dir in result_dirs:
        result_path = Path(result_dir)
        if not result_path.exists():
            print(f"Warning: {result_dir} does not exist, skipping")
            continue

        # Try to extract parameter value from directory name
        param_value = extract_param_value(result_path.name, param_name)
        if param_value is None:
            print(f"Warning: Could not extract param value from {result_path.name}")
            continue

        # Find summary CSV
        summary_files = list(result_path.glob("*summary*.csv")) + list(result_path.glob("*.csv"))
        if not summary_files:
            # Try to aggregate from JSON files
            json_files = list(result_path.glob("*_result.json"))
            if json_files:
                df = aggregate_json_results(json_files)
            else:
                print(f"Warning: No results found in {result_dir}")
                continue
        else:
            df = pd.read_csv(summary_files[0])

        if df.empty:
            continue

        # Compute statistics
        point = compute_sweep_point(df, param_value)
        points.append(point)
        param_values.append(param_value)

    # Sort by parameter value
    sorted_indices = np.argsort(param_values)
    points = [points[i] for i in sorted_indices]
    param_values = [param_values[i] for i in sorted_indices]

    # Infer param name if not provided
    if param_name is None:
        param_name = infer_param_name(result_dirs[0] if result_dirs else "")

    return SweepResults(
        param_name=param_name,
        param_values=param_values,
        points=points
    )


def extract_param_value(dirname: str, param_name: Optional[str] = None) -> Optional[float]:
    """Extract parameter value from directory name."""
    # Try common patterns
    patterns = [
        r'_(\d+(?:\.\d+)?)(?:pct|percent)?$',  # cpu_load_75pct, cpu_load_75
        r'_(\d+(?:\.\d+)?)(?:hz)?$',            # rate_1000hz, rate_1000
        r'_(\d+(?:\.\d+)?)(?:pub)?s?$',         # num_publishers_10, pubs_10
        r'(\d+(?:\.\d+)?)$',                    # trailing number
    ]

    for pattern in patterns:
        match = re.search(pattern, dirname, re.IGNORECASE)
        if match:
            return float(match.group(1))

    return None


def infer_param_name(dirname: str) -> str:
    """Infer parameter name from directory name."""
    # Remove trailing numbers and common suffixes
    cleaned = re.sub(r'_\d+(?:pct|percent|hz|pub|pubs)?$', '', dirname, flags=re.IGNORECASE)
    cleaned = re.sub(r'sweep_', '', cleaned, flags=re.IGNORECASE)
    return cleaned or "parameter"


def aggregate_json_results(json_files: List[Path]) -> pd.DataFrame:
    """Aggregate results from multiple JSON files."""
    rows = []
    for json_file in json_files:
        try:
            with open(json_file) as f:
                data = json.load(f)
            rows.append(data)
        except Exception as e:
            print(f"Warning: Failed to load {json_file}: {e}")

    return pd.DataFrame(rows) if rows else pd.DataFrame()


def compute_sweep_point(df: pd.DataFrame, param_value: float) -> SweepPoint:
    """Compute statistics for one sweep level."""
    n_trials = len(df)

    # Fix M3: Properly check if status column exists before filtering
    if 'status' in df.columns:
        n_success = len(df[df['status'] == 'success'])
        successful_df = df[df['status'] == 'success']
    else:
        n_success = n_trials
        successful_df = df

    success_rate = n_success / n_trials if n_trials > 0 else 0

    latency_stats = {}

    for metric in METRICS:
        if metric in df.columns:
            # Fix M4: Use successful trials only and handle NaN
            data = successful_df[metric].dropna()
            if len(data) > 0:
                sorted_data = np.sort(data)
                n = len(sorted_data)
                # Safe percentile calculation with bounds checking (like C5 fix)
                latency_stats[metric] = {
                    'mean': float(data.mean()),
                    'std': float(data.std()) if n > 1 else 0.0,
                    'p50': float(sorted_data[min(n // 2, n - 1)]),
                    'p95': float(sorted_data[min(int(n * 0.95), n - 1)]),
                    'p99': float(sorted_data[min(int(n * 0.99), n - 1)]),
                    'min': float(data.min()),
                    'max': float(data.max()),
                }

    return SweepPoint(
        param_value=param_value,
        n_trials=n_trials,
        n_success=n_success,
        success_rate=success_rate,
        latency_stats=latency_stats
    )


def find_knee_point(values: List[float], latencies: List[float]) -> Optional[float]:
    """
    Find the "knee" point where latency starts increasing rapidly.

    Uses the "elbow method" - finds the point of maximum curvature.
    """
    if len(values) < 3:
        return None

    # Normalize data
    x = np.array(values)
    y = np.array(latencies)

    x_norm = (x - x.min()) / (x.max() - x.min() + 1e-10)
    y_norm = (y - y.min()) / (y.max() - y.min() + 1e-10)

    # Calculate distance from each point to the line connecting first and last points
    # The knee is where this distance is maximum
    line_vec = np.array([x_norm[-1] - x_norm[0], y_norm[-1] - y_norm[0]])
    line_len = np.sqrt(line_vec[0]**2 + line_vec[1]**2)

    if line_len < 1e-10:
        return None

    line_vec = line_vec / line_len

    distances = []
    for i in range(len(x_norm)):
        point_vec = np.array([x_norm[i] - x_norm[0], y_norm[i] - y_norm[0]])
        proj_len = np.dot(point_vec, line_vec)
        proj = proj_len * line_vec
        perp = point_vec - proj
        distances.append(np.sqrt(perp[0]**2 + perp[1]**2))

    knee_idx = np.argmax(distances)
    return values[knee_idx]


def find_breaking_point(
    values: List[float],
    success_rates: List[float],
    threshold: float = 0.9
) -> Optional[float]:
    """Find the parameter value where success rate drops below threshold."""
    for i, (val, rate) in enumerate(zip(values, success_rates)):
        if rate < threshold:
            return val
    return None


# =============================================================================
# PLOTTING FUNCTIONS
# =============================================================================

def plot_latency_curves(
    results: SweepResults,
    output_dir: Path,
    metric: str = 'total_latency_ms'
):
    """Plot latency vs parameter value with confidence intervals."""
    fig, ax = plt.subplots(figsize=(10, 6))

    values = results.param_values
    means = []
    stds = []
    p95s = []

    for point in results.points:
        if metric in point.latency_stats:
            means.append(point.latency_stats[metric]['mean'])
            stds.append(point.latency_stats[metric]['std'])
            p95s.append(point.latency_stats[metric]['p95'])
        else:
            means.append(np.nan)
            stds.append(np.nan)
            p95s.append(np.nan)

    means = np.array(means)
    stds = np.array(stds)
    p95s = np.array(p95s)

    # Plot mean with confidence interval
    ax.plot(values, means, 'o-', color=COLORS['primary'], linewidth=2, label='Mean')
    ax.fill_between(values, means - stds, means + stds, alpha=0.2, color=COLORS['primary'])

    # Plot P95
    ax.plot(values, p95s, 's--', color=COLORS['secondary'], linewidth=1.5, label='P95')

    # Mark knee point if found
    knee = find_knee_point(values, list(means))
    if knee:
        knee_idx = values.index(knee) if knee in values else -1
        if knee_idx >= 0:
            ax.axvline(x=knee, color=COLORS['warning'], linestyle=':', linewidth=2, label=f'Knee ({knee})')

    ax.set_xlabel(results.param_name.replace('_', ' ').title(), fontsize=12)
    ax.set_ylabel(f'{metric.replace("_", " ").title()}', fontsize=12)
    ax.set_title(f'Latency vs {results.param_name.replace("_", " ").title()}', fontsize=14)
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / f'sweep_{metric}.png', dpi=150, bbox_inches='tight')
    plt.savefig(output_dir / f'sweep_{metric}.pdf', bbox_inches='tight')
    plt.close()

    print(f"Saved: sweep_{metric}.png")


def plot_success_rate(results: SweepResults, output_dir: Path):
    """Plot success rate vs parameter value."""
    fig, ax = plt.subplots(figsize=(10, 6))

    values = results.param_values
    rates = [point.success_rate for point in results.points]

    ax.plot(values, rates, 'o-', color=COLORS['success'], linewidth=2, markersize=8)

    # Threshold lines
    ax.axhline(y=0.9, color=COLORS['warning'], linestyle='--', label='90% threshold')
    ax.axhline(y=0.8, color=COLORS['error'], linestyle='--', label='80% threshold')

    # Mark breaking point
    breaking = find_breaking_point(values, rates)
    if breaking:
        ax.axvline(x=breaking, color=COLORS['error'], linestyle=':', linewidth=2, label=f'Breaking ({breaking})')

    ax.set_xlabel(results.param_name.replace('_', ' ').title(), fontsize=12)
    ax.set_ylabel('Success Rate', fontsize=12)
    ax.set_title(f'Success Rate vs {results.param_name.replace("_", " ").title()}', fontsize=14)
    ax.set_ylim(0, 1.05)
    ax.legend(loc='lower left')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / 'sweep_success_rate.png', dpi=150, bbox_inches='tight')
    plt.savefig(output_dir / 'sweep_success_rate.pdf', bbox_inches='tight')
    plt.close()

    print("Saved: sweep_success_rate.png")


def plot_degradation_heatmap(results: SweepResults, output_dir: Path):
    """Plot heatmap showing degradation across metrics."""
    # Build data matrix
    metrics_to_plot = [m for m in METRICS if any(m in p.latency_stats for p in results.points)]

    if not metrics_to_plot:
        return

    data = []
    for point in results.points:
        row = []
        for metric in metrics_to_plot:
            if metric in point.latency_stats:
                row.append(point.latency_stats[metric]['mean'])
            else:
                row.append(np.nan)
        data.append(row)

    data = np.array(data)

    # Normalize each metric to show relative change
    baseline = data[0, :]
    normalized = (data / baseline - 1) * 100  # Percent change from baseline

    fig, ax = plt.subplots(figsize=(10, 6))

    im = ax.imshow(normalized.T, aspect='auto', cmap='RdYlGn_r')

    ax.set_xticks(range(len(results.param_values)))
    ax.set_xticklabels([str(int(v)) if v == int(v) else f'{v:.1f}' for v in results.param_values])
    ax.set_yticks(range(len(metrics_to_plot)))
    ax.set_yticklabels([m.replace('_', ' ').replace(' ms', '') for m in metrics_to_plot])

    ax.set_xlabel(results.param_name.replace('_', ' ').title())
    ax.set_title('Latency Degradation (% change from baseline)')

    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('% Change')

    plt.tight_layout()
    plt.savefig(output_dir / 'sweep_degradation_heatmap.png', dpi=150, bbox_inches='tight')
    plt.close()

    print("Saved: sweep_degradation_heatmap.png")


# =============================================================================
# REPORT GENERATION
# =============================================================================

def generate_report(results: SweepResults, output_dir: Path):
    """Generate markdown report."""
    report_path = output_dir / 'sweep_report.md'

    lines = [
        f"# Parameter Sweep Report: {results.param_name}\n",
        f"\n## Summary\n",
        f"\n- **Parameter:** {results.param_name}\n",
        f"- **Values tested:** {results.param_values}\n",
        f"- **Total trials:** {sum(p.n_trials for p in results.points)}\n",
    ]

    # Find key points
    values = results.param_values
    success_rates = [p.success_rate for p in results.points]
    means = [p.latency_stats.get('total_latency_ms', {}).get('mean', np.nan) for p in results.points]

    knee = find_knee_point(values, means)
    breaking = find_breaking_point(values, success_rates)

    if knee:
        lines.append(f"- **Knee point:** {knee}\n")
    if breaking:
        lines.append(f"- **Breaking point:** {breaking} (success rate < 90%)\n")

    # Results table
    lines.append("\n## Results by Parameter Value\n")
    lines.append("\n| Value | Trials | Success | Mean (ms) | Std (ms) | P95 (ms) | Max (ms) |\n")
    lines.append("|-------|--------|---------|-----------|----------|----------|----------|\n")

    for point in results.points:
        stats = point.latency_stats.get('total_latency_ms', {})
        lines.append(
            f"| {point.param_value} | {point.n_trials} | {point.success_rate*100:.1f}% | "
            f"{stats.get('mean', 0):.1f} | {stats.get('std', 0):.1f} | "
            f"{stats.get('p95', 0):.1f} | {stats.get('max', 0):.1f} |\n"
        )

    # Recommendations
    lines.append("\n## Recommendations\n")

    if knee:
        lines.append(f"\n- **Safe operating range:** {results.param_values[0]} to {knee}\n")
        lines.append(f"- **Performance degrades significantly after:** {knee}\n")

    if breaking:
        lines.append(f"- **Avoid values above:** {breaking} (causes failures)\n")

    lines.append("\n## Figures\n")
    lines.append("\n- `sweep_total_latency_ms.png` - Latency vs parameter\n")
    lines.append("- `sweep_success_rate.png` - Success rate vs parameter\n")
    lines.append("- `sweep_degradation_heatmap.png` - Degradation heatmap\n")

    with open(report_path, 'w') as f:
        f.writelines(lines)

    print(f"Saved: {report_path}")


def generate_latex_table(results: SweepResults, output_dir: Path):
    """Generate LaTeX table."""
    latex_path = output_dir / 'sweep_table.tex'

    lines = [
        "\\begin{table}[h]\n",
        "\\centering\n",
        f"\\caption{{Parameter Sweep: {results.param_name.replace('_', ' ')}}}\n",
        "\\begin{tabular}{rrrrrr}\n",
        "\\toprule\n",
        "Value & Trials & Success (\\%) & Mean (ms) & P95 (ms) & Max (ms) \\\\\n",
        "\\midrule\n",
    ]

    for point in results.points:
        stats = point.latency_stats.get('total_latency_ms', {})
        lines.append(
            f"{point.param_value} & {point.n_trials} & {point.success_rate*100:.1f} & "
            f"{stats.get('mean', 0):.1f} & {stats.get('p95', 0):.1f} & {stats.get('max', 0):.1f} \\\\\n"
        )

    lines.extend([
        "\\bottomrule\n",
        "\\end{tabular}\n",
        "\\end{table}\n",
    ])

    with open(latex_path, 'w') as f:
        f.writelines(lines)

    print(f"Saved: {latex_path}")


# =============================================================================
# CLI
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Analyze parameter sweep results',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 sweep_analysis.py --param cpu_load_percent --results results/cpu_load_* --output sweep_report/
  python3 sweep_analysis.py --results results/sweep_cpu_* --output analysis/output/sweeps/
        """
    )

    parser.add_argument('--param', '-p',
                        help='Parameter name (auto-detected if not provided)')
    parser.add_argument('--results', '-r', nargs='+', required=True,
                        help='Result directories to analyze')
    parser.add_argument('--output', '-o', default='analysis/output/sweeps',
                        help='Output directory')

    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("Loading sweep results...")
    results = load_sweep_results(args.results, args.param)

    if not results.points:
        print("ERROR: No valid results found")
        return 1

    print(f"\nLoaded {len(results.points)} sweep levels")
    print(f"Parameter: {results.param_name}")
    print(f"Values: {results.param_values}")

    # Generate plots
    print("\nGenerating plots...")
    for metric in METRICS:
        if any(metric in p.latency_stats for p in results.points):
            plot_latency_curves(results, output_dir, metric)

    plot_success_rate(results, output_dir)
    plot_degradation_heatmap(results, output_dir)

    # Generate reports
    print("\nGenerating reports...")
    generate_report(results, output_dir)
    generate_latex_table(results, output_dir)

    # Save JSON data
    json_path = output_dir / 'sweep_data.json'
    data = {
        'param_name': results.param_name,
        'param_values': results.param_values,
        'points': [
            {
                'param_value': p.param_value,
                'n_trials': p.n_trials,
                'success_rate': p.success_rate,
                'latency_stats': p.latency_stats,
            }
            for p in results.points
        ]
    }
    with open(json_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Saved: {json_path}")

    print(f"\n=== Sweep Analysis Complete ===")
    print(f"Output: {output_dir}")

    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""
plot_results.py - Generate visualizations for LDOS experiment results

Usage:
    python3 plot_results.py --input combined_summary.csv --output-dir ./plots
"""

import argparse
import sys
from pathlib import Path

try:
    import pandas as pd
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
except ImportError as e:
    print(f"ERROR: Missing dependency: {e}")
    print("Install with: pip install pandas matplotlib")
    sys.exit(1)

# Style configuration
plt.style.use('seaborn-v0_8-whitegrid')
COLORS = {
    'baseline': '#2ecc71',
    'cpu_load': '#e74c3c',
    'msg_load': '#3498db',
}


def load_data(filepath: str) -> pd.DataFrame:
    """Load and validate CSV data."""
    df = pd.read_csv(filepath)

    # Filter to successful trials only for latency analysis
    if 'status' in df.columns:
        df_success = df[df['status'] == 'success'].copy()
        print(f"Loaded {len(df)} trials, {len(df_success)} successful")
        return df_success
    return df


def plot_latency_boxplot(df: pd.DataFrame, output_dir: Path):
    """Generate box plots comparing latencies across scenarios."""
    metrics = ['planning_latency_ms', 'execution_latency_ms', 'total_latency_ms']
    labels = ['T1: Planning', 'T2: Execution', 'T4: Total E2E']

    fig, axes = plt.subplots(1, 3, figsize=(14, 5))

    for ax, metric, label in zip(axes, metrics, labels):
        if metric not in df.columns:
            continue

        data = []
        scenarios = []
        for scenario in ['baseline', 'cpu_load', 'msg_load']:
            scenario_data = df[df['scenario'] == scenario][metric].dropna()
            if len(scenario_data) > 0:
                data.append(scenario_data.values)
                scenarios.append(scenario)

        if data:
            bp = ax.boxplot(data, labels=scenarios, patch_artist=True)
            for patch, scenario in zip(bp['boxes'], scenarios):
                patch.set_facecolor(COLORS.get(scenario, '#999999'))
                patch.set_alpha(0.7)

        ax.set_title(label)
        ax.set_ylabel('Latency (ms)')
        ax.set_xlabel('Scenario')

    plt.tight_layout()
    plt.savefig(output_dir / 'latency_boxplot.png', dpi=150, bbox_inches='tight')
    plt.savefig(output_dir / 'latency_boxplot.pdf', bbox_inches='tight')
    plt.close()
    print(f"Saved: latency_boxplot.png/pdf")


def plot_latency_histogram(df: pd.DataFrame, output_dir: Path):
    """Generate histograms of latency distributions."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    metrics = [
        ('planning_latency_ms', 'T1: Planning Latency'),
        ('execution_latency_ms', 'T2: Execution Latency'),
        ('total_latency_ms', 'T4: Total E2E Latency'),
    ]

    for ax, (metric, title) in zip(axes.flat[:3], metrics):
        if metric not in df.columns:
            continue

        for scenario in ['baseline', 'cpu_load', 'msg_load']:
            data = df[df['scenario'] == scenario][metric].dropna()
            if len(data) > 0:
                ax.hist(data, bins=20, alpha=0.5, label=scenario,
                       color=COLORS.get(scenario, '#999999'))

        ax.set_title(title)
        ax.set_xlabel('Latency (ms)')
        ax.set_ylabel('Count')
        ax.legend()

    # Use the 4th subplot for summary stats
    ax = axes.flat[3]
    ax.axis('off')

    # Create summary table
    summary_text = "Summary Statistics\n" + "=" * 40 + "\n\n"
    for scenario in ['baseline', 'cpu_load', 'msg_load']:
        scenario_df = df[df['scenario'] == scenario]
        if len(scenario_df) > 0:
            summary_text += f"{scenario.upper()}:\n"
            for metric in ['planning_latency_ms', 'total_latency_ms']:
                if metric in scenario_df.columns:
                    mean = scenario_df[metric].mean()
                    std = scenario_df[metric].std()
                    summary_text += f"  {metric}: {mean:.1f} ± {std:.1f} ms\n"
            summary_text += "\n"

    ax.text(0.1, 0.9, summary_text, transform=ax.transAxes,
            fontfamily='monospace', fontsize=10, verticalalignment='top')

    plt.tight_layout()
    plt.savefig(output_dir / 'latency_histogram.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: latency_histogram.png")


def plot_scenario_comparison(df: pd.DataFrame, output_dir: Path):
    """Generate bar chart comparing mean latencies with error bars."""
    scenarios = ['baseline', 'cpu_load', 'msg_load']
    metrics = ['planning_latency_ms', 'execution_latency_ms', 'total_latency_ms']
    metric_labels = ['Planning', 'Execution', 'Total']

    fig, ax = plt.subplots(figsize=(10, 6))

    x = range(len(scenarios))
    width = 0.25

    for i, (metric, label) in enumerate(zip(metrics, metric_labels)):
        if metric not in df.columns:
            continue

        means = []
        stds = []
        for scenario in scenarios:
            data = df[df['scenario'] == scenario][metric].dropna()
            means.append(data.mean() if len(data) > 0 else 0)
            stds.append(data.std() if len(data) > 0 else 0)

        offset = (i - 1) * width
        bars = ax.bar([xi + offset for xi in x], means, width,
                     yerr=stds, label=label, capsize=3)

    ax.set_xlabel('Scenario')
    ax.set_ylabel('Latency (ms)')
    ax.set_title('Latency Comparison Across Scenarios')
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios)
    ax.legend()

    plt.tight_layout()
    plt.savefig(output_dir / 'scenario_comparison.png', dpi=150, bbox_inches='tight')
    plt.savefig(output_dir / 'scenario_comparison.pdf', bbox_inches='tight')
    plt.close()
    print(f"Saved: scenario_comparison.png/pdf")


def plot_degradation_analysis(df: pd.DataFrame, output_dir: Path):
    """Analyze and plot degradation from baseline."""
    baseline = df[df['scenario'] == 'baseline']

    if len(baseline) == 0:
        print("No baseline data for degradation analysis")
        return

    metrics = ['planning_latency_ms', 'execution_latency_ms', 'total_latency_ms']
    metric_labels = ['T1: Planning', 'T2: Execution', 'T4: Total']

    fig, ax = plt.subplots(figsize=(10, 6))

    # Compute percent increase from baseline
    baseline_means = {m: baseline[m].mean() for m in metrics if m in baseline.columns}

    scenarios = ['cpu_load', 'msg_load']
    x = range(len(metrics))
    width = 0.35

    for i, scenario in enumerate(scenarios):
        scenario_df = df[df['scenario'] == scenario]
        if len(scenario_df) == 0:
            continue

        pct_increases = []
        for metric in metrics:
            if metric in scenario_df.columns and metric in baseline_means:
                scenario_mean = scenario_df[metric].mean()
                baseline_mean = baseline_means[metric]
                if baseline_mean > 0:
                    pct = ((scenario_mean - baseline_mean) / baseline_mean) * 100
                else:
                    pct = 0
                pct_increases.append(pct)
            else:
                pct_increases.append(0)

        offset = (i - 0.5) * width
        bars = ax.bar([xi + offset for xi in x], pct_increases, width,
                     label=scenario, color=COLORS.get(scenario, '#999999'))

        # Add value labels on bars
        for bar, val in zip(bars, pct_increases):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                   f'{val:.1f}%', ha='center', va='bottom', fontsize=9)

    ax.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax.set_xlabel('Metric')
    ax.set_ylabel('Percent Increase from Baseline (%)')
    ax.set_title('Latency Degradation Under Load')
    ax.set_xticks(x)
    ax.set_xticklabels(metric_labels)
    ax.legend()

    plt.tight_layout()
    plt.savefig(output_dir / 'degradation_analysis.png', dpi=150, bbox_inches='tight')
    plt.savefig(output_dir / 'degradation_analysis.pdf', bbox_inches='tight')
    plt.close()
    print(f"Saved: degradation_analysis.png/pdf")


def generate_latex_table(df: pd.DataFrame, output_dir: Path):
    """Generate LaTeX table for paper."""
    metrics = ['planning_latency_ms', 'execution_latency_ms', 'total_latency_ms']

    rows = []
    for scenario in ['baseline', 'cpu_load', 'msg_load']:
        scenario_df = df[df['scenario'] == scenario]
        if len(scenario_df) == 0:
            continue

        row = [scenario.replace('_', ' ').title()]
        for metric in metrics:
            if metric in scenario_df.columns:
                mean = scenario_df[metric].mean()
                std = scenario_df[metric].std()
                row.append(f"{mean:.1f} $\\pm$ {std:.1f}")
            else:
                row.append("-")
        row.append(str(len(scenario_df)))
        rows.append(row)

    # Generate LaTeX
    latex = """\\begin{table}[htbp]
\\centering
\\caption{Latency measurements across scenarios (ms)}
\\label{tab:latency}
\\begin{tabular}{lcccc}
\\toprule
Scenario & T1: Planning & T2: Execution & T4: Total & N \\\\
\\midrule
"""
    for row in rows:
        latex += " & ".join(row) + " \\\\\n"

    latex += """\\bottomrule
\\end{tabular}
\\end{table}
"""

    with open(output_dir / 'latency_table.tex', 'w') as f:
        f.write(latex)
    print(f"Saved: latency_table.tex")


def main():
    parser = argparse.ArgumentParser(description='Plot LDOS experiment results')
    parser.add_argument('--input', required=True, help='Path to combined_summary.csv')
    parser.add_argument('--output-dir', required=True, help='Output directory for plots')

    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading data from: {args.input}")
    df = load_data(args.input)

    if len(df) == 0:
        print("ERROR: No data to plot")
        sys.exit(1)

    print(f"\nGenerating plots...")
    plot_latency_boxplot(df, output_dir)
    plot_latency_histogram(df, output_dir)
    plot_scenario_comparison(df, output_dir)
    plot_degradation_analysis(df, output_dir)
    generate_latex_table(df, output_dir)

    print(f"\nAll plots saved to: {output_dir}")


if __name__ == '__main__':
    main()

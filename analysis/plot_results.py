#!/usr/bin/env python3
"""
plot_results.py - Generate visualizations for LDOS experiment results

Usage:
    python3 plot_results.py --input combined_summary.csv --output-dir ./plots
    python3 plot_results.py --results-dir results --output-dir ./plots --cpu-plots
"""

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Any, Optional

try:
    import numpy as np
    import pandas as pd
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
except ImportError as e:
    print(f"ERROR: Missing dependency: {e}")
    print("Install with: pip install pandas matplotlib numpy")
    sys.exit(1)

try:
    from scipy import stats as scipy_stats
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("Warning: scipy not installed. Some statistical functions unavailable.")

# Style configuration
plt.style.use('seaborn-v0_8-whitegrid')
COLORS = {
    'baseline': '#2ecc71',
    'cpu_load': '#e74c3c',
    'msg_load': '#3498db',
    'cpuset_limited': '#9b59b6',
}


# =============================================================================
# DATA CLASSES FOR CPU ANALYSIS
# =============================================================================

@dataclass
class TrialData:
    """Complete trial data including CPU samples."""
    trial_id: str
    scenario: str
    status: str
    cpu_percent_samples: List[float] = field(default_factory=list)
    cpu_percent_mean: float = 0.0
    cpu_percent_max: float = 0.0
    planning_latency_ms: float = 0.0
    execution_latency_ms: float = 0.0
    total_latency_ms: float = 0.0
    sample_rate_hz: float = 10.0  # Default from benchmark_runner
    duration_s: float = 0.0
    # Per-core CPU utilization (for cpuset verification)
    per_core_cpu_samples: List[List[float]] = field(default_factory=list)
    per_core_cpu_mean: List[float] = field(default_factory=list)
    per_core_cpu_max: List[float] = field(default_factory=list)
    active_cores: List[int] = field(default_factory=list)


# =============================================================================
# DATA LOADING FUNCTIONS
# =============================================================================

def load_json_trials(
    results_dir: Path,
    scenarios: Optional[List[str]] = None
) -> Dict[str, List[TrialData]]:
    """
    Load trial data from individual JSON files.

    Args:
        results_dir: Path to results/ directory
        scenarios: List of scenarios to load (default: all found)

    Returns:
        Dict mapping scenario name to list of TrialData objects
    """
    results_dir = Path(results_dir)

    if scenarios is None:
        scenarios = ['baseline', 'cpu_load', 'msg_load', 'cpuset_limited']

    trials_by_scenario: Dict[str, List[TrialData]] = {}

    for scenario in scenarios:
        scenario_dir = results_dir / scenario
        if not scenario_dir.exists():
            print(f"Warning: {scenario_dir} does not exist")
            continue

        json_files = sorted(scenario_dir.glob('*_result.json'))
        trials = []

        for json_file in json_files:
            try:
                with open(json_file) as f:
                    data = json.load(f)

                trial = TrialData(
                    trial_id=data.get('trial_id', json_file.stem),
                    scenario=data.get('scenario', scenario),
                    status=data.get('status', 'unknown'),
                    cpu_percent_samples=data.get('cpu_percent_samples', []),
                    cpu_percent_mean=data.get('cpu_percent_mean', 0.0),
                    cpu_percent_max=data.get('cpu_percent_max', 0.0),
                    planning_latency_ms=data.get('planning_latency_ms', 0.0),
                    execution_latency_ms=data.get('execution_latency_ms', 0.0),
                    total_latency_ms=data.get('total_latency_ms', 0.0),
                    per_core_cpu_samples=data.get('per_core_cpu_samples', []),
                    per_core_cpu_mean=data.get('per_core_cpu_mean', []),
                    per_core_cpu_max=data.get('per_core_cpu_max', []),
                    active_cores=data.get('active_cores', []),
                )

                # Compute duration from sample count
                if trial.cpu_percent_samples:
                    trial.duration_s = len(trial.cpu_percent_samples) / trial.sample_rate_hz

                trials.append(trial)

            except Exception as e:
                print(f"Warning: Failed to load {json_file}: {e}")

        if trials:
            trials_by_scenario[scenario] = trials
            print(f"Loaded {len(trials)} trials for {scenario}")

    return trials_by_scenario


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


# =============================================================================
# CPU UTILIZATION PLOTS
# =============================================================================

def plot_cpu_timeseries(
    trial: TrialData,
    output_dir: Path,
    show_stats: bool = True
) -> None:
    """
    Plot CPU% over time for a single trial.

    Args:
        trial: TrialData object with cpu_percent_samples
        output_dir: Directory for output files
        show_stats: Annotate with mean/max/std statistics
    """
    if not trial.cpu_percent_samples:
        print(f"Warning: No CPU samples for {trial.trial_id}")
        return

    samples = np.array(trial.cpu_percent_samples)
    time_s = np.arange(len(samples)) / trial.sample_rate_hz

    fig, ax = plt.subplots(figsize=(12, 5))

    # Main line plot
    ax.plot(time_s, samples, color=COLORS.get(trial.scenario, '#3498db'),
            linewidth=1.5, alpha=0.8)

    # Fill area under curve
    ax.fill_between(time_s, 0, samples, alpha=0.2,
                    color=COLORS.get(trial.scenario, '#3498db'))

    # Statistics annotations
    if show_stats:
        mean_cpu = np.mean(samples)
        std_cpu = np.std(samples)
        max_cpu = np.max(samples)

        # Horizontal lines for mean and max
        ax.axhline(y=mean_cpu, color='green', linestyle='--', linewidth=1,
                   label=f'Mean: {mean_cpu:.1f}%')
        ax.axhline(y=max_cpu, color='red', linestyle=':', linewidth=1,
                   label=f'Max: {max_cpu:.1f}%')

        # Text annotation box
        stats_text = f'Mean: {mean_cpu:.1f} +/- {std_cpu:.1f}%\nMax: {max_cpu:.1f}%\nSamples: {len(samples)}'
        ax.text(0.98, 0.98, stats_text, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('CPU Utilization (%)', fontsize=11)
    ax.set_title(f'CPU Utilization: {trial.trial_id} ({trial.scenario})', fontsize=12)
    ax.set_ylim(0, 105)
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    output_base = output_dir / f'cpu_timeseries_{trial.trial_id}'
    plt.savefig(f'{output_base}.png', dpi=150, bbox_inches='tight')
    plt.savefig(f'{output_base}.pdf', bbox_inches='tight')
    plt.close()
    print(f"Saved: cpu_timeseries_{trial.trial_id}.png/pdf")


def plot_cpu_comparison(
    trials_by_scenario: Dict[str, List[TrialData]],
    output_dir: Path,
    scenarios_to_compare: List[str] = None,
    show_ci: bool = True
) -> None:
    """
    Overlay CPU time-series from multiple scenarios.

    Args:
        trials_by_scenario: Dict from load_json_trials()
        output_dir: Output directory
        scenarios_to_compare: Which scenarios to overlay
        show_ci: Show 95% confidence interval bands
    """
    if scenarios_to_compare is None:
        scenarios_to_compare = ['baseline', 'cpu_load']

    fig, ax = plt.subplots(figsize=(12, 6))

    for scenario in scenarios_to_compare:
        if scenario not in trials_by_scenario:
            continue

        trials = [t for t in trials_by_scenario[scenario]
                  if t.status == 'success' and t.cpu_percent_samples]

        if not trials:
            continue

        # Find max length and pad shorter arrays
        max_len = max(len(t.cpu_percent_samples) for t in trials)

        # Build matrix of samples (trials x time)
        sample_matrix = np.full((len(trials), max_len), np.nan)
        for i, trial in enumerate(trials):
            samples = trial.cpu_percent_samples
            sample_matrix[i, :len(samples)] = samples

        # Compute mean and CI at each time point
        time_s = np.arange(max_len) / trials[0].sample_rate_hz
        mean_cpu = np.nanmean(sample_matrix, axis=0)
        std_cpu = np.nanstd(sample_matrix, axis=0)
        n = np.sum(~np.isnan(sample_matrix), axis=0)

        # 95% CI: mean +/- 1.96 * std / sqrt(n)
        ci = 1.96 * std_cpu / np.sqrt(np.maximum(n, 1))

        color = COLORS.get(scenario, '#999999')

        # Plot mean line
        ax.plot(time_s, mean_cpu, color=color, linewidth=2,
                label=f'{scenario} (n={len(trials)})')

        # Plot CI band
        if show_ci:
            ax.fill_between(time_s, mean_cpu - ci, mean_cpu + ci,
                           color=color, alpha=0.2)

    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('CPU Utilization (%)', fontsize=11)
    ax.set_title('CPU Utilization Comparison: Baseline vs Load Scenarios', fontsize=12)
    ax.set_ylim(0, 105)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Annotate if stress-ng expected
    if 'cpu_load' in scenarios_to_compare and 'cpu_load' in trials_by_scenario:
        ax.annotate('stress-ng active', xy=(0.5, 85), fontsize=9,
                   style='italic', color='gray')

    plt.tight_layout()
    plt.savefig(output_dir / 'cpu_comparison.png', dpi=150, bbox_inches='tight')
    plt.savefig(output_dir / 'cpu_comparison.pdf', bbox_inches='tight')
    plt.close()
    print("Saved: cpu_comparison.png/pdf")


def plot_cpu_aggregate(
    trials_by_scenario: Dict[str, List[TrialData]],
    output_dir: Path
) -> None:
    """
    Generate aggregate CPU statistics plots (box plots).

    Args:
        trials_by_scenario: Dict from load_json_trials()
        output_dir: Output directory
    """
    # Subplot layout: mean, max, variance
    fig, axes = plt.subplots(1, 3, figsize=(14, 5))

    scenarios = ['baseline', 'cpu_load', 'msg_load']

    # Collect data for each metric
    mean_data = {s: [] for s in scenarios}
    max_data = {s: [] for s in scenarios}
    std_data = {s: [] for s in scenarios}

    for scenario in scenarios:
        if scenario not in trials_by_scenario:
            continue
        for trial in trials_by_scenario[scenario]:
            if trial.status == 'success' and trial.cpu_percent_samples:
                samples = np.array(trial.cpu_percent_samples)
                mean_data[scenario].append(np.mean(samples))
                max_data[scenario].append(np.max(samples))
                std_data[scenario].append(np.std(samples))

    # Plot 1: Mean CPU
    data_lists = [mean_data.get(s, []) for s in scenarios if mean_data.get(s)]
    labels = [s for s in scenarios if mean_data.get(s)]

    if data_lists:
        bp = axes[0].boxplot(data_lists, labels=labels, patch_artist=True)
        for patch, scenario in zip(bp['boxes'], labels):
            patch.set_facecolor(COLORS.get(scenario, '#999999'))
            patch.set_alpha(0.7)

    axes[0].set_title('Mean CPU Utilization', fontsize=11)
    axes[0].set_ylabel('CPU (%)', fontsize=10)
    axes[0].set_xlabel('Scenario', fontsize=10)

    # Plot 2: Max CPU
    max_lists = [max_data.get(s, []) for s in scenarios if max_data.get(s)]
    if max_lists:
        bp = axes[1].boxplot(max_lists, labels=labels, patch_artist=True)
        for patch, scenario in zip(bp['boxes'], labels):
            patch.set_facecolor(COLORS.get(scenario, '#999999'))
            patch.set_alpha(0.7)

    axes[1].set_title('Peak CPU Utilization', fontsize=11)
    axes[1].set_ylabel('CPU (%)', fontsize=10)
    axes[1].set_xlabel('Scenario', fontsize=10)

    # Plot 3: CPU Variance (std)
    std_lists = [std_data.get(s, []) for s in scenarios if std_data.get(s)]
    if std_lists:
        bp = axes[2].boxplot(std_lists, labels=labels, patch_artist=True)
        for patch, scenario in zip(bp['boxes'], labels):
            patch.set_facecolor(COLORS.get(scenario, '#999999'))
            patch.set_alpha(0.7)

    axes[2].set_title('CPU Utilization Variability', fontsize=11)
    axes[2].set_ylabel('Std Dev (%)', fontsize=10)
    axes[2].set_xlabel('Scenario', fontsize=10)

    plt.tight_layout()
    plt.savefig(output_dir / 'cpu_aggregate.png', dpi=150, bbox_inches='tight')
    plt.savefig(output_dir / 'cpu_aggregate.pdf', bbox_inches='tight')
    plt.close()
    print("Saved: cpu_aggregate.png/pdf")


def plot_cpu_vs_latency(
    trials_by_scenario: Dict[str, List[TrialData]],
    output_dir: Path,
    show_regression: bool = True,
    show_correlation: bool = True
) -> None:
    """
    Scatter plot of CPU utilization vs latency with correlation analysis.

    Args:
        trials_by_scenario: Dict from load_json_trials()
        output_dir: Output directory
        show_regression: Add linear regression line
        show_correlation: Show Pearson r and p-value
    """
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    latency_labels = {
        'planning_latency_ms': 'T1: Planning Latency (ms)',
        'execution_latency_ms': 'T2: Execution Latency (ms)',
        'total_latency_ms': 'T4: Total Latency (ms)'
    }

    metrics = ['planning_latency_ms', 'execution_latency_ms', 'total_latency_ms']

    for ax, metric in zip(axes, metrics):
        all_cpu = []
        all_latency = []

        for scenario, trials in trials_by_scenario.items():
            cpu_vals = []
            latency_vals = []

            for trial in trials:
                if trial.status == 'success' and trial.cpu_percent_samples:
                    cpu_vals.append(np.mean(trial.cpu_percent_samples))
                    latency_vals.append(getattr(trial, metric, 0))

            if cpu_vals:
                ax.scatter(cpu_vals, latency_vals,
                          color=COLORS.get(scenario, '#999999'),
                          label=scenario, alpha=0.7, s=50)
                all_cpu.extend(cpu_vals)
                all_latency.extend(latency_vals)

        # Regression and correlation
        if show_regression and len(all_cpu) > 2 and HAS_SCIPY:
            slope, intercept, r_value, p_value, std_err = scipy_stats.linregress(all_cpu, all_latency)
            x_line = np.array([min(all_cpu), max(all_cpu)])
            y_line = slope * x_line + intercept
            ax.plot(x_line, y_line, 'k--', linewidth=1, alpha=0.5)

            if show_correlation:
                corr_text = f'r = {r_value:.3f}\np = {p_value:.4f}'
                ax.text(0.05, 0.95, corr_text, transform=ax.transAxes,
                       fontsize=9, verticalalignment='top',
                       bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        ax.set_xlabel('Mean CPU Utilization (%)', fontsize=10)
        ax.set_ylabel(latency_labels.get(metric, metric), fontsize=10)
        ax.set_title(latency_labels.get(metric, metric).split(':')[0], fontsize=11)
        ax.legend(loc='lower right', fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / 'cpu_vs_latency.png', dpi=150, bbox_inches='tight')
    plt.savefig(output_dir / 'cpu_vs_latency.pdf', bbox_inches='tight')
    plt.close()
    print("Saved: cpu_vs_latency.png/pdf")


def compute_cpu_statistics(
    trials_by_scenario: Dict[str, List[TrialData]]
) -> Dict[str, Any]:
    """
    Compute comprehensive CPU statistics with hypothesis testing.

    Returns dict with:
    - Per-scenario descriptive stats
    - Baseline vs cpu_load t-test results
    - Effect sizes (Cohen's d)
    """
    stats = {}

    for scenario, trials in trials_by_scenario.items():
        successful = [t for t in trials if t.status == 'success' and t.cpu_percent_samples]
        if not successful:
            continue

        means = [np.mean(t.cpu_percent_samples) for t in successful]
        maxes = [np.max(t.cpu_percent_samples) for t in successful]
        stds = [np.std(t.cpu_percent_samples) for t in successful]

        stats[scenario] = {
            'n_trials': len(successful),
            'cpu_mean': {
                'mean': np.mean(means),
                'std': np.std(means),
                'min': np.min(means),
                'max': np.max(means)
            },
            'cpu_max': {
                'mean': np.mean(maxes),
                'std': np.std(maxes),
                'min': np.min(maxes),
                'max': np.max(maxes)
            },
            'cpu_variability': {
                'mean': np.mean(stds),
                'std': np.std(stds)
            },
        }

    # Statistical comparison: baseline vs cpu_load
    if 'baseline' in trials_by_scenario and 'cpu_load' in trials_by_scenario and HAS_SCIPY:
        baseline_means = [np.mean(t.cpu_percent_samples)
                         for t in trials_by_scenario['baseline']
                         if t.status == 'success' and t.cpu_percent_samples]
        load_means = [np.mean(t.cpu_percent_samples)
                     for t in trials_by_scenario['cpu_load']
                     if t.status == 'success' and t.cpu_percent_samples]

        if baseline_means and load_means:
            t_stat, p_value = scipy_stats.ttest_ind(baseline_means, load_means)

            # Cohen's d
            pooled_std = np.sqrt(
                ((len(baseline_means) - 1) * np.var(baseline_means, ddof=1) +
                 (len(load_means) - 1) * np.var(load_means, ddof=1)) /
                (len(baseline_means) + len(load_means) - 2)
            )
            cohens_d = (np.mean(load_means) - np.mean(baseline_means)) / pooled_std if pooled_std > 0 else 0

            stats['comparison_baseline_vs_cpu_load'] = {
                't_statistic': t_stat,
                'p_value': p_value,
                'cohens_d': cohens_d,
                'significant': p_value < 0.05,
                'mean_diff': np.mean(load_means) - np.mean(baseline_means),
                'pct_change': ((np.mean(load_means) - np.mean(baseline_means)) / np.mean(baseline_means) * 100)
                              if np.mean(baseline_means) > 0 else 0
            }

    return stats


def generate_cpu_latex_table(
    stats: Dict[str, Any],
    output_dir: Path
) -> None:
    """Generate LaTeX table for CPU statistics."""
    latex = r"""\begin{table}[htbp]
\centering
\caption{CPU Utilization Statistics by Scenario}
\label{tab:cpu_stats}
\begin{tabular}{lcccc}
\toprule
Scenario & Mean CPU (\%) & Max CPU (\%) & Std Dev (\%) & N \\
\midrule
"""

    for scenario in ['baseline', 'cpu_load', 'msg_load']:
        if scenario not in stats:
            continue
        s = stats[scenario]
        latex += f"{scenario.replace('_', ' ').title()} & "
        latex += f"{s['cpu_mean']['mean']:.1f} $\\pm$ {s['cpu_mean']['std']:.1f} & "
        latex += f"{s['cpu_max']['mean']:.1f} $\\pm$ {s['cpu_max']['std']:.1f} & "
        latex += f"{s['cpu_variability']['mean']:.1f} & "
        latex += f"{s['n_trials']} \\\\\n"

    latex += r"""\bottomrule
\end{tabular}
\end{table}
"""

    # Add comparison if available
    if 'comparison_baseline_vs_cpu_load' in stats:
        comp = stats['comparison_baseline_vs_cpu_load']
        latex += f"\n% Statistical comparison: t={comp['t_statistic']:.2f}, p={comp['p_value']:.4f}, d={comp['cohens_d']:.2f}\n"
        latex += f"% Mean diff: {comp['mean_diff']:.1f}% ({comp['pct_change']:+.1f}% change)\n"

    with open(output_dir / 'cpu_stats_table.tex', 'w') as f:
        f.write(latex)
    print("Saved: cpu_stats_table.tex")


def print_cpu_statistics(stats: Dict[str, Any]) -> None:
    """Print CPU statistics to console."""
    print("\n" + "=" * 60)
    print("CPU UTILIZATION STATISTICS")
    print("=" * 60)

    for scenario in ['baseline', 'cpu_load', 'msg_load']:
        if scenario not in stats:
            continue
        s = stats[scenario]
        print(f"\n{scenario.upper()} (n={s['n_trials']}):")
        print(f"  Mean CPU:  {s['cpu_mean']['mean']:.1f} +/- {s['cpu_mean']['std']:.1f}%")
        print(f"  Peak CPU:  {s['cpu_max']['mean']:.1f} +/- {s['cpu_max']['std']:.1f}%")
        print(f"  Variability: {s['cpu_variability']['mean']:.1f}%")

    if 'comparison_baseline_vs_cpu_load' in stats:
        comp = stats['comparison_baseline_vs_cpu_load']
        print(f"\nBASELINE vs CPU_LOAD COMPARISON:")
        print(f"  Mean difference: {comp['mean_diff']:.1f}% ({comp['pct_change']:+.1f}%)")
        print(f"  t-statistic: {comp['t_statistic']:.2f}")
        print(f"  p-value: {comp['p_value']:.4f}")
        print(f"  Cohen's d: {comp['cohens_d']:.2f}")
        sig_str = "SIGNIFICANT" if comp['significant'] else "not significant"
        print(f"  Result: {sig_str} at alpha=0.05")

    print("=" * 60)


# =============================================================================
# CPUSET / PER-CORE PLOTS
# =============================================================================

def plot_per_core_heatmap(
    trials_by_scenario: Dict[str, List[TrialData]],
    output_dir: Path,
    scenario: str = 'cpuset_limited',
    max_trials: int = 5
) -> None:
    """
    Heatmap of per-core CPU% over time for cpuset verification.

    Each row is a CPU core, each column is a time sample.
    Active (cpuset-allowed) cores should light up; others should stay dark.
    """
    if scenario not in trials_by_scenario:
        print(f"Warning: No {scenario} trials for per-core heatmap")
        return

    trials = [t for t in trials_by_scenario[scenario]
              if t.status == 'success' and t.per_core_cpu_samples][:max_trials]

    if not trials:
        print(f"Warning: No successful {scenario} trials with per-core data")
        return

    for trial in trials:
        samples = np.array(trial.per_core_cpu_samples)  # shape: (n_samples, n_cores)
        if samples.ndim != 2:
            continue

        n_samples, n_cores = samples.shape
        time_s = np.arange(n_samples) / trial.sample_rate_hz

        fig, ax = plt.subplots(figsize=(14, max(6, n_cores * 0.15)))

        im = ax.imshow(
            samples.T,  # transpose so rows=cores, cols=time
            aspect='auto',
            cmap='YlOrRd',
            vmin=0, vmax=100,
            origin='lower',
            extent=[0, time_s[-1] if len(time_s) > 0 else 1, -0.5, n_cores - 0.5]
        )

        # Mark active cores on y-axis
        if trial.active_cores:
            for core_id in trial.active_cores:
                ax.axhline(y=core_id, color='cyan', linewidth=0.5, alpha=0.6)

        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('CPU Core', fontsize=11)
        ax.set_title(
            f'Per-Core CPU Heatmap: {trial.trial_id}\n'
            f'Active cores: {trial.active_cores} ({len(trial.active_cores)}/{n_cores})',
            fontsize=12
        )

        cbar = plt.colorbar(im, ax=ax, label='CPU %')

        plt.tight_layout()
        output_base = output_dir / f'per_core_heatmap_{trial.trial_id}'
        plt.savefig(f'{output_base}.png', dpi=150, bbox_inches='tight')
        plt.savefig(f'{output_base}.pdf', bbox_inches='tight')
        plt.close()
        print(f"Saved: per_core_heatmap_{trial.trial_id}.png/pdf")


def plot_cpuset_verification(
    trials_by_scenario: Dict[str, List[TrialData]],
    output_dir: Path,
    scenario: str = 'cpuset_limited'
) -> None:
    """
    Bar chart of mean CPU% per core, averaged across all trials.

    Verifies cpuset isolation: bars should be high only for allowed cores.
    """
    if scenario not in trials_by_scenario:
        print(f"Warning: No {scenario} trials for cpuset verification")
        return

    trials = [t for t in trials_by_scenario[scenario]
              if t.status == 'success' and t.per_core_cpu_mean]

    if not trials:
        print(f"Warning: No successful {scenario} trials with per-core data")
        return

    # Average per-core means across all trials
    n_cores = len(trials[0].per_core_cpu_mean)
    core_means = np.zeros(n_cores)
    for trial in trials:
        if len(trial.per_core_cpu_mean) == n_cores:
            core_means += np.array(trial.per_core_cpu_mean)
    core_means /= len(trials)

    # Determine active cores (>5% across-trial mean)
    active_mask = core_means > 5.0

    fig, ax = plt.subplots(figsize=(max(10, n_cores * 0.3), 6))

    colors = ['#e74c3c' if active else '#bdc3c7' for active in active_mask]
    bars = ax.bar(range(n_cores), core_means, color=colors, edgecolor='gray', linewidth=0.5)

    # Threshold line
    ax.axhline(y=5.0, color='orange', linestyle='--', linewidth=1, label='5% threshold')

    ax.set_xlabel('CPU Core ID', fontsize=11)
    ax.set_ylabel('Mean CPU Utilization (%)', fontsize=11)
    active_list = [i for i, a in enumerate(active_mask) if a]
    ax.set_title(
        f'cpuset Verification: {scenario} (n={len(trials)} trials)\n'
        f'Active cores (>5%): {active_list}',
        fontsize=12
    )
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3, axis='y')

    # Only show every Nth tick label if many cores
    if n_cores > 20:
        tick_step = max(1, n_cores // 20)
        ax.set_xticks(range(0, n_cores, tick_step))

    plt.tight_layout()
    plt.savefig(output_dir / 'cpuset_verification.png', dpi=150, bbox_inches='tight')
    plt.savefig(output_dir / 'cpuset_verification.pdf', bbox_inches='tight')
    plt.close()
    print("Saved: cpuset_verification.png/pdf")


def main():
    parser = argparse.ArgumentParser(description='Plot LDOS experiment results')
    parser.add_argument('--input', help='Path to combined_summary.csv (for latency plots)')
    parser.add_argument('--results-dir', default='results',
                       help='Path to results/ directory with JSON files (for CPU plots)')
    parser.add_argument('--output-dir', required=True, help='Output directory for plots')

    # CPU plot flags
    parser.add_argument('--cpu-plots', action='store_true',
                       help='Generate CPU utilization visualizations')
    parser.add_argument('--cpu-timeseries', action='store_true',
                       help='Generate individual trial CPU time-series plots')
    parser.add_argument('--all-plots', action='store_true',
                       help='Generate all available plots (latency + CPU)')
    parser.add_argument('--cpuset-plots', action='store_true',
                       help='Generate cpuset per-core verification plots')

    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    generated_any = False

    # Load CSV for latency plots (existing behavior)
    if args.input or args.all_plots:
        csv_path = args.input
        if args.all_plots and not args.input:
            # Try default path
            default_csv = Path(args.results_dir).parent / 'analysis' / 'output' / 'combined_summary.csv'
            if default_csv.exists():
                csv_path = str(default_csv)

        if csv_path and Path(csv_path).exists():
            print(f"Loading data from: {csv_path}")
            df = load_data(csv_path)

            if len(df) > 0:
                print(f"\nGenerating latency plots...")
                plot_latency_boxplot(df, output_dir)
                plot_latency_histogram(df, output_dir)
                plot_scenario_comparison(df, output_dir)
                plot_degradation_analysis(df, output_dir)
                generate_latex_table(df, output_dir)
                generated_any = True
        elif args.input:
            print(f"ERROR: CSV file not found: {args.input}")
            sys.exit(1)

    # CPU plots require loading JSON files
    if args.cpu_plots or args.all_plots:
        print(f"\nLoading trial JSON files from: {args.results_dir}")
        trials_by_scenario = load_json_trials(Path(args.results_dir))

        if trials_by_scenario:
            print(f"\nGenerating CPU utilization plots...")

            # Aggregate plots
            plot_cpu_aggregate(trials_by_scenario, output_dir)
            plot_cpu_comparison(trials_by_scenario, output_dir)
            plot_cpu_vs_latency(trials_by_scenario, output_dir)

            # Statistics and tables
            cpu_stats = compute_cpu_statistics(trials_by_scenario)
            generate_cpu_latex_table(cpu_stats, output_dir)
            print_cpu_statistics(cpu_stats)

            # Individual time-series (optional, can generate many files)
            if args.cpu_timeseries:
                print("\nGenerating individual trial time-series plots...")
                for scenario, trials in trials_by_scenario.items():
                    for trial in trials[:3]:  # Limit to 3 per scenario
                        if trial.cpu_percent_samples:
                            plot_cpu_timeseries(trial, output_dir)

            # cpuset per-core verification plots
            if args.cpuset_plots or args.all_plots:
                print("\nGenerating cpuset per-core plots...")
                plot_per_core_heatmap(trials_by_scenario, output_dir)
                plot_cpuset_verification(trials_by_scenario, output_dir)

            generated_any = True
        else:
            print("Warning: No trial JSON files found")

    if not generated_any:
        print("ERROR: No data found to plot.")
        print("Use --input for latency CSV, or --cpu-plots/--all-plots for CPU analysis")
        sys.exit(1)

    print(f"\nAll plots saved to: {output_dir}")


if __name__ == '__main__':
    main()

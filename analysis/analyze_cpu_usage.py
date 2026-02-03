#!/usr/bin/env python3
"""
analyze_cpu_usage.py - Analyze CPU usage data from experiments

Usage:
    python analyze_cpu_usage.py <cpu_usage.csv> [output_dir]

Generates:
    - CPU utilization over time plot
    - Per-process CPU breakdown
    - CPU pinning verification (which CPUs each process ran on)
    - Summary statistics
"""

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def load_cpu_data(csv_path: str) -> pd.DataFrame:
    """Load and preprocess CPU usage CSV."""
    df = pd.read_csv(csv_path)

    # Convert timestamp to relative time (seconds from start)
    df['timestamp'] = pd.to_numeric(df['timestamp'])
    start_time = df['timestamp'].min()
    df['time_sec'] = df['timestamp'] - start_time

    return df

def plot_cpu_over_time(df: pd.DataFrame, output_path: str):
    """Plot CPU usage over time for key processes."""
    fig, ax = plt.subplots(figsize=(12, 6))

    # Group by process and time (1-second bins)
    df['time_bin'] = df['time_sec'].astype(int)

    # Key processes to track
    key_processes = ['move_group', 'controller_m', 'gz', 'robot_state', 'benchmark']

    for proc in key_processes:
        proc_data = df[df['process'].str.contains(proc, case=False, na=False)]
        if len(proc_data) > 0:
            # Sum CPU for processes with same prefix
            grouped = proc_data.groupby('time_bin')['cpu_percent'].sum()
            ax.plot(grouped.index, grouped.values, label=proc, linewidth=1.5)

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('CPU Usage (%)')
    ax.set_title('CPU Usage Over Time by Process')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  Saved: {output_path}")

def plot_cpu_pinning(df: pd.DataFrame, output_path: str):
    """Visualize which CPUs each process ran on."""
    fig, ax = plt.subplots(figsize=(10, 6))

    # Get unique processes and CPUs
    processes = df['process'].unique()

    # For each process, count samples per CPU
    process_cpu_counts = {}
    for proc in processes:
        proc_data = df[df['process'] == proc]
        cpu_counts = proc_data['cpu_id'].value_counts().sort_index()
        if len(cpu_counts) > 0:
            process_cpu_counts[proc] = cpu_counts

    # Plot as horizontal bar chart
    if process_cpu_counts:
        # Get all unique CPUs
        all_cpus = sorted(df['cpu_id'].dropna().unique())

        y_positions = range(len(process_cpu_counts))
        height = 0.8

        for i, (proc, cpu_counts) in enumerate(process_cpu_counts.items()):
            # Create stacked bars for each CPU
            left = 0
            for cpu in all_cpus:
                count = cpu_counts.get(cpu, 0)
                if count > 0:
                    ax.barh(i, count, left=left, height=height, label=f'CPU {int(cpu)}' if i == 0 else '')
                    left += count

        ax.set_yticks(y_positions)
        ax.set_yticklabels(list(process_cpu_counts.keys()))
        ax.set_xlabel('Sample Count')
        ax.set_title('CPU Affinity per Process (which CPUs each process ran on)')

        # Create legend for CPUs
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc='upper right')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  Saved: {output_path}")

def plot_process_breakdown(df: pd.DataFrame, output_path: str):
    """Pie chart of average CPU usage per process."""
    fig, ax = plt.subplots(figsize=(10, 8))

    # Calculate mean CPU per process
    mean_cpu = df.groupby('process')['cpu_percent'].mean().sort_values(ascending=False)

    # Only show top 10 processes
    top_procs = mean_cpu.head(10)

    if len(top_procs) > 0 and top_procs.sum() > 0:
        colors = plt.cm.tab10(np.linspace(0, 1, len(top_procs)))
        wedges, texts, autotexts = ax.pie(
            top_procs.values,
            labels=top_procs.index,
            autopct=lambda pct: f'{pct:.1f}%' if pct > 3 else '',
            colors=colors,
            startangle=90
        )
        ax.set_title('Average CPU Usage by Process')
    else:
        ax.text(0.5, 0.5, 'No CPU data available', ha='center', va='center')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"  Saved: {output_path}")

def generate_summary(df: pd.DataFrame, output_path: str):
    """Generate text summary of CPU usage."""
    lines = []
    lines.append("=" * 60)
    lines.append("CPU USAGE SUMMARY")
    lines.append("=" * 60)

    duration = df['time_sec'].max()
    lines.append(f"\nExperiment Duration: {duration:.1f} seconds")
    lines.append(f"Total Samples: {len(df)}")
    lines.append(f"Unique Processes: {df['process'].nunique()}")
    lines.append(f"CPUs Used: {sorted(df['cpu_id'].dropna().unique().astype(int).tolist())}")

    lines.append("\n" + "-" * 40)
    lines.append("Per-Process Statistics:")
    lines.append("-" * 40)

    for proc in df['process'].unique():
        proc_data = df[df['process'] == proc]
        cpu_mean = proc_data['cpu_percent'].mean()
        cpu_max = proc_data['cpu_percent'].max()
        cpus_used = sorted(proc_data['cpu_id'].dropna().unique().astype(int).tolist())

        lines.append(f"\n{proc}:")
        lines.append(f"  Avg CPU: {cpu_mean:.1f}%  Max: {cpu_max:.1f}%")
        lines.append(f"  CPUs: {cpus_used}")

    lines.append("\n" + "=" * 60)

    summary_text = "\n".join(lines)

    with open(output_path, 'w') as f:
        f.write(summary_text)

    print(f"  Saved: {output_path}")
    print(summary_text)

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_cpu_usage.py <cpu_usage.csv> [output_dir]")
        sys.exit(1)

    csv_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(csv_path)

    if not os.path.exists(csv_path):
        print(f"Error: File not found: {csv_path}")
        sys.exit(1)

    print(f"\n[ANALYZE] Loading: {csv_path}")
    df = load_cpu_data(csv_path)

    if len(df) == 0:
        print("Error: No data in CSV file")
        sys.exit(1)

    print(f"[ANALYZE] Loaded {len(df)} samples, {df['process'].nunique()} processes")

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # Generate plots and summary
    print("\n[ANALYZE] Generating outputs...")

    plot_cpu_over_time(df, os.path.join(output_dir, "cpu_over_time.png"))
    plot_cpu_pinning(df, os.path.join(output_dir, "cpu_pinning.png"))
    plot_process_breakdown(df, os.path.join(output_dir, "cpu_breakdown.png"))
    generate_summary(df, os.path.join(output_dir, "cpu_summary.txt"))

    print("\n[ANALYZE] Done!")

if __name__ == "__main__":
    main()

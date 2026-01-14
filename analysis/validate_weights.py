#!/usr/bin/env python3
"""
validate_weights.py - Validate objective weights against empirical data

This script validates the weights defined in objectives.yaml by:
1. Loading baseline and load scenario results
2. Computing sensitivity of each metric to load
3. Comparing defined weights to empirical behavior
4. Suggesting adjusted weights if needed

Usage:
    python3 validate_weights.py --objectives configs/objectives.yaml \
        --baseline results/baseline/summary.csv \
        --load results/cpu_load/summary.csv

Requirements:
    pip install pandas pyyaml scipy
"""

import argparse
import json
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import pandas as pd
import yaml

# Add the analysis directory to path for local imports
SCRIPT_DIR = Path(__file__).parent.resolve()
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

# Import local stats utilities
try:
    from stats_utils import compare_scenarios, ComparisonResult
    HAS_STATS = True
except ImportError:
    HAS_STATS = False

# Fallback scipy import
try:
    from scipy import stats
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class PathValidation:
    """Validation result for a single path."""
    path_id: str
    path_name: str
    defined_weight: float
    defined_criticality: str

    # Empirical data
    baseline_mean_ms: float
    load_mean_ms: float
    pct_change: float
    absolute_change: float

    # Statistical significance
    t_statistic: float
    p_value: float
    significant: bool
    cohens_d: float

    # Suggested adjustment
    suggested_weight: float
    weight_aligned: bool
    recommendation: str


@dataclass
class ValidationReport:
    """Complete validation report."""
    objectives_file: str
    baseline_file: str
    load_file: str
    n_baseline: int
    n_load: int

    path_validations: List[PathValidation]
    overall_aligned: bool
    summary: str


# =============================================================================
# VALIDATION FUNCTIONS
# =============================================================================

def load_objectives(objectives_path: str) -> Dict:
    """Load objectives from YAML file."""
    with open(objectives_path) as f:
        return yaml.safe_load(f)


def load_results(csv_path: str) -> pd.DataFrame:
    """Load results from CSV file."""
    df = pd.read_csv(csv_path)
    # Filter to successful trials if status column exists
    if 'status' in df.columns:
        df = df[df['status'] == 'success']
    return df


def get_metric_for_path(path_id: str) -> str:
    """Map path ID to primary metric column."""
    mapping = {
        'P1_planning': 'planning_latency_ms',
        'P2_control': 'execution_latency_ms',  # Control affects execution
        'P3_feedback': 'planning_latency_ms',  # Feedback affects planning
        'P4_collision': 'planning_latency_ms',
        'P5_perception': 'total_latency_ms',
    }
    return mapping.get(path_id, 'total_latency_ms')


def compute_sensitivity(
    baseline: pd.Series,
    load: pd.Series
) -> Tuple[float, float, float, float, bool, float]:
    """
    Compute sensitivity metrics between baseline and load conditions.

    Returns:
        Tuple of (pct_change, absolute_change, t_stat, p_value, significant, cohens_d)
    """
    baseline_mean = baseline.mean()
    load_mean = load.mean()

    pct_change = ((load_mean - baseline_mean) / baseline_mean * 100) if baseline_mean > 0 else 0
    absolute_change = load_mean - baseline_mean

    # T-test
    if HAS_SCIPY:
        t_stat, p_value = stats.ttest_ind(baseline, load)
    elif HAS_STATS:
        result = compare_scenarios(baseline.values, load.values)
        t_stat, p_value = result.t_statistic, result.p_value
    else:
        # Fallback
        t_stat, p_value = 0, 1.0

    significant = p_value < 0.05

    # Cohen's d
    if HAS_STATS:
        from stats_utils import cohens_d
        d = cohens_d(baseline.values, load.values)
    else:
        pooled_std = ((baseline.std()**2 + load.std()**2) / 2) ** 0.5
        d = (load_mean - baseline_mean) / pooled_std if pooled_std > 0 else 0

    return pct_change, absolute_change, t_stat, p_value, significant, d


def suggest_weight(
    pct_change: float,
    cohens_d: float,
    defined_criticality: str,
    significant: bool
) -> float:
    """
    Suggest weight based on empirical sensitivity.

    Higher sensitivity (larger degradation under load) suggests higher weight.
    """
    # Base weight from criticality
    criticality_weights = {
        'CRITICAL': 1.0,
        'HIGH': 0.7,
        'MEDIUM': 0.4,
        'LOW': 0.1,
    }
    base_weight = criticality_weights.get(defined_criticality.upper(), 0.5)

    if not significant:
        # Not significantly affected by load - maintain or reduce weight
        return base_weight * 0.9

    # Adjust based on effect size
    abs_d = abs(cohens_d)
    if abs_d >= 0.8:
        # Large effect - increase weight
        adjustment = 1.2
    elif abs_d >= 0.5:
        # Medium effect - slight increase
        adjustment = 1.1
    else:
        # Small effect - maintain
        adjustment = 1.0

    # Cap at 1.0
    return min(1.0, base_weight * adjustment)


def check_weight_alignment(
    defined_weight: float,
    suggested_weight: float,
    tolerance: float = 0.15
) -> Tuple[bool, str]:
    """
    Check if defined weight aligns with empirical suggestion.

    Returns:
        Tuple of (is_aligned, recommendation)
    """
    diff = abs(defined_weight - suggested_weight)

    if diff <= tolerance:
        return True, "Weight is appropriate for observed sensitivity"
    elif defined_weight < suggested_weight:
        return False, f"Consider increasing weight (suggest {suggested_weight:.2f})"
    else:
        return False, f"Consider decreasing weight (suggest {suggested_weight:.2f})"


def validate_path(
    path_id: str,
    path_def: Dict,
    baseline_df: pd.DataFrame,
    load_df: pd.DataFrame
) -> PathValidation:
    """Validate a single path."""
    path_name = path_def.get('name', path_id)
    defined_weight = path_def.get('weight', 0.5)
    defined_criticality = path_def.get('criticality', 'MEDIUM')

    # Get the metric for this path
    metric = get_metric_for_path(path_id)

    if metric not in baseline_df.columns or metric not in load_df.columns:
        # Fallback to total latency
        metric = 'total_latency_ms'

    if metric not in baseline_df.columns:
        return PathValidation(
            path_id=path_id,
            path_name=path_name,
            defined_weight=defined_weight,
            defined_criticality=defined_criticality,
            baseline_mean_ms=0,
            load_mean_ms=0,
            pct_change=0,
            absolute_change=0,
            t_statistic=0,
            p_value=1.0,
            significant=False,
            cohens_d=0,
            suggested_weight=defined_weight,
            weight_aligned=True,
            recommendation="No data available for validation"
        )

    baseline_data = baseline_df[metric].dropna()
    load_data = load_df[metric].dropna()

    if len(baseline_data) < 2 or len(load_data) < 2:
        return PathValidation(
            path_id=path_id,
            path_name=path_name,
            defined_weight=defined_weight,
            defined_criticality=defined_criticality,
            baseline_mean_ms=baseline_data.mean() if len(baseline_data) > 0 else 0,
            load_mean_ms=load_data.mean() if len(load_data) > 0 else 0,
            pct_change=0,
            absolute_change=0,
            t_statistic=0,
            p_value=1.0,
            significant=False,
            cohens_d=0,
            suggested_weight=defined_weight,
            weight_aligned=True,
            recommendation="Insufficient data for validation"
        )

    # Compute sensitivity
    pct_change, abs_change, t_stat, p_value, significant, d = compute_sensitivity(
        baseline_data, load_data
    )

    # Suggest weight
    suggested = suggest_weight(pct_change, d, defined_criticality, significant)

    # Check alignment
    aligned, recommendation = check_weight_alignment(defined_weight, suggested)

    return PathValidation(
        path_id=path_id,
        path_name=path_name,
        defined_weight=defined_weight,
        defined_criticality=defined_criticality,
        baseline_mean_ms=baseline_data.mean(),
        load_mean_ms=load_data.mean(),
        pct_change=pct_change,
        absolute_change=abs_change,
        t_statistic=t_stat,
        p_value=p_value,
        significant=significant,
        cohens_d=d,
        suggested_weight=suggested,
        weight_aligned=aligned,
        recommendation=recommendation
    )


def validate_all(
    objectives: Dict,
    baseline_df: pd.DataFrame,
    load_df: pd.DataFrame,
    objectives_path: str,
    baseline_path: str,
    load_path: str
) -> ValidationReport:
    """Run validation for all paths."""
    validations = []

    paths = objectives.get('paths', {})
    for path_id, path_def in paths.items():
        if path_def.get('enabled', True):
            validation = validate_path(path_id, path_def, baseline_df, load_df)
            validations.append(validation)

    # Determine overall alignment
    aligned_count = sum(1 for v in validations if v.weight_aligned)
    total_count = len(validations)
    overall_aligned = aligned_count == total_count

    # Generate summary
    if overall_aligned:
        summary = "All weights are well-aligned with empirical observations."
    else:
        misaligned = [v.path_id for v in validations if not v.weight_aligned]
        summary = f"Weight adjustments suggested for: {', '.join(misaligned)}"

    return ValidationReport(
        objectives_file=objectives_path,
        baseline_file=baseline_path,
        load_file=load_path,
        n_baseline=len(baseline_df),
        n_load=len(load_df),
        path_validations=validations,
        overall_aligned=overall_aligned,
        summary=summary
    )


# =============================================================================
# REPORTING
# =============================================================================

def generate_report(report: ValidationReport, output_dir: Path):
    """Generate validation report."""
    output_dir.mkdir(parents=True, exist_ok=True)

    # Markdown report
    md_path = output_dir / "weight_validation_report.md"
    lines = [
        "# Objective Weight Validation Report\n",
        f"\n## Data Sources\n",
        f"\n- **Objectives:** `{report.objectives_file}`\n",
        f"- **Baseline data:** `{report.baseline_file}` (n={report.n_baseline})\n",
        f"- **Load data:** `{report.load_file}` (n={report.n_load})\n",
        f"\n## Summary\n",
        f"\n**{report.summary}**\n",
        f"\n## Path Validation Details\n",
        f"\n| Path | Weight | Criticality | Change (%) | d | Sig | Suggested | Aligned |\n",
        f"|------|--------|-------------|------------|---|-----|-----------|--------|\n",
    ]

    for v in report.path_validations:
        sig = "Yes" if v.significant else "No"
        aligned = "Yes" if v.weight_aligned else "No"
        lines.append(
            f"| {v.path_id} | {v.defined_weight:.2f} | {v.defined_criticality} | "
            f"{v.pct_change:+.1f}% | {v.cohens_d:.2f} | {sig} | "
            f"{v.suggested_weight:.2f} | {aligned} |\n"
        )

    lines.append(f"\n## Detailed Recommendations\n")

    for v in report.path_validations:
        if not v.weight_aligned:
            lines.append(f"\n### {v.path_id}: {v.path_name}\n")
            lines.append(f"\n- **Current weight:** {v.defined_weight:.2f}\n")
            lines.append(f"- **Suggested weight:** {v.suggested_weight:.2f}\n")
            lines.append(f"- **Observation:** {v.pct_change:+.1f}% change under load (d={v.cohens_d:.2f})\n")
            lines.append(f"- **Recommendation:** {v.recommendation}\n")

    lines.append(f"\n## Statistical Details\n")

    for v in report.path_validations:
        lines.append(f"\n### {v.path_id}\n")
        lines.append(f"\n- Baseline: {v.baseline_mean_ms:.2f} ms\n")
        lines.append(f"- Under load: {v.load_mean_ms:.2f} ms\n")
        lines.append(f"- Change: {v.pct_change:+.1f}% ({v.absolute_change:+.1f} ms)\n")
        lines.append(f"- t-statistic: {v.t_statistic:.2f}\n")
        lines.append(f"- p-value: {v.p_value:.4f}\n")
        lines.append(f"- Cohen's d: {v.cohens_d:.2f}\n")

    with open(md_path, 'w') as f:
        f.writelines(lines)

    print(f"Saved: {md_path}")

    # JSON data
    json_path = output_dir / "weight_validation_data.json"
    data = {
        'objectives_file': report.objectives_file,
        'baseline_file': report.baseline_file,
        'load_file': report.load_file,
        'n_baseline': report.n_baseline,
        'n_load': report.n_load,
        'overall_aligned': report.overall_aligned,
        'summary': report.summary,
        'validations': [asdict(v) for v in report.path_validations]
    }
    with open(json_path, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"Saved: {json_path}")


def print_summary(report: ValidationReport):
    """Print validation summary to console."""
    print("\n" + "="*60)
    print("WEIGHT VALIDATION SUMMARY")
    print("="*60)

    print(f"\nBaseline trials: {report.n_baseline}")
    print(f"Load trials: {report.n_load}")
    print(f"\n{report.summary}\n")

    print(f"{'Path':<15} {'Weight':>8} {'Suggested':>10} {'Change':>10} {'Aligned':>8}")
    print("-"*55)

    for v in report.path_validations:
        aligned = "Yes" if v.weight_aligned else "NO"
        print(f"{v.path_id:<15} {v.defined_weight:>8.2f} {v.suggested_weight:>10.2f} "
              f"{v.pct_change:>+9.1f}% {aligned:>8}")

    print()


# =============================================================================
# CLI
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Validate objective weights against empirical data',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 validate_weights.py --objectives configs/objectives.yaml \\
      --baseline results/baseline/summary.csv \\
      --load results/cpu_load/summary.csv

  python3 validate_weights.py -o configs/objectives.yaml -b results/baseline/ -l results/cpu_load/
        """
    )

    parser.add_argument('--objectives', '-o', required=True,
                        help='Path to objectives.yaml')
    parser.add_argument('--baseline', '-b', required=True,
                        help='Path to baseline results (CSV or directory)')
    parser.add_argument('--load', '-l', required=True,
                        help='Path to load scenario results (CSV or directory)')
    parser.add_argument('--output', default='analysis/output',
                        help='Output directory for reports')

    args = parser.parse_args()

    # Resolve paths
    baseline_path = Path(args.baseline)
    load_path = Path(args.load)

    # If directories given, look for summary CSV
    if baseline_path.is_dir():
        csv_files = list(baseline_path.glob("*summary*.csv")) + list(baseline_path.glob("*.csv"))
        baseline_path = csv_files[0] if csv_files else baseline_path / "summary.csv"

    if load_path.is_dir():
        csv_files = list(load_path.glob("*summary*.csv")) + list(load_path.glob("*.csv"))
        load_path = csv_files[0] if csv_files else load_path / "summary.csv"

    print(f"Loading objectives from: {args.objectives}")
    print(f"Loading baseline from: {baseline_path}")
    print(f"Loading load data from: {load_path}")

    # Load data
    objectives = load_objectives(args.objectives)
    baseline_df = load_results(str(baseline_path))
    load_df = load_results(str(load_path))

    print(f"\nLoaded {len(baseline_df)} baseline trials, {len(load_df)} load trials")

    # Run validation
    report = validate_all(
        objectives, baseline_df, load_df,
        args.objectives, str(baseline_path), str(load_path)
    )

    # Generate outputs
    generate_report(report, Path(args.output))
    print_summary(report)

    return 0 if report.overall_aligned else 1


if __name__ == '__main__':
    sys.exit(main())

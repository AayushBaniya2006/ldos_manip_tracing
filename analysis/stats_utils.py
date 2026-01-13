#!/usr/bin/env python3
"""
stats_utils.py - Statistical utilities for LDOS analysis

This module provides reusable statistical functions for:
1. Confidence intervals
2. Effect size calculations (Cohen's d)
3. Hypothesis testing (t-tests, ANOVA)
4. Power analysis

Usage:
    from analysis.stats_utils import compare_scenarios, confidence_interval, cohens_d

Requirements:
    pip install scipy numpy statsmodels
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Union
from dataclasses import dataclass


# =============================================================================
# TRY TO IMPORT OPTIONAL DEPENDENCIES
# =============================================================================

try:
    from scipy import stats
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("Warning: scipy not installed. Some statistical functions unavailable.")

try:
    from statsmodels.stats.multicomp import pairwise_tukeyhsd
    from statsmodels.stats.power import TTestIndPower
    HAS_STATSMODELS = True
except ImportError:
    HAS_STATSMODELS = False
    print("Warning: statsmodels not installed. Post-hoc tests unavailable.")


# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class ComparisonResult:
    """Result of comparing two groups."""
    group1_name: str
    group2_name: str
    group1_n: int
    group2_n: int
    group1_mean: float
    group1_std: float
    group2_mean: float
    group2_std: float
    mean_diff: float
    pct_change: float
    t_statistic: float
    p_value: float
    cohens_d: float
    significant: bool  # at alpha=0.05
    ci_lower: float    # 95% CI for mean difference
    ci_upper: float

    def __str__(self) -> str:
        sig_str = "SIGNIFICANT" if self.significant else "not significant"
        return (
            f"{self.group1_name} vs {self.group2_name}: "
            f"diff={self.mean_diff:.2f} ({self.pct_change:+.1f}%), "
            f"p={self.p_value:.4f} ({sig_str}), "
            f"d={self.cohens_d:.2f}"
        )


@dataclass
class ANOVAResult:
    """Result of one-way ANOVA."""
    group_names: List[str]
    f_statistic: float
    p_value: float
    significant: bool
    eta_squared: float  # Effect size
    posthoc_results: Optional[str] = None


# =============================================================================
# BASIC STATISTICS
# =============================================================================

def confidence_interval(
    data: Union[List[float], np.ndarray],
    confidence: float = 0.95
) -> Tuple[float, float]:
    """
    Compute confidence interval for the mean.

    Args:
        data: Sample data
        confidence: Confidence level (default 0.95 for 95% CI)

    Returns:
        Tuple of (lower_bound, upper_bound)
    """
    data = np.array(data)
    n = len(data)

    if n < 2:
        return (np.nan, np.nan)

    mean = np.mean(data)

    if HAS_SCIPY:
        se = stats.sem(data)
        h = se * stats.t.ppf((1 + confidence) / 2, n - 1)
    else:
        # Fallback: use normal approximation
        se = np.std(data, ddof=1) / np.sqrt(n)
        z = {0.90: 1.645, 0.95: 1.96, 0.99: 2.576}.get(confidence, 1.96)
        h = se * z

    return (mean - h, mean + h)


def cohens_d(
    group1: Union[List[float], np.ndarray],
    group2: Union[List[float], np.ndarray]
) -> float:
    """
    Compute Cohen's d effect size.

    Cohen's d measures the standardized difference between two means:
    - d < 0.2: negligible
    - 0.2 <= d < 0.5: small
    - 0.5 <= d < 0.8: medium
    - d >= 0.8: large

    Args:
        group1: First sample
        group2: Second sample

    Returns:
        Cohen's d value
    """
    g1 = np.array(group1)
    g2 = np.array(group2)

    n1, n2 = len(g1), len(g2)

    if n1 < 2 or n2 < 2:
        return np.nan

    var1 = np.var(g1, ddof=1)
    var2 = np.var(g2, ddof=1)

    # Pooled standard deviation
    pooled_std = np.sqrt(((n1 - 1) * var1 + (n2 - 1) * var2) / (n1 + n2 - 2))

    if pooled_std == 0:
        return 0.0

    return (np.mean(g1) - np.mean(g2)) / pooled_std


def interpret_cohens_d(d: float) -> str:
    """Interpret Cohen's d effect size."""
    d = abs(d)
    if d < 0.2:
        return "negligible"
    elif d < 0.5:
        return "small"
    elif d < 0.8:
        return "medium"
    else:
        return "large"


# =============================================================================
# HYPOTHESIS TESTING
# =============================================================================

def compare_scenarios(
    baseline: Union[List[float], np.ndarray],
    treatment: Union[List[float], np.ndarray],
    baseline_name: str = "baseline",
    treatment_name: str = "treatment",
    alpha: float = 0.05
) -> ComparisonResult:
    """
    Comprehensive statistical comparison between two scenarios.

    Performs:
    - Two-sample t-test
    - Cohen's d effect size
    - 95% confidence interval for mean difference

    Args:
        baseline: Baseline measurements
        treatment: Treatment measurements
        baseline_name: Name for baseline group
        treatment_name: Name for treatment group
        alpha: Significance level (default 0.05)

    Returns:
        ComparisonResult with all statistics
    """
    g1 = np.array(baseline)
    g2 = np.array(treatment)

    n1, n2 = len(g1), len(g2)
    mean1, mean2 = np.mean(g1), np.mean(g2)
    std1, std2 = np.std(g1, ddof=1), np.std(g2, ddof=1)

    mean_diff = mean2 - mean1
    pct_change = ((mean2 - mean1) / mean1 * 100) if mean1 != 0 else np.nan

    # T-test
    if HAS_SCIPY:
        t_stat, p_value = stats.ttest_ind(g1, g2)
    else:
        # Simple t-test approximation
        se_diff = np.sqrt(std1**2/n1 + std2**2/n2)
        t_stat = mean_diff / se_diff if se_diff > 0 else 0
        # Use approximation for p-value
        p_value = 0.05 if abs(t_stat) > 2 else 0.5

    # Effect size
    d = cohens_d(g1, g2)

    # CI for mean difference
    se_diff = np.sqrt(std1**2/n1 + std2**2/n2)
    if HAS_SCIPY:
        df = n1 + n2 - 2
        t_crit = stats.t.ppf(0.975, df)
    else:
        t_crit = 1.96  # Approximation

    ci_lower = mean_diff - t_crit * se_diff
    ci_upper = mean_diff + t_crit * se_diff

    return ComparisonResult(
        group1_name=baseline_name,
        group2_name=treatment_name,
        group1_n=n1,
        group2_n=n2,
        group1_mean=mean1,
        group1_std=std1,
        group2_mean=mean2,
        group2_std=std2,
        mean_diff=mean_diff,
        pct_change=pct_change,
        t_statistic=t_stat,
        p_value=p_value,
        cohens_d=d,
        significant=p_value < alpha,
        ci_lower=ci_lower,
        ci_upper=ci_upper
    )


def anova_with_posthoc(
    groups: Dict[str, Union[List[float], np.ndarray]],
    alpha: float = 0.05
) -> ANOVAResult:
    """
    One-way ANOVA with Tukey HSD post-hoc test.

    Args:
        groups: Dict mapping group names to data arrays
        alpha: Significance level

    Returns:
        ANOVAResult with F-statistic, p-value, and post-hoc results
    """
    group_names = list(groups.keys())
    group_data = [np.array(groups[name]) for name in group_names]

    # One-way ANOVA
    if HAS_SCIPY:
        f_stat, p_value = stats.f_oneway(*group_data)
    else:
        # Simplified ANOVA calculation
        all_data = np.concatenate(group_data)
        grand_mean = np.mean(all_data)
        ss_between = sum(len(g) * (np.mean(g) - grand_mean)**2 for g in group_data)
        ss_within = sum(np.sum((g - np.mean(g))**2) for g in group_data)
        df_between = len(groups) - 1
        df_within = len(all_data) - len(groups)
        f_stat = (ss_between / df_between) / (ss_within / df_within) if df_within > 0 else 0
        p_value = 0.05 if f_stat > 4 else 0.5  # Rough approximation

    # Effect size (eta squared)
    all_data = np.concatenate(group_data)
    grand_mean = np.mean(all_data)
    ss_total = np.sum((all_data - grand_mean)**2)
    ss_between = sum(len(g) * (np.mean(g) - grand_mean)**2 for g in group_data)
    eta_squared = ss_between / ss_total if ss_total > 0 else 0

    # Post-hoc test
    posthoc_str = None
    if HAS_STATSMODELS and p_value < alpha:
        # Prepare data for Tukey HSD
        all_data = []
        all_labels = []
        for name, data in groups.items():
            all_data.extend(data)
            all_labels.extend([name] * len(data))

        tukey = pairwise_tukeyhsd(all_data, all_labels, alpha=alpha)
        posthoc_str = str(tukey.summary())

    return ANOVAResult(
        group_names=group_names,
        f_statistic=f_stat,
        p_value=p_value,
        significant=p_value < alpha,
        eta_squared=eta_squared,
        posthoc_results=posthoc_str
    )


# =============================================================================
# NORMALITY AND ASSUMPTIONS
# =============================================================================

def test_normality(
    data: Union[List[float], np.ndarray],
    alpha: float = 0.05
) -> Tuple[float, float, bool]:
    """
    Test for normality using Shapiro-Wilk test.

    Args:
        data: Sample data
        alpha: Significance level

    Returns:
        Tuple of (W statistic, p-value, is_normal)
    """
    data = np.array(data)

    if len(data) < 3:
        return (np.nan, np.nan, True)

    if HAS_SCIPY:
        if len(data) > 5000:
            # Use Anderson-Darling for large samples
            result = stats.anderson(data)
            # Use 5% significance level index
            is_normal = result.statistic < result.critical_values[2]
            return (result.statistic, np.nan, is_normal)
        else:
            w_stat, p_value = stats.shapiro(data)
            return (w_stat, p_value, p_value > alpha)
    else:
        # Simple kurtosis/skewness check
        skew = stats.skew(data) if HAS_SCIPY else 0
        return (np.nan, np.nan, abs(skew) < 2)


def test_homogeneity(
    *groups: Union[List[float], np.ndarray],
    alpha: float = 0.05
) -> Tuple[float, float, bool]:
    """
    Test for homogeneity of variances using Levene's test.

    Args:
        *groups: Variable number of data arrays
        alpha: Significance level

    Returns:
        Tuple of (W statistic, p-value, variances_equal)
    """
    if not HAS_SCIPY:
        return (np.nan, np.nan, True)

    if len(groups) < 2:
        return (np.nan, np.nan, True)

    groups = [np.array(g) for g in groups]
    w_stat, p_value = stats.levene(*groups)

    return (w_stat, p_value, p_value > alpha)


# =============================================================================
# POWER ANALYSIS
# =============================================================================

def power_analysis(
    effect_size: float,
    n1: int,
    n2: Optional[int] = None,
    alpha: float = 0.05
) -> float:
    """
    Compute statistical power for two-sample t-test.

    Args:
        effect_size: Expected Cohen's d
        n1: Sample size of group 1
        n2: Sample size of group 2 (defaults to n1)
        alpha: Significance level

    Returns:
        Statistical power (probability of detecting effect)
    """
    if n2 is None:
        n2 = n1

    if HAS_STATSMODELS:
        analysis = TTestIndPower()
        ratio = n2 / n1
        return analysis.power(effect_size=effect_size, nobs1=n1, ratio=ratio, alpha=alpha)
    else:
        # Simple approximation
        se = np.sqrt(2.0 / ((n1 + n2) / 2))
        ncp = effect_size / se  # Non-centrality parameter
        # Rough power estimate
        return min(0.99, 0.5 + 0.4 * ncp)


def required_sample_size(
    effect_size: float,
    power: float = 0.8,
    alpha: float = 0.05
) -> int:
    """
    Compute required sample size per group for two-sample t-test.

    Args:
        effect_size: Expected Cohen's d
        power: Desired power (default 0.8)
        alpha: Significance level

    Returns:
        Required sample size per group
    """
    if HAS_STATSMODELS:
        analysis = TTestIndPower()
        n = analysis.solve_power(effect_size=effect_size, power=power, alpha=alpha, ratio=1.0)
        return int(np.ceil(n))
    else:
        # Cohen's approximation
        if effect_size < 0.2:
            return 500
        elif effect_size < 0.5:
            return 100
        elif effect_size < 0.8:
            return 50
        else:
            return 25


# =============================================================================
# REPORTING UTILITIES
# =============================================================================

def format_comparison_table(
    comparisons: List[ComparisonResult],
    latex: bool = False
) -> str:
    """
    Format multiple comparisons as a table.

    Args:
        comparisons: List of ComparisonResult objects
        latex: If True, output LaTeX table

    Returns:
        Formatted table string
    """
    if latex:
        lines = [
            "\\begin{table}[h]",
            "\\centering",
            "\\caption{Statistical Comparisons}",
            "\\begin{tabular}{lrrrrrc}",
            "\\toprule",
            "Comparison & $\\Delta$ Mean & \\% Change & $t$ & $p$ & $d$ & Sig. \\\\",
            "\\midrule",
        ]

        for c in comparisons:
            sig = "$\\checkmark$" if c.significant else ""
            lines.append(
                f"{c.group1_name} vs {c.group2_name} & "
                f"{c.mean_diff:.2f} & {c.pct_change:+.1f}\\% & "
                f"{c.t_statistic:.2f} & {c.p_value:.4f} & {c.cohens_d:.2f} & {sig} \\\\"
            )

        lines.extend([
            "\\bottomrule",
            "\\end{tabular}",
            "\\end{table}",
        ])

        return '\n'.join(lines)
    else:
        lines = [
            "| Comparison | Mean Diff | % Change | t | p | d | Sig |",
            "|------------|-----------|----------|---|---|---|-----|",
        ]

        for c in comparisons:
            sig = "Yes" if c.significant else "No"
            lines.append(
                f"| {c.group1_name} vs {c.group2_name} | "
                f"{c.mean_diff:.2f} | {c.pct_change:+.1f}% | "
                f"{c.t_statistic:.2f} | {c.p_value:.4f} | {c.cohens_d:.2f} | {sig} |"
            )

        return '\n'.join(lines)


# =============================================================================
# MAIN (for testing)
# =============================================================================

if __name__ == '__main__':
    # Example usage
    np.random.seed(42)

    baseline = np.random.normal(100, 15, 50)
    treatment = np.random.normal(120, 20, 50)

    print("=== Statistical Utilities Demo ===\n")

    # Confidence intervals
    ci = confidence_interval(baseline)
    print(f"Baseline 95% CI: ({ci[0]:.2f}, {ci[1]:.2f})")

    # Effect size
    d = cohens_d(baseline, treatment)
    print(f"Cohen's d: {d:.2f} ({interpret_cohens_d(d)} effect)")

    # Full comparison
    result = compare_scenarios(baseline, treatment, "baseline", "treatment")
    print(f"\n{result}")

    # Normality test
    w, p, is_normal = test_normality(baseline)
    print(f"\nNormality test: W={w:.4f}, p={p:.4f}, normal={is_normal}")

    # Power analysis
    power = power_analysis(d, len(baseline), len(treatment))
    print(f"Statistical power: {power:.2%}")

    required_n = required_sample_size(0.5, power=0.8)
    print(f"Required N for medium effect (d=0.5): {required_n} per group")

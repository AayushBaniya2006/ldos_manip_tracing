#!/usr/bin/env python3
"""
metrics_schema.py - Metrics definitions for LDOS tracing analysis

This module defines:
1. Objective metrics (T1-T5)
2. CSV schema for results
3. Weighting methods for multi-objective analysis
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Callable
import json


class MetricUnit(Enum):
    """Units for metrics."""
    MILLISECONDS = "ms"
    MICROSECONDS = "us"
    NANOSECONDS = "ns"
    SECONDS = "s"
    HERTZ = "Hz"
    COUNT = "count"
    PERCENT = "%"


@dataclass
class MetricDefinition:
    """Definition of a single metric."""
    id: str
    name: str
    description: str
    unit: MetricUnit
    start_marker: Optional[str] = None
    end_marker: Optional[str] = None
    source: Optional[str] = None  # Alternative to markers
    higher_is_better: bool = False
    weight: float = 1.0  # Default weight for multi-objective


# =============================================================================
# OBJECTIVE METRICS (T1-T5)
# =============================================================================

OBJECTIVE_METRICS: Dict[str, MetricDefinition] = {
    "T1": MetricDefinition(
        id="T1",
        name="Planning Latency",
        description="Time from planning request to plan received",
        unit=MetricUnit.MILLISECONDS,
        start_marker="planning_request_sent",
        end_marker="planning_response_received",
        higher_is_better=False,
        weight=1.0,
    ),
    "T2": MetricDefinition(
        id="T2",
        name="Execution Latency",
        description="Time from execute request to motion complete",
        unit=MetricUnit.MILLISECONDS,
        start_marker="execute_request_sent",
        end_marker="execute_complete",
        higher_is_better=False,
        weight=1.0,
    ),
    "T3": MetricDefinition(
        id="T3",
        name="Control Loop Jitter",
        description="Standard deviation of control loop period",
        unit=MetricUnit.MILLISECONDS,
        source="controller_manager callback intervals",
        higher_is_better=False,
        weight=0.5,  # Lower weight - secondary metric
    ),
    "T4": MetricDefinition(
        id="T4",
        name="Total E2E Latency",
        description="Total time from goal sent to goal achieved",
        unit=MetricUnit.MILLISECONDS,
        start_marker="benchmark_goal_sent",
        end_marker="benchmark_goal_achieved",
        higher_is_better=False,
        weight=1.5,  # Higher weight - primary objective
    ),
    "T5": MetricDefinition(
        id="T5",
        name="Sim Step Timing",
        description="Gazebo simulation step period consistency",
        unit=MetricUnit.MILLISECONDS,
        source="gz_sim world update callbacks",
        higher_is_better=False,
        weight=0.3,  # Lower weight - infrastructure metric
    ),
}


# =============================================================================
# CSV SCHEMA
# =============================================================================

# Columns for trial result CSV
RESULT_CSV_COLUMNS = [
    # Identification
    "trial_id",
    "scenario",
    "experiment_id",
    "timestamp",

    # Status
    "status",  # success, planning_failed, execution_failed, timeout, error
    "error_message",

    # Primary metrics (milliseconds)
    "planning_latency_ms",
    "execution_latency_ms",
    "total_latency_ms",

    # Planning details
    "planner_id",
    "planning_time_requested",
    "planning_time_actual",
    "trajectory_points",
    "trajectory_duration_s",

    # Goal pose
    "goal_x",
    "goal_y",
    "goal_z",

    # Trace metadata
    "trace_session",
    "trace_path",
]

# Columns for aggregated summary CSV
SUMMARY_CSV_COLUMNS = [
    "scenario",
    "n_trials",
    "n_success",
    "success_rate",

    # Planning latency stats
    "T1_mean",
    "T1_std",
    "T1_p50",
    "T1_p95",
    "T1_p99",
    "T1_min",
    "T1_max",

    # Execution latency stats
    "T2_mean",
    "T2_std",
    "T2_p50",
    "T2_p95",
    "T2_p99",
    "T2_min",
    "T2_max",

    # Total latency stats
    "T4_mean",
    "T4_std",
    "T4_p50",
    "T4_p95",
    "T4_p99",
    "T4_min",
    "T4_max",
]


# =============================================================================
# WEIGHTING METHODS
# =============================================================================

def compute_weighted_score(
    metrics: Dict[str, float],
    weights: Optional[Dict[str, float]] = None,
    normalize: bool = True
) -> float:
    """
    Compute weighted score from multiple metrics.

    Args:
        metrics: Dict of metric_id -> value
        weights: Optional custom weights (defaults to OBJECTIVE_METRICS weights)
        normalize: Whether to normalize by sum of weights

    Returns:
        Weighted score (lower is better for latency metrics)
    """
    if weights is None:
        weights = {m.id: m.weight for m in OBJECTIVE_METRICS.values()}

    score = 0.0
    total_weight = 0.0

    for metric_id, value in metrics.items():
        if metric_id in weights:
            w = weights[metric_id]
            score += w * value
            total_weight += w

    if normalize and total_weight > 0:
        score /= total_weight

    return score


def sensitivity_analysis(
    baseline_metrics: Dict[str, float],
    perturbed_metrics: Dict[str, float],
) -> Dict[str, float]:
    """
    Compute sensitivity of each metric to perturbation.

    Returns dict of metric_id -> percent change
    """
    sensitivity = {}
    for metric_id in baseline_metrics:
        if metric_id in perturbed_metrics:
            baseline = baseline_metrics[metric_id]
            perturbed = perturbed_metrics[metric_id]
            if baseline > 0:
                pct_change = ((perturbed - baseline) / baseline) * 100
                sensitivity[metric_id] = pct_change
    return sensitivity


def suggest_weights_from_sensitivity(
    sensitivity: Dict[str, float],
    invert: bool = True
) -> Dict[str, float]:
    """
    Suggest weights based on sensitivity analysis.

    Metrics that change more under load should get higher weights
    (or lower if invert=True, focusing on stable metrics).

    Args:
        sensitivity: Dict from sensitivity_analysis()
        invert: If True, give higher weight to less sensitive metrics

    Returns:
        Suggested weights (normalized to sum to 1.0)
    """
    # Use absolute sensitivity values
    abs_sens = {k: abs(v) for k, v in sensitivity.items()}

    if invert:
        # More sensitive = lower weight
        max_sens = max(abs_sens.values()) if abs_sens else 1.0
        weights = {k: (max_sens - v + 0.1) for k, v in abs_sens.items()}
    else:
        # More sensitive = higher weight
        weights = {k: (v + 0.1) for k, v in abs_sens.items()}

    # Normalize
    total = sum(weights.values())
    if total > 0:
        weights = {k: v / total for k, v in weights.items()}

    return weights


# =============================================================================
# TRACE EVENT MAPPING
# =============================================================================

# Mapping from LTTng event names to metric contributions
TRACE_EVENT_METRICS = {
    "ros2:callback_start": ["T3"],  # Used for control loop timing
    "ros2:callback_end": ["T3"],
    "ros2:rcl_publish": ["T2"],  # Trajectory publishing
    "ros2:rclcpp_publish": ["T2"],
    "sched_switch": ["T3", "T5"],  # Context switches affect timing
}


# =============================================================================
# EXPORT FUNCTIONS
# =============================================================================

def export_schema_json(filepath: str):
    """Export schema as JSON for documentation."""
    schema = {
        "objectives": {
            k: {
                "id": v.id,
                "name": v.name,
                "description": v.description,
                "unit": v.unit.value,
                "start_marker": v.start_marker,
                "end_marker": v.end_marker,
                "source": v.source,
                "higher_is_better": v.higher_is_better,
                "default_weight": v.weight,
            }
            for k, v in OBJECTIVE_METRICS.items()
        },
        "csv_columns": {
            "results": RESULT_CSV_COLUMNS,
            "summary": SUMMARY_CSV_COLUMNS,
        },
    }

    with open(filepath, 'w') as f:
        json.dump(schema, f, indent=2)


if __name__ == "__main__":
    # Print schema summary
    print("LDOS Metrics Schema")
    print("=" * 60)
    print("\nObjective Metrics:")
    for metric_id, metric in OBJECTIVE_METRICS.items():
        print(f"  {metric_id}: {metric.name}")
        print(f"      {metric.description}")
        print(f"      Unit: {metric.unit.value}, Weight: {metric.weight}")
        if metric.start_marker:
            print(f"      Markers: {metric.start_marker} -> {metric.end_marker}")
        if metric.source:
            print(f"      Source: {metric.source}")
        print()

    # Export to JSON
    export_schema_json("/tmp/ldos_metrics_schema.json")
    print("Schema exported to /tmp/ldos_metrics_schema.json")

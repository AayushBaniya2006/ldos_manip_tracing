#!/bin/bash
# cpuset_sweep.sh - Run experiments with varying CPU allocations
#
# Usage: ./cpuset_sweep.sh [num_trials] [cpu_values]
#
# Examples:
#   ./cpuset_sweep.sh           # Default: 10 trials, sweep 1,2,4 CPUs
#   ./cpuset_sweep.sh 5         # 5 trials per CPU config
#   ./cpuset_sweep.sh 10 "1 2 4 8"  # Custom CPU values
#
# This script runs the cpuset_limited scenario with different ROS stack
# CPU allocations to find the breaking point where performance degrades.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[SWEEP]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[SWEEP]${NC} $1"; }
log_error() { echo -e "${RED}[SWEEP ERROR]${NC} $1"; }
log_header() { echo -e "\n${CYAN}========================================${NC}"; echo -e "${CYAN} $1${NC}"; echo -e "${CYAN}========================================${NC}\n"; }

# Configuration
NUM_TRIALS="${1:-10}"
CPU_VALUES="${2:-1 2 4}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SWEEP_DIR="${WS_ROOT}/results/cpuset_sweep_${TIMESTAMP}"

# Validate CPU values
TOTAL_CPUS=$(nproc)
log_info "System has $TOTAL_CPUS CPUs"
log_info "Sweep configuration: $NUM_TRIALS trials per CPU config"
log_info "CPU values to test: $CPU_VALUES"

# Create sweep results directory
mkdir -p "$SWEEP_DIR"

# Save sweep configuration
cat > "$SWEEP_DIR/sweep_config.json" << EOF
{
    "timestamp": "$TIMESTAMP",
    "total_cpus": $TOTAL_CPUS,
    "num_trials": $NUM_TRIALS,
    "cpu_values": [$CPU_VALUES],
    "hostname": "$(hostname)",
    "kernel": "$(uname -r)"
}
EOF

# Check cpuset support first
log_info "Verifying cpuset support..."
if ! "$SCRIPT_DIR/check_cpuset_support.sh"; then
    log_error "cpuset support check failed. Cannot proceed with sweep."
    exit 1
fi

# Initialize results tracking
RESULTS_FILE="$SWEEP_DIR/sweep_results.csv"
echo "cpu_count,trials,successes,success_rate,planning_mean_ms,planning_std_ms,execution_mean_ms,execution_std_ms,total_mean_ms,total_std_ms" > "$RESULTS_FILE"

# Run experiments for each CPU configuration
for cpu_count in $CPU_VALUES; do
    log_header "Testing with ROS_CPU_COUNT=$cpu_count"

    # Validate CPU count is possible
    if [ "$cpu_count" -ge "$TOTAL_CPUS" ]; then
        log_warn "Skipping cpu_count=$cpu_count (exceeds available CPUs: $TOTAL_CPUS)"
        continue
    fi

    # Calculate minimum Gazebo CPUs (need at least 2 for stable simulation)
    MIN_GAZEBO_CPUS=2
    if [ "$((TOTAL_CPUS - cpu_count))" -lt "$MIN_GAZEBO_CPUS" ]; then
        log_warn "Skipping cpu_count=$cpu_count (leaves < $MIN_GAZEBO_CPUS CPUs for Gazebo)"
        continue
    fi

    # Set up result directory for this CPU count
    CPU_RESULT_DIR="$SWEEP_DIR/${cpu_count}cpu"
    mkdir -p "$CPU_RESULT_DIR"

    log_info "Running $NUM_TRIALS trials with $cpu_count CPU(s) for ROS stack..."
    log_info "Results will be saved to: $CPU_RESULT_DIR"

    # Start CPU monitoring for this configuration
    CPU_MONITOR_FILE="$CPU_RESULT_DIR/cpu_usage.csv"
    log_info "Starting CPU monitor -> $CPU_MONITOR_FILE"
    "$SCRIPT_DIR/cpu_monitor.sh" start "$CPU_MONITOR_FILE" 0.5

    # Run experiment with this CPU count
    export ROS_CPU_COUNT="$cpu_count"

    if "$SCRIPT_DIR/run_experiment_suite.sh" "$NUM_TRIALS" "cpuset_limited"; then
        log_info "Experiment completed successfully"
    else
        log_warn "Experiment had some failures (this may be expected)"
    fi

    # Stop CPU monitoring
    "$SCRIPT_DIR/cpu_monitor.sh" stop
    log_info "CPU monitor stopped. Data saved to: $CPU_MONITOR_FILE"

    # Move results to sweep directory
    if [ -d "${WS_ROOT}/results/cpuset_limited" ]; then
        mv "${WS_ROOT}/results/cpuset_limited"/* "$CPU_RESULT_DIR/" 2>/dev/null || true
    fi

    # Calculate summary statistics for this CPU count
    if [ -f "$CPU_RESULT_DIR/summary.json" ] || ls "$CPU_RESULT_DIR"/*_result.json >/dev/null 2>&1; then
        log_info "Calculating statistics for $cpu_count CPU(s)..."

        # Use Python to calculate stats from JSON results
        python3 << PYTHON_SCRIPT
import json
import glob
import os

cpu_count = $cpu_count
result_dir = "$CPU_RESULT_DIR"
results_file = "$RESULTS_FILE"

# Find all result JSON files
result_files = glob.glob(os.path.join(result_dir, "*_result.json"))

if not result_files:
    print(f"No result files found for {cpu_count} CPUs")
    exit(0)

# Parse results
planning = []
execution = []
total = []
successes = 0

for f in result_files:
    try:
        with open(f) as fp:
            data = json.load(fp)
            if data.get("status") == "success":
                successes += 1
                if "planning_latency_ms" in data:
                    planning.append(data["planning_latency_ms"])
                if "execution_latency_ms" in data:
                    execution.append(data["execution_latency_ms"])
                if "total_latency_ms" in data:
                    total.append(data["total_latency_ms"])
    except Exception as e:
        print(f"Error reading {f}: {e}")

# Calculate statistics
import statistics

def calc_stats(values):
    if len(values) < 2:
        return (statistics.mean(values) if values else 0, 0)
    return (statistics.mean(values), statistics.stdev(values))

trials = len(result_files)
success_rate = successes / trials if trials > 0 else 0

planning_mean, planning_std = calc_stats(planning)
execution_mean, execution_std = calc_stats(execution)
total_mean, total_std = calc_stats(total)

# Append to CSV
with open(results_file, "a") as f:
    f.write(f"{cpu_count},{trials},{successes},{success_rate:.3f},{planning_mean:.2f},{planning_std:.2f},{execution_mean:.2f},{execution_std:.2f},{total_mean:.2f},{total_std:.2f}\n")

print(f"  Trials: {trials}, Successes: {successes} ({success_rate*100:.1f}%)")
print(f"  Planning: {planning_mean:.2f} ± {planning_std:.2f} ms")
print(f"  Execution: {execution_mean:.2f} ± {execution_std:.2f} ms")
print(f"  Total: {total_mean:.2f} ± {total_std:.2f} ms")
PYTHON_SCRIPT
    else
        log_warn "No results found for $cpu_count CPU(s)"
    fi

    # Cooldown between configurations
    log_info "Cooldown before next configuration..."
    sleep 30
done

# Print final summary
log_header "Sweep Complete"
log_info "Results saved to: $SWEEP_DIR"
echo ""
echo "=== Summary ==="
if [ -f "$RESULTS_FILE" ]; then
    column -t -s',' "$RESULTS_FILE"
fi

# Save final summary
cat > "$SWEEP_DIR/sweep_summary.txt" << EOF
CPUSET Sweep Results
====================
Timestamp: $TIMESTAMP
Total CPUs: $TOTAL_CPUS
Trials per config: $NUM_TRIALS
CPU values tested: $CPU_VALUES

Results:
$(cat "$RESULTS_FILE" 2>/dev/null || echo "No results available")
EOF

log_info "Sweep complete. See $SWEEP_DIR for full results."

# Analyze CPU usage data automatically
log_header "Analyzing CPU Usage Data"
for cpu_dir in "$SWEEP_DIR"/*cpu; do
    if [ -d "$cpu_dir" ] && [ -f "$cpu_dir/cpu_usage.csv" ]; then
        log_info "Analyzing CPU data in: $cpu_dir"
        python3 "$WS_ROOT/analysis/analyze_cpu_usage.py" "$cpu_dir/cpu_usage.csv" "$cpu_dir" || log_warn "Analysis failed for $cpu_dir"
    fi
done

log_info "All analysis complete!"
echo ""
echo "=== Output Files ==="
echo "  Sweep results:     $RESULTS_FILE"
echo "  CPU usage plots:   $SWEEP_DIR/*/cpu_*.png"
echo "  CPU summaries:     $SWEEP_DIR/*/cpu_summary.txt"

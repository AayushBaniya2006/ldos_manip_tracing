#!/bin/bash
# cpuset_sweep.sh - Run experiments with varying CPU allocations
#
# Usage:
#   ./cpuset_sweep.sh [num_trials] [cpu_values]        # positional args
#   ./cpuset_sweep.sh --trials 10 --cpus "1 2 4 8"     # named flags
#   ./cpuset_sweep.sh --cpus "1,2,4,8" --trials 5      # commas OK
#
# Examples:
#   ./cpuset_sweep.sh                     # Default: 10 trials, sweep 1,2,4 CPUs
#   ./cpuset_sweep.sh 5                   # 5 trials per CPU config
#   ./cpuset_sweep.sh 10 "1 2 4 8"       # Custom CPU values (positional)
#   ./cpuset_sweep.sh --trials 10 --cpus "1,2,3,4,6,8"  # Named flags
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

# Parse arguments: support both positional and named flags
NUM_TRIALS=""
CPU_VALUES=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --trials|-t)
            NUM_TRIALS="$2"
            shift 2
            ;;
        --cpus|--cpu-list|-c)
            CPU_VALUES="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [--trials N] [--cpus \"1 2 4 8\"]"
            echo "       $0 [num_trials] [cpu_values]"
            echo ""
            echo "Options:"
            echo "  --trials, -t N          Number of trials per CPU config (default: 10)"
            echo "  --cpus, --cpu-list, -c  Space or comma-separated CPU counts (default: 1 2 4)"
            echo ""
            echo "Environment variables:"
            echo "  ENABLE_TRACING_SWEEP    Enable LTTng tracing (default: 0)"
            echo "  MAX_INVALID_RERUNS      Retry invalid runs (default: 1)"
            exit 0
            ;;
        *)
            # Positional argument fallback
            if [[ -z "$NUM_TRIALS" ]]; then
                NUM_TRIALS="$1"
            elif [[ -z "$CPU_VALUES" ]]; then
                CPU_VALUES="$1"
            else
                log_error "Unknown argument: $1"
                exit 1
            fi
            shift
            ;;
    esac
done

# Apply defaults
NUM_TRIALS="${NUM_TRIALS:-10}"
CPU_VALUES="${CPU_VALUES:-1 2 4}"

# Normalize: convert commas to spaces in CPU_VALUES
CPU_VALUES="$(echo "$CPU_VALUES" | tr ',' ' ')"

# Validate NUM_TRIALS is a positive integer
if ! [[ "$NUM_TRIALS" =~ ^[0-9]+$ ]] || [ "$NUM_TRIALS" -le 0 ]; then
    log_error "Invalid --trials value: '$NUM_TRIALS' (must be a positive integer)"
    exit 1
fi

# Validate each CPU value is a positive integer
for v in $CPU_VALUES; do
    if ! [[ "$v" =~ ^[0-9]+$ ]] || [ "$v" -le 0 ]; then
        log_error "Invalid CPU count in --cpus: '$v' (must be a positive integer)"
        exit 1
    fi
done
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SWEEP_DIR="${WS_ROOT}/results/cpuset_sweep_${TIMESTAMP}"
ENABLE_TRACING_SWEEP="${ENABLE_TRACING_SWEEP:-0}"
MAX_INVALID_RERUNS="${MAX_INVALID_RERUNS:-1}"
FAIL_FAST_START_STATE_INVALID_STREAK="${FAIL_FAST_START_STATE_INVALID_STREAK:-3}"

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
    "cpu_values": [$(echo $CPU_VALUES | sed 's/ /, /g')],
    "enable_tracing_sweep": $ENABLE_TRACING_SWEEP,
    "max_invalid_reruns": $MAX_INVALID_RERUNS,
    "fail_fast_start_state_invalid_streak": $FAIL_FAST_START_STATE_INVALID_STREAK,
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

cleanup_between_runs() {
    if [ -x "$SCRIPT_DIR/cleanup_harness_processes.sh" ]; then
        "$SCRIPT_DIR/cleanup_harness_processes.sh" 5 || true
    else
        "$SCRIPT_DIR/cpuset_cleanup.sh" 2>/dev/null || true
        pkill -9 -f "gz sim" 2>/dev/null || true
        pkill -9 -f "gazebo" 2>/dev/null || true
        pkill -9 -f "move_group" 2>/dev/null || true
        pkill -9 -f "robot_state_pub" 2>/dev/null || true
        pkill -9 -f "parameter_bridge" 2>/dev/null || true
        pkill -9 -f "ros2.*launch" 2>/dev/null || true
        pkill -9 -f "ros2.*control" 2>/dev/null || true
        command -v lttng >/dev/null 2>&1 && lttng destroy --all >/dev/null 2>&1 || true
        sleep 5
    fi
}

latest_experiment_id() {
    local latest_file="$WS_ROOT/results/.latest_experiment_id"
    if [ -f "$latest_file" ]; then
        cat "$latest_file"
    fi
}

archive_attempt_artifacts() {
    local cpu_result_dir="$1"
    local attempt_dir="$2"
    local experiment_id="$3"
    mkdir -p "$attempt_dir"

    if [ -n "$experiment_id" ]; then
        [ -f "$WS_ROOT/results/${experiment_id}_log.txt" ] && cp "$WS_ROOT/results/${experiment_id}_log.txt" "$attempt_dir/" || true
        [ -f "$WS_ROOT/results/${experiment_id}_config.json" ] && cp "$WS_ROOT/results/${experiment_id}_config.json" "$attempt_dir/" || true
        [ -d "$WS_ROOT/results/$experiment_id" ] && cp -R "$WS_ROOT/results/$experiment_id" "$attempt_dir/" || true
    fi

    mkdir -p "$attempt_dir/cpuset_limited_results"
    if [ -d "${WS_ROOT}/results/cpuset_limited" ]; then
        cp "${WS_ROOT}/results/cpuset_limited"/* "$attempt_dir/cpuset_limited_results/" 2>/dev/null || true
        # Also refresh root-level copy with the latest attempt for convenience
        cp "${WS_ROOT}/results/cpuset_limited"/* "$cpu_result_dir/" 2>/dev/null || true
    fi

    mkdir -p "$attempt_dir/analysis_output"
    [ -f "$WS_ROOT/analysis/output/combined_summary.csv" ] && cp "$WS_ROOT/analysis/output/combined_summary.csv" "$attempt_dir/analysis_output/" || true
    [ -f "$WS_ROOT/analysis/output/cpuset_limited/summary.csv" ] && cp "$WS_ROOT/analysis/output/cpuset_limited/summary.csv" "$attempt_dir/analysis_output/" || true
}

# Initialize results tracking
RESULTS_FILE="$SWEEP_DIR/sweep_results.csv"
echo "cpu_count,trials,successes,success_rate,planning_mean_ms,planning_std_ms,execution_mean_ms,execution_std_ms,total_mean_ms,total_std_ms,valid,invalid_reason,start_state_invalid_count,cpu_monitor_rows,experiment_id,isolation_verified" > "$RESULTS_FILE"

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

    CPU_RESULT_DIR="$SWEEP_DIR/${cpu_count}cpu"
    mkdir -p "$CPU_RESULT_DIR"
    CPU_MONITOR_FILE="$CPU_RESULT_DIR/cpu_usage.csv"

    ATTEMPT=0
    ROW_WRITTEN=false
    while [ "$ATTEMPT" -le "$MAX_INVALID_RERUNS" ]; do
        [ "$ATTEMPT" -gt 0 ] && log_warn "Retrying cpu_count=$cpu_count after invalid run (attempt $ATTEMPT/$MAX_INVALID_RERUNS)"

        log_info "Running $NUM_TRIALS trials with $cpu_count CPU(s) for ROS stack..."
        log_info "Results will be saved to: $CPU_RESULT_DIR"

        # Clean state before each attempt and clear previous scenario results.
        cleanup_between_runs
        rm -rf "${WS_ROOT}/results/cpuset_limited" 2>/dev/null || true
        rm -f "$CPU_MONITOR_FILE"

        # Start CPU monitoring for this configuration
        log_info "Starting CPU monitor -> $CPU_MONITOR_FILE"
        "$SCRIPT_DIR/cpu_monitor.sh" start "$CPU_MONITOR_FILE" 0.5

        # Run experiment with this CPU count (CPU-first mode disables tracing by default)
        export ROS_CPU_COUNT="$cpu_count"
        export ENABLE_TRACING="$ENABLE_TRACING_SWEEP"
        export FAIL_FAST_START_STATE_INVALID_STREAK="$FAIL_FAST_START_STATE_INVALID_STREAK"

        if "$SCRIPT_DIR/run_experiment_suite.sh" "$NUM_TRIALS" "cpuset_limited"; then
            log_info "Experiment completed successfully"
        else
            log_warn "Experiment had failures (will validate result semantics before accepting)"
        fi

        # Stop CPU monitoring
        "$SCRIPT_DIR/cpu_monitor.sh" stop
        log_info "CPU monitor stopped. Data saved to: $CPU_MONITOR_FILE"

        CPU_MONITOR_ROWS=0
        if [ -f "$CPU_MONITOR_FILE" ]; then
            CPU_MONITOR_ROWS=$(wc -l < "$CPU_MONITOR_FILE" | tr -d ' ')
            if [ "$CPU_MONITOR_ROWS" -le 1 ]; then
                log_warn "CPU monitor CSV is empty (header only): $CPU_MONITOR_FILE"
            fi
        else
            log_warn "CPU monitor output missing: $CPU_MONITOR_FILE"
        fi

        EXPERIMENT_ID="$(latest_experiment_id)"
        EXPERIMENT_LOG="${WS_ROOT}/results/${EXPERIMENT_ID}_log.txt"
        ATTEMPT_DIR="$CPU_RESULT_DIR/attempt_${ATTEMPT}"
        archive_attempt_artifacts "$CPU_RESULT_DIR" "$ATTEMPT_DIR" "$EXPERIMENT_ID"

        INVALID_REASON=""
        # Check isolation verification from experiment log
        ISOLATION_VERIFIED="unknown"
        if [ -f "$EXPERIMENT_LOG" ]; then
            if grep -q "CPU isolation VERIFIED:" "$EXPERIMENT_LOG"; then
                ISOLATION_VERIFIED="true"
            elif grep -q "CPU isolation PARTIAL:" "$EXPERIMENT_LOG" || grep -q "\[FAIL\].*affinity=" "$EXPERIMENT_LOG"; then
                ISOLATION_VERIFIED="false"
                INVALID_REASON="isolation_not_verified"
                log_warn "CPU isolation verification FAILED for $cpu_count CPU(s)"
            fi
        fi

        # Calculate summary statistics and detect invalid START_STATE_INVALID runs.
        log_info "Calculating statistics for $cpu_count CPU(s)..."
        SUMMARY_JSON="$ATTEMPT_DIR/run_summary.json"
        SUMMARY_ROW_FILE="$ATTEMPT_DIR/summary_row.csv"
        python3 << PYTHON_SCRIPT
import json, glob, os, statistics, sys

cpu_count = $cpu_count
root_result_dir = "$CPU_RESULT_DIR"
attempt_result_dir = "$ATTEMPT_DIR/cpuset_limited_results"
results_file = "$RESULTS_FILE"
summary_json = "$SUMMARY_JSON"
summary_row_file = "$SUMMARY_ROW_FILE"
invalid_reason_seed = "$INVALID_REASON"
cpu_monitor_rows = int("$CPU_MONITOR_ROWS" or "0")
experiment_id = "$EXPERIMENT_ID"
isolation_verified = "$ISOLATION_VERIFIED"

result_dir = attempt_result_dir if os.path.isdir(attempt_result_dir) else root_result_dir
result_files = sorted(glob.glob(os.path.join(result_dir, "*_result.json")))

planning, execution, total = [], [], []
successes = 0
status_counts = {}
start_state_invalid_count = 0

for f in result_files:
    try:
        with open(f) as fp:
            data = json.load(fp)
    except Exception as e:
        print(f"Error reading {f}: {e}")
        continue
    status = data.get("status", "unknown")
    status_counts[status] = status_counts.get(status, 0) + 1
    if status == "start_state_invalid" or int(data.get("moveit_error_code") or 0) == -26:
        start_state_invalid_count += 1
    if status == "success":
        successes += 1
        if "planning_latency_ms" in data:
            planning.append(data["planning_latency_ms"])
        if "execution_latency_ms" in data:
            execution.append(data["execution_latency_ms"])
        if "total_latency_ms" in data:
            total.append(data["total_latency_ms"])

def calc_stats(values):
    if not values:
        return (0.0, 0.0)
    if len(values) < 2:
        return (float(statistics.mean(values)), 0.0)
    return (float(statistics.mean(values)), float(statistics.stdev(values)))

trials = len(result_files)
success_rate = (successes / trials) if trials else 0.0
planning_mean, planning_std = calc_stats(planning)
execution_mean, execution_std = calc_stats(execution)
total_mean, total_std = calc_stats(total)

invalid_reason = invalid_reason_seed
if trials == 0:
    invalid_reason = "no_results" if not invalid_reason else f"{invalid_reason}+no_results"
if start_state_invalid_count > 0:
    invalid_reason = "start_state_invalid" if not invalid_reason else f"{invalid_reason}+start_state_invalid"

valid = "true" if not invalid_reason else "false"
summary = {
    "cpu_count": cpu_count,
    "trials": trials,
    "successes": successes,
    "success_rate": success_rate,
    "planning_mean_ms": planning_mean,
    "planning_std_ms": planning_std,
    "execution_mean_ms": execution_mean,
    "execution_std_ms": execution_std,
    "total_mean_ms": total_mean,
    "total_std_ms": total_std,
    "valid": valid == "true",
    "invalid_reason": invalid_reason,
    "start_state_invalid_count": start_state_invalid_count,
    "cpu_monitor_rows": cpu_monitor_rows,
    "experiment_id": experiment_id,
    "isolation_verified": isolation_verified,
    "status_counts": status_counts,
}
with open(summary_json, "w") as f:
    json.dump(summary, f, indent=2)

row = ",".join([
    str(cpu_count),
    str(trials),
    str(successes),
    f"{success_rate:.3f}",
    f"{planning_mean:.2f}",
    f"{planning_std:.2f}",
    f"{execution_mean:.2f}",
    f"{execution_std:.2f}",
    f"{total_mean:.2f}",
    f"{total_std:.2f}",
    valid,
    invalid_reason or "",
    str(start_state_invalid_count),
    str(cpu_monitor_rows),
    experiment_id,
    isolation_verified,
])
with open(summary_row_file, "w") as f:
    f.write(row + "\n")

# Emit row to stdout for visibility
print(row)
print(f"  Trials: {trials}, Successes: {successes} ({success_rate*100:.1f}%)", file=sys.stderr)
print(f"  Status counts: {status_counts}", file=sys.stderr)
print(f"  Planning: {planning_mean:.2f} +/- {planning_std:.2f} ms", file=sys.stderr)
print(f"  Execution: {execution_mean:.2f} +/- {execution_std:.2f} ms", file=sys.stderr)
print(f"  Total: {total_mean:.2f} +/- {total_std:.2f} ms", file=sys.stderr)
if invalid_reason:
    print(f"  INVALID RUN: {invalid_reason}", file=sys.stderr)
if cpu_monitor_rows <= 1:
    print("  CPU monitor data missing/empty (plots not trustworthy)", file=sys.stderr)
PYTHON_SCRIPT
        SUMMARY_ROW="$(tr -d '\n' < "$SUMMARY_ROW_FILE" 2>/dev/null || true)"

        RUN_VALID="$(python3 -c 'import json,sys; print("true" if json.load(open(sys.argv[1])).get("valid") else "false")' "$SUMMARY_JSON")"
        RUN_INVALID_REASON="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("invalid_reason",""))' "$SUMMARY_JSON")"

        if [ "$RUN_VALID" = "true" ] || [ "$ATTEMPT" -ge "$MAX_INVALID_RERUNS" ]; then
            echo "$SUMMARY_ROW" >> "$RESULTS_FILE"
            ROW_WRITTEN=true
            break
        fi

        log_warn "cpu_count=$cpu_count attempt $ATTEMPT invalid ($RUN_INVALID_REASON); retrying after cleanup"
        ATTEMPT=$((ATTEMPT + 1))
    done

    if [ "$ROW_WRITTEN" = false ]; then
        log_warn "No summary row was written for cpu_count=$cpu_count (unexpected)"
    fi

    # Cleanup before next configuration
    log_info "Cleaning up before next configuration..."
    cleanup_between_runs
    rm -rf "${WS_ROOT}/results/cpuset_limited" 2>/dev/null || true

    log_info "Cooldown before next configuration..."
    sleep 15
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
        cpu_rows=$(wc -l < "$cpu_dir/cpu_usage.csv" | tr -d ' ')
        if [ "$cpu_rows" -le 1 ]; then
            log_warn "Skipping CPU analysis for $cpu_dir (empty cpu_usage.csv)"
            continue
        fi
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

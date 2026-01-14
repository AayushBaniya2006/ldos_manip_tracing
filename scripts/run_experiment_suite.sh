#!/bin/bash
# run_experiment_suite.sh - Run complete experiment suite with all scenarios
# Usage: ./run_experiment_suite.sh [num_trials] [scenarios]
#
# This script runs:
#   1. Baseline trials (no load)
#   2. CPU load trials
#   3. Message load trials
#   4. Analysis and aggregation
#
# Each scenario runs N trials with tracing enabled.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

NUM_TRIALS="${1:-10}"
SCENARIOS="${2:-baseline,cpu_load,msg_load}"
WARMUP_TRIALS=2
COOLDOWN_SECONDS=5
MAX_RETRIES=2  # Retry failed trials up to this many times

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[SUITE]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_section() { echo -e "\n${CYAN}======================================${NC}"; echo -e "${CYAN}=== $1 ===${NC}"; echo -e "${CYAN}======================================${NC}"; }

# Source ROS (disable -u temporarily for ROS setup scripts)
set +u
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
fi
set -u

log_section "LDOS Experiment Suite"
log_info "Trials per scenario: $NUM_TRIALS"
log_info "Warmup trials: $WARMUP_TRIALS (not recorded)"
log_info "Scenarios: $SCENARIOS"
log_info "Workspace: $WS_ROOT"

# Parse scenarios
IFS=',' read -ra SCENARIO_ARRAY <<< "$SCENARIOS"

# Global experiment metadata
EXPERIMENT_ID="exp_$(date +%Y%m%d_%H%M%S)"
EXPERIMENT_LOG="$WS_ROOT/results/${EXPERIMENT_ID}_log.txt"
mkdir -p "$WS_ROOT/results"

log_info "Experiment ID: $EXPERIMENT_ID"
log_info "Log file: $EXPERIMENT_LOG"

# Save experiment config
cat > "$WS_ROOT/results/${EXPERIMENT_ID}_config.json" << EOF
{
    "experiment_id": "$EXPERIMENT_ID",
    "start_time": "$(date -Iseconds)",
    "num_trials": $NUM_TRIALS,
    "warmup_trials": $WARMUP_TRIALS,
    "scenarios": [$(echo "${SCENARIO_ARRAY[@]}" | sed 's/ /", "/g' | sed 's/^/"/' | sed 's/$/"/')],
    "cooldown_seconds": $COOLDOWN_SECONDS
}
EOF

# Track results
declare -A SCENARIO_SUCCESS
declare -A SCENARIO_FAILED

run_scenario() {
    local scenario="$1"
    local num_trials="$2"

    log_section "Scenario: $scenario"

    SCENARIO_SUCCESS[$scenario]=0
    SCENARIO_FAILED[$scenario]=0

    # Start the ROS stack once for all trials in this scenario
    log_info "Starting ROS stack for $scenario..."

    ros2 launch ldos_harness full_stack.launch.py headless:=true use_rviz:=false &
    STACK_PID=$!

    # Wait for stack to initialize with polling (timeout: 120s)
    log_info "Waiting for stack to initialize (polling, max 120s)..."
    STACK_TIMEOUT=120
    STACK_POLL_INTERVAL=5
    STACK_ELAPSED=0

    while [ $STACK_ELAPSED -lt $STACK_TIMEOUT ]; do
        # Check if MoveGroup action is available
        if ros2 action list 2>/dev/null | grep -q "move_action"; then
            log_info "Stack ready after ${STACK_ELAPSED}s"
            break
        fi

        # Check if stack process is still running
        if ! kill -0 $STACK_PID 2>/dev/null; then
            log_error "Stack process died unexpectedly"
            return 1
        fi

        sleep $STACK_POLL_INTERVAL
        STACK_ELAPSED=$((STACK_ELAPSED + STACK_POLL_INTERVAL))
        log_info "  Still waiting... (${STACK_ELAPSED}s / ${STACK_TIMEOUT}s)"
    done

    # Final verification after polling loop
    if ! ros2 action list 2>/dev/null | grep -q "move_action"; then
        log_error "Stack failed to start for $scenario (timeout after ${STACK_TIMEOUT}s)"
        kill $STACK_PID 2>/dev/null || true
        return 1
    fi

    # Start load generator if needed
    LOAD_PID=""
    if [ "$scenario" = "cpu_load" ]; then
        log_info "Starting CPU load..."
        "$SCRIPT_DIR/cpu_load.sh" 4 $((num_trials * 120)) &
        LOAD_PID=$!
        sleep 3
    elif [ "$scenario" = "msg_load" ]; then
        log_info "Starting message load..."
        "$SCRIPT_DIR/msg_load.sh" 1000 4 $((num_trials * 120)) &
        LOAD_PID=$!
        sleep 3
    fi

    # Run warmup trials
    log_info "Running $WARMUP_TRIALS warmup trials..."
    for w in $(seq 1 $WARMUP_TRIALS); do
        log_info "Warmup trial $w/$WARMUP_TRIALS"
        ros2 run ldos_harness benchmark_runner.py \
            --trial-id "warmup_${scenario}_$(printf '%03d' "$w")" \
            --scenario "$scenario" \
            --output-dir "$WS_ROOT/results/warmup" \
            --timeout 60.0 2>&1 || true
        sleep 2
    done

    # Run actual trials
    log_info "Running $num_trials data collection trials..."
    for trial in $(seq 1 $num_trials); do
        TRIAL_ID="${scenario}_$(printf '%03d' "$trial")"
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        TRACE_SESSION="ldos_${TRIAL_ID}_${TIMESTAMP}"
        TRACE_DIR="$WS_ROOT/traces/$TRACE_SESSION"
        RESULT_DIR="$WS_ROOT/results/$scenario"

        log_info "Trial $trial/$num_trials: $TRIAL_ID"

        mkdir -p "$TRACE_DIR"
        mkdir -p "$RESULT_DIR"

        # Collect metadata using Python for safe JSON encoding
        python3 -c "
import json
import os
import subprocess
from datetime import datetime

# Get CPU model safely
try:
    with open('/proc/cpuinfo', 'r') as f:
        for line in f:
            if 'model name' in line:
                cpu_model = line.split(':')[1].strip()
                break
        else:
            cpu_model = 'unknown'
except:
    cpu_model = 'unknown'

metadata = {
    'trial_id': '$TRIAL_ID',
    'experiment_id': '$EXPERIMENT_ID',
    'scenario': '$scenario',
    'trial_number': $trial,
    'timestamp': datetime.now().isoformat(),
    'trace_session': '$TRACE_SESSION',
    'environment': {
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        'hostname': subprocess.run(['hostname'], capture_output=True, text=True).stdout.strip(),
        'kernel': subprocess.run(['uname', '-r'], capture_output=True, text=True).stdout.strip(),
        'cpu_model': cpu_model
    }
}

with open('$RESULT_DIR/${TRIAL_ID}_metadata.json', 'w') as f:
    json.dump(metadata, f, indent=4)
"

        # Retry loop for transient failures
        TRIAL_SUCCESS=false
        for retry in $(seq 0 $MAX_RETRIES); do
            if [ $retry -gt 0 ]; then
                log_info "  Retry $retry/$MAX_RETRIES for $TRIAL_ID"
                # Update trace session name for retry
                TIMESTAMP=$(date +%Y%m%d_%H%M%S)
                TRACE_SESSION="ldos_${TRIAL_ID}_retry${retry}_${TIMESTAMP}"
                TRACE_DIR="$WS_ROOT/traces/$TRACE_SESSION"
                mkdir -p "$TRACE_DIR"
                sleep 2  # Brief pause before retry
            fi

            # Start trace
            "$SCRIPT_DIR/start_trace.sh" "$TRACE_SESSION" "$TRACE_DIR" > /dev/null 2>&1

            sleep 1

            # Run benchmark
            BENCH_EXIT=0
            ros2 run ldos_harness benchmark_runner.py \
                --trial-id "$TRIAL_ID" \
                --scenario "$scenario" \
                --output-dir "$RESULT_DIR" \
                --timeout 60.0 2>&1 || BENCH_EXIT=$?

            # Stop trace
            "$SCRIPT_DIR/stop_trace.sh" "$TRACE_SESSION" > /dev/null 2>&1 || true

            # Verify result file was created
            RESULT_FILE="$RESULT_DIR/${TRIAL_ID}_result.json"
            RESULT_FILE_EXISTS=false
            if [[ -f "$RESULT_FILE" ]]; then
                RESULT_FILE_EXISTS=true
            fi

            # Check if successful
            if [ $BENCH_EXIT -eq 0 ] && [ "$RESULT_FILE_EXISTS" = true ]; then
                TRIAL_SUCCESS=true
                break
            fi

            # Log retry reason
            if [ $retry -lt $MAX_RETRIES ]; then
                if [ $BENCH_EXIT -ne 0 ]; then
                    log_warn "  Trial failed (exit code $BENCH_EXIT), will retry..."
                elif [ "$RESULT_FILE_EXISTS" = false ]; then
                    log_warn "  Result file not created, will retry..."
                fi
            fi
        done

        # Track final success/failure after all retries
        if [ "$TRIAL_SUCCESS" = true ]; then
            SCENARIO_SUCCESS[$scenario]=$((${SCENARIO_SUCCESS[$scenario]} + 1))
            if [ $retry -gt 0 ]; then
                log_info "  Result: SUCCESS (after $retry retries)"
            else
                log_info "  Result: SUCCESS"
            fi
        else
            SCENARIO_FAILED[$scenario]=$((${SCENARIO_FAILED[$scenario]} + 1))
            if [ $BENCH_EXIT -ne 0 ]; then
                log_warn "  Result: FAILED (exit code $BENCH_EXIT after $MAX_RETRIES retries)"
            elif [ "$RESULT_FILE_EXISTS" = false ]; then
                log_warn "  Result: FAILED (result file not created after $MAX_RETRIES retries)"
            fi
        fi

        # Cooldown between trials
        if [ $trial -lt $num_trials ]; then
            sleep $COOLDOWN_SECONDS
        fi
    done

    # Stop load generator
    if [ -n "$LOAD_PID" ] && kill -0 "$LOAD_PID" 2>/dev/null; then
        log_info "Stopping load generator..."
        kill "$LOAD_PID" 2>/dev/null || true
        wait "$LOAD_PID" 2>/dev/null || true
    fi

    # Stop stack
    log_info "Stopping ROS stack..."
    kill $STACK_PID 2>/dev/null || true
    wait $STACK_PID 2>/dev/null || true

    # Cool down before next scenario
    log_info "Cooling down..."
    sleep 10

    log_info "Scenario $scenario complete: ${SCENARIO_SUCCESS[$scenario]} success, ${SCENARIO_FAILED[$scenario]} failed"
}

# Run all scenarios
for scenario in "${SCENARIO_ARRAY[@]}"; do
    run_scenario "$scenario" "$NUM_TRIALS" 2>&1 | tee -a "$EXPERIMENT_LOG"
done

log_section "Running Analysis"
"$SCRIPT_DIR/analyze_traces.sh" all 2>&1 | tee -a "$EXPERIMENT_LOG"

log_section "Experiment Complete"
log_info "Experiment ID: $EXPERIMENT_ID"
log_info ""

# Print summary
log_info "Results Summary:"
for scenario in "${SCENARIO_ARRAY[@]}"; do
    log_info "  $scenario: ${SCENARIO_SUCCESS[$scenario]:-0} success, ${SCENARIO_FAILED[$scenario]:-0} failed"
done

log_info ""
log_info "Results: $WS_ROOT/results/"
log_info "Traces:  $WS_ROOT/traces/"
log_info "Analysis: $WS_ROOT/analysis/output/"
log_info "Log: $EXPERIMENT_LOG"

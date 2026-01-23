#!/bin/bash
# run_single_trial.sh - Run a single benchmark trial with tracing
# Usage: ./run_single_trial.sh <scenario> <trial_num> [skip_bringup]
#
# Arguments:
#   scenario   - baseline, cpu_load, or msg_load
#   trial_num  - Trial number (e.g., 001)
#   skip_bringup - If "true", assumes stack is already running
#
# This script:
# 1. Optionally starts the full stack (Gazebo + MoveIt)
# 2. Starts tracing
# 3. Runs the benchmark
# 4. Stops tracing
# 5. Collects metadata
# 6. Optionally shuts down the stack

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

# Configurable parameters
MOVEIT_ACTION_NAME="${MOVEIT_ACTION_NAME:-move_action}"
INIT_WAIT="${INIT_WAIT:-25}"  # Must match moveit_delay in full_stack.launch.py
BENCHMARK_TIMEOUT="${BENCHMARK_TIMEOUT:-60.0}"

# Arguments
SCENARIO="${1:-baseline}"
TRIAL_NUM="${2:-001}"
SKIP_BRINGUP="${3:-false}"

# Derived names
TRIAL_ID="${SCENARIO}_$(printf '%03d' "$TRIAL_NUM")"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
TRACE_SESSION="ldos_${TRIAL_ID}_${TIMESTAMP}"
RESULT_DIR="$WS_ROOT/results/$SCENARIO"
TRACE_DIR="$WS_ROOT/traces/$TRACE_SESSION"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_section() { echo -e "\n${CYAN}=== $1 ===${NC}"; }

# Cleanup function
STACK_PID=""
LOAD_PID=""
cleanup() {
    log_section "Cleanup"

    # Stop load generators
    if [ -n "$LOAD_PID" ] && kill -0 "$LOAD_PID" 2>/dev/null; then
        log_info "Stopping load generator..."
        kill "$LOAD_PID" 2>/dev/null || true
        wait "$LOAD_PID" 2>/dev/null || true
    fi

    # Stop tracing if active
    if lttng list 2>/dev/null | grep -q "^$TRACE_SESSION "; then
        log_info "Stopping trace..."
        "$SCRIPT_DIR/stop_trace.sh" "$TRACE_SESSION" || true
    fi

    # Stop stack if we started it
    if [ -n "$STACK_PID" ] && kill -0 "$STACK_PID" 2>/dev/null; then
        log_info "Stopping ROS stack..."
        kill "$STACK_PID" 2>/dev/null || true
        wait "$STACK_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# =============================================================================
# ROS 2 ENVIRONMENT SETUP (function-based for stability with strict mode)
# =============================================================================

source_ros() {
    set +u

    local ros_found=false
    for distro in jazzy humble iron rolling; do
        if [ -f "/opt/ros/${distro}/setup.bash" ]; then
            source "/opt/ros/${distro}/setup.bash"
            ros_found=true
            break
        fi
    done

    if [ "$ros_found" = false ]; then
        set -u
        log_error "No ROS 2 installation found in /opt/ros/"
        exit 1
    fi

    if [ -f "$WS_ROOT/install/setup.bash" ]; then
        source "$WS_ROOT/install/setup.bash"
    else
        set -u
        log_error "Workspace not built. Run 'make setup' first."
        exit 1
    fi

    if [ -f "$WS_ROOT/.venv/bin/activate" ]; then
        source "$WS_ROOT/.venv/bin/activate"
    fi

    set -u
}

source_ros

log_section "Trial: $TRIAL_ID"
log_info "Scenario: $SCENARIO"
log_info "Trace session: $TRACE_SESSION"
log_info "Result dir: $RESULT_DIR"

mkdir -p "$RESULT_DIR"
mkdir -p "$TRACE_DIR"

# Collect metadata
log_section "Collecting Metadata"
METADATA_FILE="$RESULT_DIR/${TRIAL_ID}_metadata.json"

collect_metadata() {
    local git_hash="unknown"
    if [ -d "$WS_ROOT/.git" ]; then
        git_hash=$(cd "$WS_ROOT" && git rev-parse --short HEAD 2>/dev/null || echo "unknown")
    fi

    cat > "$METADATA_FILE" << EOF
{
    "trial_id": "$TRIAL_ID",
    "scenario": "$SCENARIO",
    "timestamp": "$(date -Iseconds)",
    "trace_session": "$TRACE_SESSION",
    "trace_dir": "$TRACE_DIR",
    "environment": {
        "ros_distro": "${ROS_DISTRO:-unknown}",
        "ros_version": "${ROS_VERSION:-unknown}",
        "hostname": "$(hostname)",
        "kernel": "$(uname -r)",
        "cpu_model": "$(grep -m1 'model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2 | xargs || echo 'unknown')",
        "cpu_cores": "$(nproc 2>/dev/null || echo 'unknown')",
        "memory_gb": "$(awk '/MemTotal/ {printf "%.1f", $2/1024/1024}' /proc/meminfo 2>/dev/null || echo 'unknown')"
    },
    "config": {
        "git_hash": "$git_hash",
        "controller_rate_hz": 500,
        "sim_step_size": 0.001,
        "planning_time_s": 5.0,
        "planner_id": "RRTConnect"
    }
}
EOF
    log_info "Metadata saved: $METADATA_FILE"
}
collect_metadata

# Start the stack if not skipping
if [ "$SKIP_BRINGUP" != "true" ]; then
    log_section "Starting ROS Stack"
    log_info "Launching Gazebo + Panda + MoveIt..."

    ros2 launch ldos_harness full_stack.launch.py headless:=true use_rviz:=false &
    STACK_PID=$!

    # Wait for stack to be ready
    log_info "Waiting for stack to initialize (${INIT_WAIT}s)..."
    sleep "$INIT_WAIT"

    # Verify MoveGroup is available
    if ! ros2 action list 2>/dev/null | grep -q "$MOVEIT_ACTION_NAME"; then
        log_error "MoveGroup action '$MOVEIT_ACTION_NAME' not found. Stack may have failed to start."
        exit 1
    fi
    log_info "Stack is ready"
else
    log_info "Skipping bringup (assuming stack is running)"
fi

# Start load generator if needed
if [ "$SCENARIO" = "cpu_load" ]; then
    log_section "Starting CPU Load"
    "$SCRIPT_DIR/cpu_load.sh" &
    LOAD_PID=$!
    sleep 2
elif [ "$SCENARIO" = "msg_load" ]; then
    log_section "Starting Message Load"
    "$SCRIPT_DIR/msg_load.sh" &
    LOAD_PID=$!
    sleep 2
fi

# Start tracing
log_section "Starting Tracing"
"$SCRIPT_DIR/start_trace.sh" "$TRACE_SESSION" "$TRACE_DIR"

# Brief settle time
sleep 1

# Run benchmark
log_section "Running Benchmark"
BENCHMARK_EXIT=0
ros2 run ldos_harness benchmark_runner.py \
    --trial-id "$TRIAL_ID" \
    --scenario "$SCENARIO" \
    --output-dir "$RESULT_DIR" \
    --timeout "$BENCHMARK_TIMEOUT" \
    --action-name "$MOVEIT_ACTION_NAME" || BENCHMARK_EXIT=$?

# Stop tracing
log_section "Stopping Tracing"
"$SCRIPT_DIR/stop_trace.sh" "$TRACE_SESSION"

# Update metadata with trace path
if [ -f "$METADATA_FILE" ]; then
    # Add trace completion info using Python (more reliable than sed for JSON)
    python3 << EOF
import json
with open("$METADATA_FILE", "r") as f:
    meta = json.load(f)
meta["trace_completed"] = True
meta["benchmark_exit_code"] = $BENCHMARK_EXIT
with open("$METADATA_FILE", "w") as f:
    json.dump(meta, f, indent=2)
EOF
fi

# Summary
log_section "Trial Complete"
log_info "Trial ID: $TRIAL_ID"
log_info "Benchmark exit code: $BENCHMARK_EXIT"
log_info "Result: $RESULT_DIR/${TRIAL_ID}_result.json"
log_info "Trace: $TRACE_DIR"
log_info "Metadata: $METADATA_FILE"

exit $BENCHMARK_EXIT

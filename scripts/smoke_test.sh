#!/bin/bash
# smoke_test.sh - Quick validation of the LDOS tracing harness
#
# This script validates the full pipeline in <60 seconds by:
# 1. Starting the full stack (Gazebo + MoveIt)
# 2. Verifying MoveGroup action is available
# 3. Running a single benchmark trial
# 4. Checking result file was created
#
# Usage: ./scripts/smoke_test.sh

set -euo pipefail

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

# Configurable parameters
MOVEIT_ACTION_NAME="${MOVEIT_ACTION_NAME:-move_action}"
INIT_WAIT="${INIT_WAIT:-25}"
STACK_TIMEOUT="${STACK_TIMEOUT:-50}"
BENCHMARK_TIMEOUT="${BENCHMARK_TIMEOUT:-30}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Logging functions
log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_pass() { echo -e "${GREEN}[PASS]${NC} $1"; }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; }

# Cleanup function
STACK_PID=""
cleanup() {
    log_info "Cleaning up..."

    # Kill stack process group if running
    if [[ -n "$STACK_PID" ]]; then
        # First, try to kill child processes
        pkill -TERM -P "$STACK_PID" 2>/dev/null || true
        sleep 1
        # Kill process group (negative PID)
        kill -TERM -"$STACK_PID" 2>/dev/null || true
        # Also kill by PID directly as fallback
        kill -TERM "$STACK_PID" 2>/dev/null || true
        # Wait with timeout, then force kill if needed
        local wait_count=0
        while kill -0 "$STACK_PID" 2>/dev/null && [[ $wait_count -lt 5 ]]; do
            sleep 1
            wait_count=$((wait_count + 1))
        done
        if kill -0 "$STACK_PID" 2>/dev/null; then
            log_warn "Force killing stack process..."
            kill -9 "$STACK_PID" 2>/dev/null || true
        fi
    fi

    # Kill any remaining ROS/Gazebo processes from this session
    pkill -f "ros2 launch ldos_harness" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true

    # Wait for processes to terminate
    sleep 2

    log_info "Cleanup complete"
}

# Set trap for cleanup on exit
trap cleanup EXIT INT TERM

# Source ROS environment
source_ros() {
    local ros_found=false
    for distro in jazzy humble iron rolling; do
        if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
            source "/opt/ros/${distro}/setup.bash"
            log_info "Using ROS 2 $distro"
            ros_found=true
            break
        fi
    done

    if [[ "$ros_found" = false ]]; then
        log_error "ROS 2 installation not found in /opt/ros/"
        exit 1
    fi

    if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
        source "$WS_ROOT/install/setup.bash"
    else
        log_error "Workspace not built. Run 'make setup' first."
        exit 1
    fi

    # Activate venv if exists
    if [[ -f "$WS_ROOT/.venv/bin/activate" ]]; then
        source "$WS_ROOT/.venv/bin/activate"
    fi
}

# Main test function
run_smoke_test() {
    log_info "=== LDOS Harness Smoke Test ==="
    log_info "Working directory: $WS_ROOT"

    # Source ROS
    log_info "Sourcing ROS environment..."
    source_ros

    # Create output directory
    RESULT_DIR="$WS_ROOT/results/smoke"
    mkdir -p "$RESULT_DIR"

    # Start the stack in a new process group
    log_info "Starting full stack (timeout: ${STACK_TIMEOUT}s)..."
    set -m  # Enable job control for process groups
    timeout "$STACK_TIMEOUT" ros2 launch ldos_harness full_stack.launch.py \
        headless:=true \
        use_rviz:=false \
        2>&1 | tee "$RESULT_DIR/stack.log" &
    STACK_PID=$!
    set +m

    log_info "Stack PID: $STACK_PID"

    # Wait for initialization with progress
    log_info "Waiting for stack initialization (${INIT_WAIT}s)..."
    for i in $(seq 1 $INIT_WAIT); do
        echo -n "."
        sleep 1
    done
    echo ""

    # Check if stack is still running
    if ! kill -0 "$STACK_PID" 2>/dev/null; then
        log_fail "Stack process died during initialization"
        cat "$RESULT_DIR/stack.log" | tail -50
        exit 1
    fi
    log_pass "Stack is running"

    # Check MoveGroup action availability
    log_info "Checking MoveGroup action availability (looking for $MOVEIT_ACTION_NAME)..."
    ACTION_CHECK=$(ros2 action list 2>/dev/null || echo "")
    if echo "$ACTION_CHECK" | grep -q "$MOVEIT_ACTION_NAME"; then
        log_pass "MoveGroup action available: $MOVEIT_ACTION_NAME"
    else
        log_fail "MoveGroup action '$MOVEIT_ACTION_NAME' not found"
        log_info "Available actions:"
        echo "$ACTION_CHECK"
        exit 1
    fi

    # Run benchmark
    log_info "Running benchmark trial (timeout: ${BENCHMARK_TIMEOUT}s)..."
    TRIAL_ID="smoke_test_$(date +%s)"

    set +e  # Don't exit on benchmark failure
    timeout "$BENCHMARK_TIMEOUT" ros2 run ldos_harness benchmark_runner.py \
        --trial-id "$TRIAL_ID" \
        --scenario smoke \
        --output-dir "$RESULT_DIR" \
        --timeout 25.0 \
        --action-name "$MOVEIT_ACTION_NAME" \
        2>&1 | tee "$RESULT_DIR/benchmark.log"
    BENCH_EXIT=$?
    set -e

    if [[ $BENCH_EXIT -eq 0 ]]; then
        log_pass "Benchmark completed successfully"
    else
        log_warn "Benchmark exited with code: $BENCH_EXIT"
    fi

    # Check result file
    RESULT_FILE="$RESULT_DIR/${TRIAL_ID}_result.json"
    if [[ -f "$RESULT_FILE" ]]; then
        log_pass "Result file created: $RESULT_FILE"

        # Parse and display status
        if command -v python3 &>/dev/null; then
            STATUS=$(python3 -c "import json; d=json.load(open('$RESULT_FILE')); print(f'Status: {d.get(\"status\", \"unknown\")}')" 2>/dev/null || echo "Status: parse error")
            log_info "$STATUS"
        fi
    else
        log_fail "No result file created"
        log_info "Expected: $RESULT_FILE"
        log_info "Contents of $RESULT_DIR:"
        ls -la "$RESULT_DIR" || true
        exit 1
    fi

    # Final summary
    echo ""
    log_info "=== Smoke Test Complete ==="
    log_pass "All checks passed!"

    return 0
}

# Run the test
run_smoke_test

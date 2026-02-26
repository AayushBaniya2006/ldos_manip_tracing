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
MOVEIT_ACTION_NAME="${MOVEIT_ACTION_NAME:-/move_action}"
# Startup timeout for polling readiness (action + controller active)
STACK_TIMEOUT="${STACK_TIMEOUT:-90}"
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
    # Disable -u temporarily - ROS setup scripts use unset variables
    set +u

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
        set -u
        log_error "ROS 2 installation not found in /opt/ros/"
        exit 1
    fi

    if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
        source "$WS_ROOT/install/setup.bash"
    else
        set -u
        log_error "Workspace not built. Run 'make setup' first."
        exit 1
    fi

    # Activate venv if exists
    if [[ -f "$WS_ROOT/.venv/bin/activate" ]]; then
        source "$WS_ROOT/.venv/bin/activate"
    fi

    # Re-enable -u
    set -u
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

    # Start the stack (capture the actual ros2 launch PID, no timeout wrapper)
    log_info "Starting full stack (timeout: ${STACK_TIMEOUT}s)..."
    ros2 launch ldos_harness full_stack.launch.py \
        headless:=true \
        use_rviz:=false \
        > "$RESULT_DIR/stack.log" 2>&1 &
    STACK_PID=$!

    log_info "Stack PID: $STACK_PID"

    # Poll readiness (MoveGroup action + active arm controller)
    log_info "Waiting for stack readiness (polling, max ${STACK_TIMEOUT}s)..."
    READY=false
    for ((elapsed=0; elapsed<STACK_TIMEOUT; elapsed+=2)); do
        if ! kill -0 "$STACK_PID" 2>/dev/null; then
            log_fail "Stack process died during initialization"
            tail -50 "$RESULT_DIR/stack.log" || true
            exit 1
        fi
        if ros2 action list 2>/dev/null | grep -q "$MOVEIT_ACTION_NAME"; then
            if ros2 control list_controllers 2>/dev/null | grep -q "panda_arm_controller.*active"; then
                READY=true
                log_pass "Stack ready after ~${elapsed}s (action + controller active)"
                break
            fi
        fi
        echo -n "."
        sleep 2
    done
    echo ""

    if [[ "$READY" != true ]]; then
        log_fail "Stack not ready within ${STACK_TIMEOUT}s"
        tail -50 "$RESULT_DIR/stack.log" || true
        exit 1
    fi

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

    # Check result file and validate status
    RESULT_FILE="$RESULT_DIR/${TRIAL_ID}_result.json"
    if [[ -f "$RESULT_FILE" ]]; then
        log_pass "Result file created: $RESULT_FILE"

        # Parse and validate benchmark status
        BENCH_STATUS="unknown"
        if command -v python3 &>/dev/null; then
            BENCH_STATUS=$(python3 -c "import json; d=json.load(open('$RESULT_FILE')); print(d.get('status', 'unknown'))" 2>/dev/null || echo "parse_error")
            log_info "Benchmark status: $BENCH_STATUS"
        fi

        if [[ "$BENCH_STATUS" != "success" ]]; then
            log_fail "Benchmark did not succeed (status: $BENCH_STATUS)"
            # Show error details from result file
            if command -v python3 &>/dev/null; then
                python3 -c "
import json
d = json.load(open('$RESULT_FILE'))
err = d.get('error_message', '')
code = d.get('moveit_error_code', '')
label = d.get('moveit_error_label', '')
if err: print(f'  Error: {err}')
if code: print(f'  MoveIt code: {code} ({label})')
" 2>/dev/null || true
            fi
            exit 1
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
    log_pass "All checks passed (benchmark status: success)"

    return 0
}

# Run the test
run_smoke_test

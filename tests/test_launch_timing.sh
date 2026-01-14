#!/bin/bash
# test_launch_timing.sh - Test stack comes up reliably under load
#
# This test verifies that the ROS stack starts correctly even under CPU stress,
# testing the increased MoveIt delay (R1 fix).

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_pass() { echo -e "${GREEN}[PASS]${NC} $1"; }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; }
log_info() { echo -e "${YELLOW}[INFO]${NC} $1"; }

# Source ROS
set +u
for distro in jazzy humble iron rolling; do
    if [ -f "/opt/ros/${distro}/setup.bash" ]; then
        source "/opt/ros/${distro}/setup.bash"
        break
    fi
done
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
fi
set -u

# Check if stress-ng is available
if ! command -v stress-ng &>/dev/null; then
    log_info "stress-ng not found. Running without CPU load."
    USE_STRESS=false
else
    USE_STRESS=true
fi

ITERATIONS=${1:-3}
PASS=0
FAIL=0

log_info "Running $ITERATIONS launch timing tests..."

for i in $(seq 1 $ITERATIONS); do
    echo ""
    log_info "=== Test $i/$ITERATIONS ==="

    STRESS_PID=""
    LAUNCH_PID=""

    # Start with CPU load if stress-ng available
    if [ "$USE_STRESS" = true ]; then
        log_info "Starting CPU stress (80% on 4 cores)..."
        stress-ng --cpu 4 --cpu-load 80 --timeout 120 &
        STRESS_PID=$!
        sleep 5
    fi

    # Launch stack
    log_info "Launching full stack..."
    timeout 90 ros2 launch ldos_harness full_stack.launch.py headless:=true use_rviz:=false &
    LAUNCH_PID=$!

    # Wait for action server with polling
    READY=false
    for j in $(seq 1 45); do
        if ros2 action list 2>/dev/null | grep -q move_action; then
            READY=true
            log_info "Stack ready after ~$((j * 2))s"
            break
        fi

        # Check if launch process died
        if ! kill -0 $LAUNCH_PID 2>/dev/null; then
            log_info "Launch process exited early"
            break
        fi

        sleep 2
    done

    # Cleanup
    if [ -n "$LAUNCH_PID" ]; then
        kill $LAUNCH_PID 2>/dev/null || true
        wait $LAUNCH_PID 2>/dev/null || true
    fi
    if [ -n "$STRESS_PID" ]; then
        kill $STRESS_PID 2>/dev/null || true
        wait $STRESS_PID 2>/dev/null || true
    fi

    # Kill any leftover Gazebo processes
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f ruby 2>/dev/null || true  # Gazebo uses Ruby
    sleep 5

    if [ "$READY" = true ]; then
        log_pass "Stack ready in iteration $i"
        PASS=$((PASS + 1))
    else
        log_fail "Stack not ready in iteration $i"
        FAIL=$((FAIL + 1))
    fi
done

echo ""
echo "=========================================="
echo "  Launch Timing Test Results"
echo "=========================================="
echo "Pass: $PASS/$ITERATIONS"
echo "Fail: $FAIL/$ITERATIONS"

[ $FAIL -eq 0 ] && exit 0 || exit 1

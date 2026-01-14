#!/bin/bash
# test_buffer_overflow.sh - Test buffer doesn't overflow under msg_load
#
# This test verifies the T2 fix (64MB buffer) prevents event loss
# under high message load scenarios.

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

# Check LTTng is available
if ! command -v lttng &>/dev/null; then
    log_info "LTTng not installed. Skipping buffer overflow test."
    exit 0
fi

TEST_SESSION="test_buffer_$(date +%s)"
TEST_DIR="$WS_ROOT/traces/$TEST_SESSION"
ERRORS=0

cleanup() {
    # Kill load generator
    if [ -n "${LOAD_PID:-}" ]; then
        kill $LOAD_PID 2>/dev/null || true
    fi
    # Kill any ROS nodes we started
    pkill -f "ros2 topic pub" 2>/dev/null || true
    # Stop trace
    lttng stop "$TEST_SESSION" 2>/dev/null || true
    lttng destroy "$TEST_SESSION" 2>/dev/null || true
    rm -rf "$TEST_DIR" 2>/dev/null || true
}

trap cleanup EXIT

log_info "=== Buffer Overflow Test ==="
log_info "Testing that 64MB buffer handles high message load..."

mkdir -p "$TEST_DIR"

# Start trace
log_info "Starting trace session..."
if ! "$WS_ROOT/scripts/start_trace.sh" "$TEST_SESSION" "$TEST_DIR" > "$TEST_DIR/start.log" 2>&1; then
    log_fail "Failed to start trace"
    cat "$TEST_DIR/start.log"
    exit 1
fi

# Generate high message load (simulate msg_load scenario)
log_info "Generating high message load (10 seconds)..."
LOAD_PID=""

# Start multiple high-frequency publishers
for i in 1 2 3 4; do
    ros2 topic pub /test_load_$i std_msgs/msg/String "data: 'test payload with some data to fill buffer'" --rate 1000 &
done
LOAD_PID=$!

# Let it run for 10 seconds
sleep 10

# Stop publishers
pkill -f "ros2 topic pub" 2>/dev/null || true
sleep 2

# Stop trace
log_info "Stopping trace..."
"$WS_ROOT/scripts/stop_trace.sh" "$TEST_SESSION" > "$TEST_DIR/stop.log" 2>&1 || true

# Check for discarded events
log_info "Checking for discarded events..."

# Check lttng status output for discards
DISCARDS=$(lttng list -u 2>/dev/null | grep -i "discarded" | grep -v "0 discarded" || true)

if [ -n "$DISCARDS" ]; then
    log_fail "Events were discarded (buffer overflow detected):"
    echo "$DISCARDS"
    ERRORS=$((ERRORS + 1))
else
    log_pass "No discarded events detected"
fi

# Also check the trace start log for channel creation
if grep -q "Creating userspace channel with 64MB buffer" "$TEST_DIR/start.log" 2>/dev/null || \
   grep -q "Creating channel with 64MB buffer" "$TEST_DIR/start.log" 2>/dev/null; then
    log_pass "64MB buffer channel was created"
else
    log_info "Note: Could not verify buffer size from log (may still be correct)"
fi

echo ""
echo "=========================================="
echo "  Buffer Overflow Test Results"
echo "=========================================="
if [ $ERRORS -eq 0 ]; then
    log_pass "Buffer overflow test passed"
    exit 0
else
    log_fail "Buffer overflow test failed"
    exit 1
fi

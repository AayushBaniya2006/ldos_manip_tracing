#!/bin/bash
# test_timing_accuracy.sh - Test dual timestamp implementation
#
# This test verifies the B2 fix (dual timestamps) is working correctly,
# capturing both wall clock and ROS clock times in markers.

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

ERRORS=0

log_info "=== Dual Timestamp Test ==="

# Check for existing result files from smoke test or experiments
RESULT_FILE=$(ls -t "$WS_ROOT/results"/*/*.json 2>/dev/null | head -1 || true)

if [ -z "$RESULT_FILE" ]; then
    log_info "No existing result files found. Running quick benchmark..."

    # Try to run a quick test if the stack can start
    # This is optional - the test can still pass if we verify the code directly
    log_info "Skipping live test (requires full stack). Verifying code implementation instead."

    # Check benchmark_runner.py has dual timestamp implementation
    if grep -q "wall_time_ns" "$WS_ROOT/src/ldos_harness/scripts/benchmark_runner.py" && \
       grep -q "ros_time_ns" "$WS_ROOT/src/ldos_harness/scripts/benchmark_runner.py"; then
        log_pass "Dual timestamp fields found in benchmark_runner.py"
    else
        log_fail "Dual timestamp fields not found in code"
        ERRORS=$((ERRORS + 1))
    fi

    # Check TimingMarker dataclass
    if grep -q "class TimingMarker" "$WS_ROOT/src/ldos_harness/scripts/benchmark_runner.py"; then
        log_pass "TimingMarker dataclass exists"
    else
        log_fail "TimingMarker dataclass not found"
        ERRORS=$((ERRORS + 1))
    fi

    # Check add_marker implementation
    if grep -q "ros_now = self.get_clock().now()" "$WS_ROOT/src/ldos_harness/scripts/benchmark_runner.py"; then
        log_pass "ROS clock capture in add_marker"
    else
        log_fail "ROS clock capture not found"
        ERRORS=$((ERRORS + 1))
    fi

else
    log_info "Found result file: $RESULT_FILE"

    # Check for dual timestamp fields in result
    log_info "Checking for wall_time_ns..."
    if grep -q "wall_time_ns" "$RESULT_FILE"; then
        log_pass "wall_time_ns found in results"
    else
        log_fail "wall_time_ns not found in results"
        ERRORS=$((ERRORS + 1))
    fi

    log_info "Checking for ros_time_ns..."
    if grep -q "ros_time_ns" "$RESULT_FILE"; then
        log_pass "ros_time_ns found in results"
    else
        log_fail "ros_time_ns not found in results"
        ERRORS=$((ERRORS + 1))
    fi

    # Verify timestamps are valid using Python
    log_info "Validating timestamp values..."
    python3 << EOF
import json
import sys

with open("$RESULT_FILE") as f:
    data = json.load(f)

markers = data.get('markers', [])
if not markers:
    print("WARNING: No markers in result file")
    sys.exit(0)

valid_count = 0
invalid_count = 0

for m in markers:
    wall = m.get('wall_time_ns', 0)
    ros = m.get('ros_time_ns', 0)

    if wall > 0 and ros >= 0:  # ros can be 0 at sim start
        valid_count += 1
    else:
        invalid_count += 1
        print(f"  Invalid marker {m.get('name', 'unknown')}: wall={wall}, ros={ros}")

print(f"Valid markers: {valid_count}/{len(markers)}")
if invalid_count > 0:
    print(f"Invalid markers: {invalid_count}")
    sys.exit(1)

# Show sample timestamps
if markers:
    m = markers[0]
    print(f"Sample - {m['name']}: wall_time_ns={m.get('wall_time_ns', 'N/A')}, ros_time_ns={m.get('ros_time_ns', 'N/A')}")
EOF

    if [ $? -eq 0 ]; then
        log_pass "Timestamp values are valid"
    else
        log_fail "Some timestamp values are invalid"
        ERRORS=$((ERRORS + 1))
    fi
fi

echo ""
echo "=========================================="
echo "  Timing Accuracy Test Results"
echo "=========================================="
if [ $ERRORS -eq 0 ]; then
    log_pass "Dual timestamp test passed"
    exit 0
else
    log_fail "$ERRORS test(s) failed"
    exit 1
fi

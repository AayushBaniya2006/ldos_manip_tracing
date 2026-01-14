#!/bin/bash
# run_all_tests.sh - Master test runner for LDOS harness regression tests
#
# Runs all test scripts in the tests/ directory and reports results.
# Use this to verify all fixes are working correctly.
#
# Usage:
#   ./tests/run_all_tests.sh           # Run all tests
#   ./tests/run_all_tests.sh quick     # Run quick tests only (skip launch timing)
#   ./tests/run_all_tests.sh --help    # Show help

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_pass() { echo -e "${GREEN}[PASS]${NC} $1"; }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; }
log_info() { echo -e "${YELLOW}[INFO]${NC} $1"; }
log_header() { echo -e "\n${CYAN}$1${NC}"; }

# Help
if [[ "${1:-}" == "--help" ]] || [[ "${1:-}" == "-h" ]]; then
    echo "LDOS Harness Regression Test Runner"
    echo ""
    echo "Usage:"
    echo "  $0              Run all tests"
    echo "  $0 quick        Run quick tests (skip launch_timing)"
    echo "  $0 --help       Show this help"
    echo ""
    echo "Available tests:"
    for test in "$SCRIPT_DIR"/test_*.sh; do
        if [ -f "$test" ]; then
            echo "  - $(basename "$test")"
        fi
    done
    exit 0
fi

MODE="${1:-full}"
PASS=0
FAIL=0
SKIP=0
RESULTS=()

echo ""
echo "=========================================="
echo "  LDOS Harness Regression Test Suite"
echo "=========================================="
echo ""
echo "Mode: $MODE"
echo "Time: $(date)"
echo ""

# Make all test scripts executable
chmod +x "$SCRIPT_DIR"/test_*.sh 2>/dev/null || true

# Run tests
for test in "$SCRIPT_DIR"/test_*.sh; do
    if [ ! -f "$test" ]; then
        continue
    fi

    TEST_NAME=$(basename "$test" .sh)

    # Skip launch_timing in quick mode (it's slow)
    if [[ "$MODE" == "quick" ]] && [[ "$TEST_NAME" == "test_launch_timing" ]]; then
        log_info "Skipping $TEST_NAME (quick mode)"
        SKIP=$((SKIP + 1))
        RESULTS+=("SKIP: $TEST_NAME")
        continue
    fi

    log_header ">>> Running $TEST_NAME..."
    echo ""

    START_TIME=$(date +%s)

    if "$test"; then
        END_TIME=$(date +%s)
        DURATION=$((END_TIME - START_TIME))
        log_pass "$TEST_NAME completed in ${DURATION}s"
        PASS=$((PASS + 1))
        RESULTS+=("PASS: $TEST_NAME (${DURATION}s)")
    else
        END_TIME=$(date +%s)
        DURATION=$((END_TIME - START_TIME))
        log_fail "$TEST_NAME failed after ${DURATION}s"
        FAIL=$((FAIL + 1))
        RESULTS+=("FAIL: $TEST_NAME (${DURATION}s)")
    fi

    echo ""
done

# Summary
echo ""
echo "=========================================="
echo "  Test Results Summary"
echo "=========================================="
echo ""
for result in "${RESULTS[@]}"; do
    if [[ "$result" == PASS* ]]; then
        echo -e "${GREEN}$result${NC}"
    elif [[ "$result" == FAIL* ]]; then
        echo -e "${RED}$result${NC}"
    else
        echo -e "${YELLOW}$result${NC}"
    fi
done
echo ""
echo "=========================================="
TOTAL=$((PASS + FAIL))
echo "  Total:   $TOTAL tests"
echo "  Passed:  $PASS"
echo "  Failed:  $FAIL"
echo "  Skipped: $SKIP"
echo "=========================================="

if [ $FAIL -eq 0 ]; then
    echo ""
    log_pass "All tests passed!"
    exit 0
else
    echo ""
    log_fail "$FAIL test(s) failed"
    exit 1
fi

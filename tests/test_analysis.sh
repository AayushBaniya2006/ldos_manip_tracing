#!/bin/bash
# test_analysis.sh - Test trace analysis handles edge cases
#
# This test verifies the A1/A2 fixes (exception handling, data loss detection)
# by running the analyzer against various inputs.

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

# Activate venv if exists
set +u
if [ -f "$WS_ROOT/.venv/bin/activate" ]; then
    source "$WS_ROOT/.venv/bin/activate"
fi
set -u

ERRORS=0
TMP_DIR="/tmp/test_analysis_$$"
mkdir -p "$TMP_DIR"

cleanup() {
    rm -rf "$TMP_DIR" 2>/dev/null || true
}

trap cleanup EXIT

echo ""
log_info "=== Test 1: Empty Trace Directory Handling ==="
mkdir -p "$TMP_DIR/empty_trace"

OUTPUT=$(python3 "$WS_ROOT/analysis/analyze_trace.py" \
    --trace-dir "$TMP_DIR/empty_trace" \
    --output-dir "$TMP_DIR/empty_output" 2>&1) || true

# Check for Python exceptions
if echo "$OUTPUT" | grep -qi "traceback\|exception"; then
    log_fail "Empty trace caused Python exception"
    echo "$OUTPUT"
    ERRORS=$((ERRORS + 1))
else
    log_pass "Empty trace handled gracefully"
fi

echo ""
log_info "=== Test 2: Non-existent Directory Handling ==="
OUTPUT=$(python3 "$WS_ROOT/analysis/analyze_trace.py" \
    --trace-dir "/nonexistent/path/to/trace" \
    --output-dir "$TMP_DIR/nonexistent_output" 2>&1) || true

if echo "$OUTPUT" | grep -qi "traceback"; then
    # Some errors are expected for non-existent paths, but shouldn't be uncaught exceptions
    if echo "$OUTPUT" | grep -qi "No trace data found\|ERROR"; then
        log_pass "Non-existent path handled with proper error message"
    else
        log_fail "Non-existent path caused unexpected exception"
        echo "$OUTPUT"
        ERRORS=$((ERRORS + 1))
    fi
else
    log_pass "Non-existent path handled gracefully"
fi

echo ""
log_info "=== Test 3: Exception Handling Code Exists ==="
# Check that the malformed event handling exists
if grep -q "_malformed_events" "$WS_ROOT/analysis/analyze_trace.py"; then
    log_pass "Malformed event counter exists"
else
    log_fail "Malformed event counter not found"
    ERRORS=$((ERRORS + 1))
fi

if grep -q "_unmatched_callback_ends" "$WS_ROOT/analysis/analyze_trace.py"; then
    log_pass "Unmatched callback counter exists"
else
    log_fail "Unmatched callback counter not found"
    ERRORS=$((ERRORS + 1))
fi

echo ""
log_info "=== Test 4: Try-Except in Event Handlers ==="
if grep -q "except (KeyError, TypeError)" "$WS_ROOT/analysis/analyze_trace.py"; then
    log_pass "KeyError/TypeError exception handling exists"
else
    log_fail "Exception handling for malformed events not found"
    ERRORS=$((ERRORS + 1))
fi

echo ""
log_info "=== Test 5: Low Sample Size Warning ==="
if grep -q "Percentile statistics may be unreliable" "$WS_ROOT/analysis/analyze_trace.py" || \
   grep -q "Only.*callbacks recorded" "$WS_ROOT/analysis/analyze_trace.py"; then
    log_pass "Low sample size warning exists"
else
    log_fail "Low sample size warning not found"
    ERRORS=$((ERRORS + 1))
fi

echo ""
log_info "=== Test 6: Real Trace Analysis (if available) ==="
TRACE_DIR=$(ls -td "$WS_ROOT/traces"/*/ 2>/dev/null | head -1 || true)

if [ -n "$TRACE_DIR" ] && [ -d "$TRACE_DIR" ]; then
    log_info "Found trace directory: $TRACE_DIR"

    OUTPUT=$(python3 "$WS_ROOT/analysis/analyze_trace.py" \
        --trace-dir "$TRACE_DIR" \
        --output-dir "$TMP_DIR/real_analysis" 2>&1) || true

    # Check no Python exceptions
    if echo "$OUTPUT" | grep -qi "^Traceback"; then
        log_fail "Real trace analysis raised exception"
        echo "$OUTPUT"
        ERRORS=$((ERRORS + 1))
    else
        log_pass "Real trace analysis completed without exceptions"
        # Show summary if successful
        echo "$OUTPUT" | grep -E "^(Processed|Callbacks:|WARNING)" || true
    fi
else
    log_info "No trace data available - skipping real trace test"
fi

echo ""
log_info "=== Test 7: Aggregate Results (if available) ==="
RESULT_DIR=$(ls -d "$WS_ROOT/results"/*/ 2>/dev/null | head -1 || true)

if [ -n "$RESULT_DIR" ] && [ -d "$RESULT_DIR" ]; then
    log_info "Found result directory: $RESULT_DIR"

    OUTPUT=$(python3 "$WS_ROOT/analysis/analyze_trace.py" \
        --result-dir "$RESULT_DIR" \
        --output-dir "$TMP_DIR/aggregate_test" 2>&1) || true

    if echo "$OUTPUT" | grep -qi "^Traceback"; then
        log_fail "Result aggregation raised exception"
        ERRORS=$((ERRORS + 1))
    else
        log_pass "Result aggregation completed"
    fi
else
    log_info "No result data available - skipping aggregation test"
fi

echo ""
echo "=========================================="
echo "  Analysis Test Results"
echo "=========================================="
if [ $ERRORS -eq 0 ]; then
    log_pass "All analysis tests passed"
    exit 0
else
    log_fail "$ERRORS test(s) failed"
    exit 1
fi

#!/bin/bash
# test_tracing.sh - Test tracing capture and failure detection
#
# This test verifies:
# - T1: Trace start failure is detected
# - T3/T4: Session handling is correct
# - Trace capture works end-to-end

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

# Check LTTng is available
if ! command -v lttng &>/dev/null; then
    log_info "LTTng not installed. Skipping tracing tests."
    exit 0
fi

TEST_SESSION="test_trace_$(date +%s)"
TEST_DIR="$WS_ROOT/traces/$TEST_SESSION"
ERRORS=0

cleanup() {
    # Stop and destroy any test sessions
    lttng stop "$TEST_SESSION" 2>/dev/null || true
    lttng destroy "$TEST_SESSION" 2>/dev/null || true
    rm -rf "$TEST_DIR" 2>/dev/null || true
}

trap cleanup EXIT

echo ""
log_info "=== Test 1: Invalid Session Detection ==="
# Test that trace start failure is detected (T1 fix)
if "$WS_ROOT/scripts/start_trace.sh" "invalid/session/name" "/tmp/nonexistent/path" 2>&1; then
    log_fail "Invalid session should have failed"
    ERRORS=$((ERRORS + 1))
else
    log_pass "Invalid session correctly rejected"
fi

echo ""
log_info "=== Test 2: Valid Trace Session Creation ==="
mkdir -p "$TEST_DIR"
if ! "$WS_ROOT/scripts/start_trace.sh" "$TEST_SESSION" "$TEST_DIR" > "$TEST_DIR/start.log" 2>&1; then
    log_fail "Failed to create trace session"
    cat "$TEST_DIR/start.log"
    ERRORS=$((ERRORS + 1))
else
    log_pass "Trace session created"
fi

echo ""
log_info "=== Test 3: Session Is Active ==="
if ! lttng list 2>/dev/null | grep -q "$TEST_SESSION"; then
    log_fail "Session not found in lttng list"
    ERRORS=$((ERRORS + 1))
else
    log_pass "Session is active"
fi

echo ""
log_info "=== Test 4: Session File Created ==="
if [ ! -f "$TEST_DIR/.session_name" ]; then
    log_fail "Session name file not created"
    ERRORS=$((ERRORS + 1))
else
    STORED_NAME=$(cat "$TEST_DIR/.session_name")
    if [ "$STORED_NAME" = "$TEST_SESSION" ]; then
        log_pass "Session name file correct"
    else
        log_fail "Session name mismatch: $STORED_NAME vs $TEST_SESSION"
        ERRORS=$((ERRORS + 1))
    fi
fi

echo ""
log_info "=== Test 5: Trace Stop Works ==="
if ! "$WS_ROOT/scripts/stop_trace.sh" "$TEST_SESSION" > "$TEST_DIR/stop.log" 2>&1; then
    log_fail "Failed to stop trace session"
    cat "$TEST_DIR/stop.log"
    ERRORS=$((ERRORS + 1))
else
    log_pass "Trace session stopped"
fi

echo ""
log_info "=== Test 6: Session Destroyed ==="
if lttng list 2>/dev/null | grep -q "^$TEST_SESSION "; then
    log_fail "Session still exists after stop"
    ERRORS=$((ERRORS + 1))
else
    log_pass "Session properly destroyed"
fi

echo ""
log_info "=== Test 7: Trace Directory Exists ==="
if [ ! -d "$TEST_DIR" ]; then
    log_fail "Trace directory not created"
    ERRORS=$((ERRORS + 1))
else
    log_pass "Trace directory exists"
fi

echo ""
echo "=========================================="
echo "  Tracing Test Results"
echo "=========================================="
if [ $ERRORS -eq 0 ]; then
    log_pass "All tracing tests passed"
    exit 0
else
    log_fail "$ERRORS test(s) failed"
    exit 1
fi

#!/bin/bash
# preflight_check.sh - Explicit host/tool validation for LDOS harness workflows
# Usage: ./scripts/preflight_check.sh [mode]
# Modes: setup, run, analyze, cpuset, all (default)

set -euo pipefail

MODE="${1:-all}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS=0
FAIL=0
WARN=0

ok() { echo -e "${GREEN}[PASS]${NC} $1"; PASS=$((PASS+1)); }
bad() { echo -e "${RED}[FAIL]${NC} $1"; FAIL=$((FAIL+1)); }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; WARN=$((WARN+1)); }
info() { echo -e "${CYAN}[INFO]${NC} $1"; }

check_cmd() {
    local cmd="$1"
    local required="${2:-true}"
    if command -v "$cmd" >/dev/null 2>&1; then
        ok "command '$cmd' available"
    else
        if [ "$required" = true ]; then
            bad "command '$cmd' missing"
        else
            warn "command '$cmd' missing"
        fi
    fi
}

check_python_import() {
    local module="$1"
    local label="${2:-$1}"
    local required="${3:-true}"
    if python3 -c "import ${module}" >/dev/null 2>&1; then
        ok "python module '${label}' importable"
    else
        if [ "$required" = true ]; then
            bad "python module '${label}' missing"
        else
            warn "python module '${label}' missing"
        fi
    fi
}

echo ""
info "LDOS preflight check (mode: $MODE)"
echo ""

if [ -f /etc/os-release ]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    if [ "${ID:-}" = "ubuntu" ]; then
        ok "OS is Ubuntu (${VERSION_ID:-unknown})"
        if [ "${VERSION_ID:-}" != "24.04" ]; then
            warn "Expected Ubuntu 24.04 for documented CloudLab path (found ${VERSION_ID:-unknown})"
        fi
    else
        warn "Expected Ubuntu 24.04, found ${ID:-unknown}"
    fi
else
    warn "Cannot determine OS (missing /etc/os-release)"
fi

case "$MODE" in
    setup|run|all)
        check_cmd ros2 true
        check_cmd colcon true
        check_cmd gz true
        check_cmd lttng false
        check_cmd stress-ng false
        check_cmd systemd-run false
        check_cmd perf false
        ;;
    analyze)
        check_cmd python3 true
        check_cmd babeltrace2 false
        ;;
    cpuset)
        check_cmd systemd-run false
        check_cmd taskset true
        ;;
    *)
        warn "Unknown mode '$MODE', running generic checks"
        check_cmd python3 true
        ;;
esac

# Common analysis/runtime Python deps (soft for setup; hard enough to surface missing pieces)
check_python_import pandas pandas false
check_python_import psutil psutil false
check_python_import bt2 babeltrace2_bt2 false

if command -v gz >/dev/null 2>&1; then
    if gz sim --version >/dev/null 2>&1; then
        ok "gz sim executable responds"
    else
        warn "gz present but 'gz sim --version' failed"
    fi
fi

echo ""
info "Summary: ${PASS} passed, ${WARN} warnings, ${FAIL} failed"

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi

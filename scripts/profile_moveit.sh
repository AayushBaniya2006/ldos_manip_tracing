#!/bin/bash
# profile_moveit.sh - Profile MoveIt move_group process specifically
#
# This script captures CPU profile data specifically for the MoveIt move_group
# process, generating flamegraphs that show planning and execution hotspots.
#
# Usage: ./profile_moveit.sh [duration_seconds] [output_name]
#
# Prerequisites:
#   - FlameGraph tools: git clone https://github.com/brendangregg/FlameGraph.git ~/FlameGraph
#   - Linux perf: sudo apt install linux-tools-common linux-tools-generic linux-tools-$(uname -r)
#   - MoveIt stack must be running (ros2 launch ldos_harness full_stack.launch.py)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
FLAMEGRAPH_DIR="${FLAMEGRAPH_DIR:-${HOME}/FlameGraph}"

DURATION="${1:-30}"
OUTPUT_NAME="${2:-moveit_profile}"
OUTPUT_DIR="${WS_ROOT}/analysis/output/profiles"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[MOVEIT_PROFILE]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check prerequisites
check_prerequisites() {
    log_info "Checking prerequisites..."

    # Check perf
    if ! command -v perf &>/dev/null; then
        log_error "perf not found. Install with: sudo apt install linux-tools-common linux-tools-generic linux-tools-\$(uname -r)"
        exit 1
    fi

    # Check FlameGraph tools
    if [ ! -d "$FLAMEGRAPH_DIR" ]; then
        log_warn "FlameGraph tools not found at $FLAMEGRAPH_DIR"
        log_info "Installing FlameGraph tools..."
        git clone https://github.com/brendangregg/FlameGraph.git "$FLAMEGRAPH_DIR"
    fi

    if [ ! -f "${FLAMEGRAPH_DIR}/stackcollapse-perf.pl" ]; then
        log_error "FlameGraph stackcollapse-perf.pl not found"
        exit 1
    fi
}

# Find MoveIt processes
find_moveit_pids() {
    local pids=""

    # Look for move_group node
    local move_group_pid=$(pgrep -f "move_group" 2>/dev/null | head -1)
    if [ -n "$move_group_pid" ]; then
        pids="$move_group_pid"
        log_info "Found move_group (PID: $move_group_pid)"
    fi

    # Look for planning scene monitor
    local psm_pid=$(pgrep -f "planning_scene_monitor" 2>/dev/null | head -1)
    if [ -n "$psm_pid" ]; then
        [ -n "$pids" ] && pids="$pids,$psm_pid" || pids="$psm_pid"
        log_info "Found planning_scene_monitor (PID: $psm_pid)"
    fi

    # Look for controller manager
    local cm_pid=$(pgrep -f "controller_manager" 2>/dev/null | head -1)
    if [ -n "$cm_pid" ]; then
        [ -n "$pids" ] && pids="$pids,$cm_pid" || pids="$cm_pid"
        log_info "Found controller_manager (PID: $cm_pid)"
    fi

    echo "$pids"
}

# Main profiling function
run_profile() {
    mkdir -p "$OUTPUT_DIR"

    # Find MoveIt PIDs
    log_info "Searching for MoveIt processes..."
    MOVEIT_PIDS=$(find_moveit_pids)

    if [ -z "$MOVEIT_PIDS" ]; then
        log_error "No MoveIt processes found. Is the stack running?"
        log_info "Start the stack first: ros2 launch ldos_harness full_stack.launch.py"
        exit 1
    fi

    local PERF_DATA="${OUTPUT_DIR}/${OUTPUT_NAME}.perf.data"
    local FLAMEGRAPH="${OUTPUT_DIR}/${OUTPUT_NAME}_flamegraph.svg"
    local FLAMEGRAPH_REV="${OUTPUT_DIR}/${OUTPUT_NAME}_flamegraph_reverse.svg"
    local REPORT="${OUTPUT_DIR}/${OUTPUT_NAME}_report.txt"

    log_info "Profiling MoveIt processes for ${DURATION}s..."
    log_info "PIDs: $MOVEIT_PIDS"
    log_info "Output directory: ${OUTPUT_DIR}"

    # Record with perf targeting specific PIDs
    # -F 99: Sample at 99 Hz
    # -p: Target specific PIDs
    # -g: Enable call-graph recording
    # --call-graph dwarf: Use DWARF for accurate C++ stack traces
    sudo perf record -F 99 -p "$MOVEIT_PIDS" -g --call-graph dwarf \
        -o "$PERF_DATA" -- sleep "$DURATION"

    log_info "Generating flamegraph..."

    # Generate standard flamegraph (caller-oriented)
    sudo perf script -i "$PERF_DATA" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/stackcollapse-perf.pl" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/flamegraph.pl" \
            --title "MoveIt CPU Profile - ${OUTPUT_NAME}" \
            --subtitle "Duration: ${DURATION}s | PIDs: ${MOVEIT_PIDS}" \
            --width 1800 \
            --colors java \
        > "$FLAMEGRAPH"

    # Generate reverse flamegraph (callee-oriented - shows most called functions)
    sudo perf script -i "$PERF_DATA" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/stackcollapse-perf.pl" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/flamegraph.pl" --reverse \
            --title "MoveIt CPU Profile (Reverse) - ${OUTPUT_NAME}" \
            --subtitle "Duration: ${DURATION}s - Callee-oriented view" \
            --width 1800 \
            --colors java \
        > "$FLAMEGRAPH_REV"

    # Generate text report with focus on planning functions
    log_info "Generating text report..."
    {
        echo "=== MoveIt CPU Profile Report ==="
        echo "Duration: ${DURATION}s"
        echo "PIDs: ${MOVEIT_PIDS}"
        echo "Generated: $(date -Iseconds)"
        echo ""
        echo "=== Top Functions (by CPU time) ==="
        sudo perf report -i "$PERF_DATA" --stdio --no-children 2>/dev/null | head -100
        echo ""
        echo "=== Planning-related Functions ==="
        sudo perf report -i "$PERF_DATA" --stdio --no-children 2>/dev/null | \
            grep -iE "(ompl|planning|motion|trajectory|ik|kinematics)" | head -50 || true
        echo ""
        echo "=== Controller-related Functions ==="
        sudo perf report -i "$PERF_DATA" --stdio --no-children 2>/dev/null | \
            grep -iE "(controller|joint|hardware|update)" | head -50 || true
    } > "$REPORT"

    # Fix permissions so non-root can read
    sudo chown "$(id -u):$(id -g)" "$PERF_DATA" "$FLAMEGRAPH" "$FLAMEGRAPH_REV" "$REPORT" 2>/dev/null || true

    log_info "MoveIt profiling complete!"
    echo ""
    echo -e "${CYAN}=== Output Files ===${NC}"
    echo "  Flamegraph:      ${FLAMEGRAPH}"
    echo "  Reverse graph:   ${FLAMEGRAPH_REV}"
    echo "  Text report:     ${REPORT}"
    echo "  Raw perf data:   ${PERF_DATA}"
    echo ""
    echo "Open the SVG files in a browser to interactively explore CPU hotspots."
    echo ""
    echo -e "${CYAN}=== Quick Analysis ===${NC}"
    echo "Top 5 functions by CPU time:"
    sudo perf report -i "$PERF_DATA" --stdio --no-children 2>/dev/null | \
        grep -E "^\s+[0-9]" | head -5 || true
}

# Main
main() {
    echo ""
    echo "=============================================="
    echo "  MoveIt CPU Profiler (perf + FlameGraph)"
    echo "=============================================="
    echo ""

    check_prerequisites
    run_profile
}

main "$@"

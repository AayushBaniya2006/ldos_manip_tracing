#!/bin/bash
# cpu_profile.sh - Record CPU profile and generate flamegraph
#
# This script uses Linux perf to record CPU samples and generates
# interactive flamegraph SVGs for visual CPU hotspot analysis.
#
# Usage: ./cpu_profile.sh [duration_seconds] [output_name]
#
# Prerequisites:
#   - FlameGraph tools: git clone https://github.com/brendangregg/FlameGraph.git ~/FlameGraph
#   - Linux perf: sudo apt install linux-tools-common linux-tools-generic linux-tools-$(uname -r)
#   - sudo access for perf record

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
FLAMEGRAPH_DIR="${FLAMEGRAPH_DIR:-${HOME}/FlameGraph}"

DURATION="${1:-30}"
OUTPUT_NAME="${2:-cpu_profile}"
OUTPUT_DIR="${WS_ROOT}/analysis/output/profiles"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[CPU_PROFILE]${NC} $1"; }
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

    # Check sudo access
    if ! sudo -n true 2>/dev/null; then
        log_warn "sudo password may be required for perf record"
    fi
}

# Source ROS environment (function-based for stability)
source_ros() {
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
}

# Main profiling function
run_profile() {
    mkdir -p "$OUTPUT_DIR"

    local PERF_DATA="${OUTPUT_DIR}/${OUTPUT_NAME}.perf.data"
    local FLAMEGRAPH="${OUTPUT_DIR}/${OUTPUT_NAME}_flamegraph.svg"
    local FLAMEGRAPH_REV="${OUTPUT_DIR}/${OUTPUT_NAME}_flamegraph_reverse.svg"
    local REPORT="${OUTPUT_DIR}/${OUTPUT_NAME}_report.txt"

    log_info "Recording CPU profile for ${DURATION}s..."
    log_info "Output directory: ${OUTPUT_DIR}"

    # Record with perf (system-wide, all CPUs, with call graphs)
    # -F 99: Sample at 99 Hz (avoid lockstep with timers)
    # -a: All CPUs
    # -g: Enable call-graph recording
    # --call-graph dwarf: Use DWARF for accurate stack traces
    sudo perf record -F 99 -a -g --call-graph dwarf \
        -o "$PERF_DATA" -- sleep "$DURATION"

    log_info "Generating flamegraph..."

    # Generate standard flamegraph (caller-oriented)
    sudo perf script -i "$PERF_DATA" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/stackcollapse-perf.pl" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/flamegraph.pl" \
            --title "LDOS CPU Profile - ${OUTPUT_NAME}" \
            --subtitle "Duration: ${DURATION}s" \
            --width 1800 \
        > "$FLAMEGRAPH"

    # Generate reverse flamegraph (callee-oriented - shows most called functions)
    sudo perf script -i "$PERF_DATA" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/stackcollapse-perf.pl" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/flamegraph.pl" --reverse \
            --title "LDOS CPU Profile (Reverse) - ${OUTPUT_NAME}" \
            --subtitle "Duration: ${DURATION}s - Callee-oriented view" \
            --width 1800 \
        > "$FLAMEGRAPH_REV"

    # Generate text report
    log_info "Generating text report..."
    sudo perf report -i "$PERF_DATA" --stdio --no-children > "$REPORT" 2>/dev/null || true

    # Fix permissions so non-root can read
    sudo chown "$(id -u):$(id -g)" "$PERF_DATA" "$FLAMEGRAPH" "$FLAMEGRAPH_REV" "$REPORT" 2>/dev/null || true

    log_info "CPU profiling complete!"
    echo ""
    echo "=== Output Files ==="
    echo "  Flamegraph:      ${FLAMEGRAPH}"
    echo "  Reverse graph:   ${FLAMEGRAPH_REV}"
    echo "  Text report:     ${REPORT}"
    echo "  Raw perf data:   ${PERF_DATA}"
    echo ""
    echo "Open the SVG files in a browser to interactively explore CPU hotspots."
    echo "Click on frames to zoom in, click 'Reset Zoom' to zoom out."
}

# Main
main() {
    echo ""
    echo "=============================================="
    echo "  LDOS CPU Profiler (perf + FlameGraph)"
    echo "=============================================="
    echo ""

    check_prerequisites
    source_ros
    run_profile
}

main "$@"

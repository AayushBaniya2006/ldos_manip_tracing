#!/bin/bash
# profile_experiment.sh - Run experiment with integrated CPU profiling
#
# This script runs a full experiment trial while simultaneously capturing
# CPU profile data, allowing correlation between benchmark results and
# CPU hotspots.
#
# Usage: ./profile_experiment.sh [scenario] [profile_duration]
#
# Arguments:
#   scenario          - baseline, cpu_load, or msg_load (default: baseline)
#   profile_duration  - Duration of CPU profiling in seconds (default: 60)
#
# Prerequisites:
#   - FlameGraph tools: git clone https://github.com/brendangregg/FlameGraph.git ~/FlameGraph
#   - Linux perf: sudo apt install linux-tools-common linux-tools-generic linux-tools-$(uname -r)
#   - Workspace built: make setup

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
FLAMEGRAPH_DIR="${FLAMEGRAPH_DIR:-${HOME}/FlameGraph}"

SCENARIO="${1:-baseline}"
PROFILE_DURATION="${2:-60}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
PROFILE_NAME="${SCENARIO}_profiled_${TIMESTAMP}"
MOVEIT_ACTION_NAME="${MOVEIT_ACTION_NAME:-/move_action}"
BENCHMARK_TIMEOUT="${BENCHMARK_TIMEOUT:-60.0}"

# Directories
OUTPUT_DIR="${WS_ROOT}/analysis/output/profiles"
RESULT_DIR="${WS_ROOT}/results/${SCENARIO}"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[PROFILE_EXP]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_section() { echo -e "\n${CYAN}======================================${NC}"; echo -e "${CYAN}=== $1 ===${NC}"; echo -e "${CYAN}======================================${NC}"; }

# Cleanup function
STACK_PID=""
LOAD_PID=""
PROFILE_PID=""
cleanup() {
    log_section "Cleanup"

    # Stop profiling if still running
    if [ -n "$PROFILE_PID" ] && kill -0 "$PROFILE_PID" 2>/dev/null; then
        log_info "Stopping profiler..."
        kill "$PROFILE_PID" 2>/dev/null || true
        wait "$PROFILE_PID" 2>/dev/null || true
    fi

    # Stop load generator
    if [ -n "$LOAD_PID" ] && kill -0 "$LOAD_PID" 2>/dev/null; then
        log_info "Stopping load generator..."
        kill "$LOAD_PID" 2>/dev/null || true
        wait "$LOAD_PID" 2>/dev/null || true
    fi

    # Stop stack
    if [ -n "$STACK_PID" ] && kill -0 "$STACK_PID" 2>/dev/null; then
        log_info "Stopping ROS stack..."
        kill "$STACK_PID" 2>/dev/null || true
        wait "$STACK_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# =============================================================================
# ROS 2 ENVIRONMENT SETUP
# =============================================================================

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
    if [ -f "$WS_ROOT/.venv/bin/activate" ]; then
        source "$WS_ROOT/.venv/bin/activate"
    fi
    set -u
}

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

    # Check workspace
    if [ ! -d "$WS_ROOT/install/ldos_harness" ]; then
        log_error "ldos_harness not built. Run: make setup"
        exit 1
    fi
}

# Start CPU profiling in background
start_profiling() {
    local duration="$1"
    local output_name="$2"
    local perf_data="${OUTPUT_DIR}/${output_name}.perf.data"

    log_info "Starting CPU profiling for ${duration}s..."

    # Run perf record in background
    sudo perf record -F 99 -a -g --call-graph dwarf \
        -o "$perf_data" -- sleep "$duration" &
    PROFILE_PID=$!

    log_info "Profiler started (PID: $PROFILE_PID)"
}

# Generate flamegraphs from perf data
generate_flamegraphs() {
    local output_name="$1"
    local perf_data="${OUTPUT_DIR}/${output_name}.perf.data"
    local flamegraph="${OUTPUT_DIR}/${output_name}_flamegraph.svg"
    local flamegraph_rev="${OUTPUT_DIR}/${output_name}_flamegraph_reverse.svg"
    local report="${OUTPUT_DIR}/${output_name}_report.txt"

    log_info "Generating flamegraphs..."

    # Wait for perf data to be complete
    sleep 2

    if [ ! -f "$perf_data" ]; then
        log_warn "Perf data file not found: $perf_data"
        return 1
    fi

    # Generate standard flamegraph
    sudo perf script -i "$perf_data" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/stackcollapse-perf.pl" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/flamegraph.pl" \
            --title "LDOS Profiled Experiment - ${output_name}" \
            --subtitle "Scenario: ${SCENARIO} | Duration: ${PROFILE_DURATION}s" \
            --width 1800 \
        > "$flamegraph"

    # Generate reverse flamegraph
    sudo perf script -i "$perf_data" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/stackcollapse-perf.pl" 2>/dev/null | \
        "${FLAMEGRAPH_DIR}/flamegraph.pl" --reverse \
            --title "LDOS Profiled Experiment (Reverse) - ${output_name}" \
            --subtitle "Scenario: ${SCENARIO} - Callee-oriented view" \
            --width 1800 \
        > "$flamegraph_rev"

    # Generate text report
    {
        echo "=== Profiled Experiment Report ==="
        echo "Scenario: ${SCENARIO}"
        echo "Profile Name: ${output_name}"
        echo "Duration: ${PROFILE_DURATION}s"
        echo "Generated: $(date -Iseconds)"
        echo ""
        echo "=== Top Functions (by CPU time) ==="
        sudo perf report -i "$perf_data" --stdio --no-children 2>/dev/null | head -80
    } > "$report"

    # Fix permissions
    sudo chown "$(id -u):$(id -g)" "$perf_data" "$flamegraph" "$flamegraph_rev" "$report" 2>/dev/null || true

    log_info "Flamegraphs generated:"
    log_info "  Standard: $flamegraph"
    log_info "  Reverse:  $flamegraph_rev"
    log_info "  Report:   $report"
}

# Main
main() {
    echo ""
    echo "=============================================="
    echo "  LDOS Profiled Experiment Runner"
    echo "=============================================="
    echo ""

    source_ros
    check_prerequisites

    mkdir -p "$OUTPUT_DIR"
    mkdir -p "$RESULT_DIR"

    log_section "Configuration"
    log_info "Scenario: $SCENARIO"
    log_info "Profile duration: ${PROFILE_DURATION}s"
    log_info "Profile name: $PROFILE_NAME"
    log_info "Output directory: $OUTPUT_DIR"

    log_section "Starting ROS Stack"
    log_info "Launching Gazebo + Panda + MoveIt (headless)..."

    ros2 launch ldos_harness full_stack.launch.py headless:=true use_rviz:=false &
    STACK_PID=$!

    # Wait for stack to initialize with polling
    log_info "Waiting for stack to initialize..."
    STACK_TIMEOUT=120
    STACK_ELAPSED=0
    STACK_POLL_INTERVAL=5

    while [ $STACK_ELAPSED -lt $STACK_TIMEOUT ]; do
        if ros2 action list 2>/dev/null | grep -q "$MOVEIT_ACTION_NAME"; then
            log_info "Stack ready after ${STACK_ELAPSED}s"
            break
        fi

        if ! kill -0 $STACK_PID 2>/dev/null; then
            log_error "Stack process died unexpectedly"
            exit 1
        fi

        sleep $STACK_POLL_INTERVAL
        STACK_ELAPSED=$((STACK_ELAPSED + STACK_POLL_INTERVAL))
        log_info "  Waiting... (${STACK_ELAPSED}s / ${STACK_TIMEOUT}s)"
    done

    if [ $STACK_ELAPSED -ge $STACK_TIMEOUT ]; then
        log_error "Stack failed to start within ${STACK_TIMEOUT}s"
        exit 1
    fi

    # Start load generator if needed
    if [ "$SCENARIO" = "cpu_load" ]; then
        log_section "Starting CPU Load"
        "$SCRIPT_DIR/cpu_load.sh" 4 "$PROFILE_DURATION" &
        LOAD_PID=$!
        log_info "Waiting for load to stabilize (10s)..."
        sleep 10
    elif [ "$SCENARIO" = "msg_load" ]; then
        log_section "Starting Message Load"
        "$SCRIPT_DIR/msg_load.sh" 1000 4 "$PROFILE_DURATION" &
        LOAD_PID=$!
        log_info "Waiting for load to stabilize (10s)..."
        sleep 10
    fi

    log_section "Starting CPU Profiling"
    start_profiling "$PROFILE_DURATION" "$PROFILE_NAME"

    # Brief settle time
    sleep 2

    log_section "Running Benchmark"
    BENCHMARK_EXIT=0
    ros2 run ldos_harness benchmark_runner.py \
        --trial-id "$PROFILE_NAME" \
        --scenario "$SCENARIO" \
        --output-dir "$RESULT_DIR" \
        --timeout "$BENCHMARK_TIMEOUT" \
        --action-name "$MOVEIT_ACTION_NAME" || BENCHMARK_EXIT=$?

    # Wait for profiling to complete
    log_section "Finalizing Profile"
    if [ -n "$PROFILE_PID" ] && kill -0 "$PROFILE_PID" 2>/dev/null; then
        log_info "Waiting for profiler to complete..."
        wait "$PROFILE_PID" 2>/dev/null || true
    fi

    # Generate flamegraphs
    generate_flamegraphs "$PROFILE_NAME"

    log_section "Experiment Complete"
    log_info "Scenario: $SCENARIO"
    log_info "Benchmark exit code: $BENCHMARK_EXIT"
    echo ""
    echo -e "${CYAN}=== Output Files ===${NC}"
    echo "  Benchmark result: ${RESULT_DIR}/${PROFILE_NAME}_result.json"
    echo "  Flamegraph:       ${OUTPUT_DIR}/${PROFILE_NAME}_flamegraph.svg"
    echo "  Reverse graph:    ${OUTPUT_DIR}/${PROFILE_NAME}_flamegraph_reverse.svg"
    echo "  CPU report:       ${OUTPUT_DIR}/${PROFILE_NAME}_report.txt"
    echo ""
    echo "Open the SVG files in a browser to correlate CPU usage with benchmark results."

    exit $BENCHMARK_EXIT
}

main "$@"

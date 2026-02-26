#!/bin/bash
# cpu_load.sh - Generate CPU contention for load testing
# Usage: ./cpu_load.sh [num_workers] [duration_seconds] [cpu_load_percent]
#
# Uses stress-ng to create CPU load. Pins workers to specific cores
# to create predictable contention patterns.

set -euo pipefail

NUM_WORKERS="${1:-4}"
DURATION="${2:-300}"  # 5 minutes default, 0 = run until killed
CPU_LOAD_PERCENT="${3:-80}"

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[CPU_LOAD]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check stress-ng is available
if ! command -v stress-ng &> /dev/null; then
    log_error "stress-ng not found. Install with: sudo apt install stress-ng"
    exit 1
fi

# Get CPU count
NUM_CPUS=$(nproc)
log_info "System has $NUM_CPUS CPUs"
if [ "$DURATION" -gt 0 ]; then
    log_info "Starting $NUM_WORKERS CPU stress workers for ${DURATION}s at ${CPU_LOAD_PERCENT}% load"
else
    log_info "Starting $NUM_WORKERS CPU stress workers until killed at ${CPU_LOAD_PERCENT}% load"
fi

STRESS_ARGS=(
    --cpu "$NUM_WORKERS"
    --cpu-method matrixprod
    --cpu-load "$CPU_LOAD_PERCENT"
    --metrics-brief
)
if [ "$DURATION" -gt 0 ]; then
    STRESS_ARGS+=(--timeout "${DURATION}s")
fi

# Calculate which CPUs to use
# Leave CPU 0 for system tasks, use remaining CPUs
if [ "$NUM_CPUS" -gt 1 ]; then
    # Pin to CPUs 1 through min(NUM_WORKERS, NUM_CPUS-1)
    MAX_CPU=$((NUM_CPUS - 1))
    if [ "$NUM_WORKERS" -gt "$MAX_CPU" ]; then
        NUM_WORKERS=$MAX_CPU
    fi

    CPU_LIST="1-$((NUM_WORKERS))"
    log_info "Pinning workers to CPUs: $CPU_LIST"

    # Use taskset to pin stress-ng workers
    taskset -c "$CPU_LIST" stress-ng "${STRESS_ARGS[@]}" &
else
    # Single CPU system - just run without pinning
    log_info "Single CPU system - running without affinity"
    stress-ng "${STRESS_ARGS[@]}" &
fi

STRESS_PID=$!
log_info "stress-ng started with PID: $STRESS_PID"

# Verify stress-ng actually started
sleep 0.5
if ! kill -0 "$STRESS_PID" 2>/dev/null; then
    log_error "stress-ng failed to start"
    exit 1
fi

# Handle termination - include EXIT to catch all exit paths
cleanup() {
    log_info "Cleaning up stress-ng processes..."
    # Kill the main stress-ng process
    kill "$STRESS_PID" 2>/dev/null || true
    # Also kill any child processes (stress-ng spawns workers)
    pkill -P "$STRESS_PID" 2>/dev/null || true
    # Note: removed pkill -P $$ which would kill sibling processes in experiment suite
}
trap cleanup SIGTERM SIGINT EXIT

# Wait for stress-ng to complete
wait $STRESS_PID 2>/dev/null || true

log_info "CPU load generator finished"

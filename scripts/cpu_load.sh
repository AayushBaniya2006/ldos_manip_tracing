#!/bin/bash
# cpu_load.sh - Generate CPU contention for load testing
# Usage: ./cpu_load.sh [num_workers] [duration_seconds]
#
# Uses stress-ng to create CPU load. Pins workers to specific cores
# to create predictable contention patterns.

set -euo pipefail

NUM_WORKERS="${1:-4}"
DURATION="${2:-300}"  # 5 minutes default

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
log_info "Starting $NUM_WORKERS CPU stress workers for ${DURATION}s"

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
    taskset -c "$CPU_LIST" stress-ng \
        --cpu "$NUM_WORKERS" \
        --cpu-method matrixprod \
        --cpu-load 80 \
        --timeout "${DURATION}s" \
        --metrics-brief &
else
    # Single CPU system - just run without pinning
    log_info "Single CPU system - running without affinity"
    stress-ng \
        --cpu "$NUM_WORKERS" \
        --cpu-method matrixprod \
        --cpu-load 80 \
        --timeout "${DURATION}s" \
        --metrics-brief &
fi

STRESS_PID=$!
log_info "stress-ng started with PID: $STRESS_PID"

# Handle termination
trap "kill $STRESS_PID 2>/dev/null; exit 0" SIGTERM SIGINT

# Wait for stress-ng to complete
wait $STRESS_PID 2>/dev/null || true

log_info "CPU load generator finished"

#!/bin/bash
# msg_load.sh - Generate DDS message load for testing
# Usage: ./msg_load.sh [rate_hz] [num_publishers] [duration_seconds] [payload_size]
#
# Starts multiple publisher nodes that flood the DDS network
# with messages at the specified rate.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

RATE_HZ="${1:-1000}"
NUM_PUBS="${2:-4}"
DURATION="${3:-300}"
PAYLOAD_SIZE="${4:-1024}"

GREEN='\033[0;32m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[MSG_LOAD]${NC} $1"; }

# =============================================================================
# ROS 2 ENVIRONMENT SETUP (function-based for stability with strict mode)
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
    set -u
}

source_ros

log_info "Starting $NUM_PUBS message flood publishers at ${RATE_HZ} Hz each"
log_info "Payload size: ${PAYLOAD_SIZE} bytes"
log_info "Duration: ${DURATION}s"

# Array to store PIDs
PIDS=()
FAILED_SPAWNS=0

# Start multiple flood nodes with verification
for i in $(seq 1 "$NUM_PUBS"); do
    ros2 run ldos_harness msg_flood_node.py \
        --rate "$RATE_HZ" \
        --topic "/flood_topic_$i" \
        --payload-size "$PAYLOAD_SIZE" &
    PID=$!

    # Brief pause to let process start
    sleep 0.2

    # Verify the process actually started
    if kill -0 "$PID" 2>/dev/null; then
        PIDS+=("$PID")
        log_info "Started publisher $i with PID $PID"
    else
        log_info "WARNING: Publisher $i failed to start, retrying..."
        FAILED_SPAWNS=$((FAILED_SPAWNS + 1))

        # Retry once
        ros2 run ldos_harness msg_flood_node.py \
            --rate "$RATE_HZ" \
            --topic "/flood_topic_$i" \
            --payload-size "$PAYLOAD_SIZE" &
        PID=$!
        sleep 0.2

        if kill -0 "$PID" 2>/dev/null; then
            PIDS+=("$PID")
            log_info "Started publisher $i with PID $PID (retry succeeded)"
        else
            log_info "ERROR: Publisher $i failed to start after retry"
        fi
    fi
done

if [ ${#PIDS[@]} -eq 0 ]; then
    log_info "ERROR: No publishers started successfully"
    exit 1
fi

log_info "Successfully started ${#PIDS[@]}/$NUM_PUBS publishers"

if [ "${#PIDS[@]}" -ne "$NUM_PUBS" ] && [ "${ALLOW_PARTIAL_MSG_LOAD:-false}" != "true" ]; then
    log_info "ERROR: Partial publisher startup is not allowed (${#PIDS[@]}/$NUM_PUBS)"
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    exit 1
fi

# Handle termination
cleanup() {
    log_info "Stopping message flood nodes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
}
trap cleanup SIGTERM SIGINT EXIT

# Wait for duration (0 = run until killed)
if [ "$DURATION" -gt 0 ]; then
    sleep "$DURATION"
else
    while true; do
        sleep 3600
    done
fi

log_info "Message load generator finished"

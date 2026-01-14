#!/bin/bash
# msg_load.sh - Generate DDS message load for testing
# Usage: ./msg_load.sh [rate_hz] [num_publishers] [duration_seconds]
#
# Starts multiple publisher nodes that flood the DDS network
# with messages at the specified rate.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

RATE_HZ="${1:-1000}"
NUM_PUBS="${2:-4}"
DURATION="${3:-300}"

GREEN='\033[0;32m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[MSG_LOAD]${NC} $1"; }

# Source ROS (disable -u temporarily for ROS setup scripts)
set +u
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
fi
set -u

log_info "Starting $NUM_PUBS message flood publishers at ${RATE_HZ} Hz each"
log_info "Duration: ${DURATION}s"

# Array to store PIDs
PIDS=()

# Start multiple flood nodes
for i in $(seq 1 "$NUM_PUBS"); do
    ros2 run ldos_harness msg_flood_node.py \
        --rate "$RATE_HZ" \
        --topic "/flood_topic_$i" \
        --payload-size 1024 &
    PIDS+=($!)
    log_info "Started publisher $i with PID ${PIDS[-1]}"
done

# Handle termination
cleanup() {
    log_info "Stopping message flood nodes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
}
trap cleanup SIGTERM SIGINT EXIT

# Wait for duration
sleep "$DURATION"

log_info "Message load generator finished"

#!/bin/bash
# cleanup_harness_processes.sh - Aggressive cleanup for ROS/Gazebo harness state
#
# Usage:
#   ./cleanup_harness_processes.sh [cooldown_seconds]
#
# This is intended for experiment hygiene between runs/sweeps. It kills known
# harness-related processes, destroys leftover LTTng sessions (if present), and
# invokes cpuset cgroup cleanup.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[CLEANUP]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[CLEANUP]${NC} $1"; }
log_step() { echo -e "${CYAN}[CLEANUP]${NC} $1"; }

COOLDOWN_SECONDS="${1:-5}"

kill_pattern() {
    local pattern="$1"
    local label="$2"
    if pkill -9 -f "$pattern" 2>/dev/null; then
        log_info "Killed: $label"
    else
        log_step "No processes matched: $label"
    fi
}

log_info "Starting harness cleanup..."

# Stop known ROS/Gazebo processes used by the harness.
kill_pattern "ros2 launch ldos_harness" "ros2 launch ldos_harness"
kill_pattern "move_group" "move_group"
kill_pattern "gz sim|gazebo" "Gazebo (gz sim / gazebo)"
kill_pattern "controller_manager|ros2 control load_controller" "controller_manager / ros2 control spawners"
kill_pattern "parameter_bridge|robot_state_publisher|ros_gz_sim create" "bridge / robot_state_publisher / spawn helper"
kill_pattern "benchmark_runner.py|ros2 run ldos_harness benchmark_runner.py" "benchmark runner"
kill_pattern "msg_flood_node.py" "message flood nodes"
kill_pattern "stress-ng" "stress-ng load generators"

# Clean up leftover cpuset cgroups if supported.
if [ -x "$SCRIPT_DIR/cpuset_cleanup.sh" ]; then
    "$SCRIPT_DIR/cpuset_cleanup.sh" 2>/dev/null || true
fi

# Clean up LTTng sessions (optional).
if command -v lttng >/dev/null 2>&1; then
    if lttng destroy --all >/dev/null 2>&1; then
        log_info "Destroyed any leftover LTTng sessions"
    else
        log_step "No LTTng sessions to destroy"
    fi
else
    log_step "lttng not installed; skipping LTTng cleanup"
fi

if [[ "$COOLDOWN_SECONDS" =~ ^[0-9]+$ ]] && [ "$COOLDOWN_SECONDS" -gt 0 ]; then
    log_info "Cooldown for ${COOLDOWN_SECONDS}s..."
    sleep "$COOLDOWN_SECONDS"
fi

log_info "Cleanup complete."

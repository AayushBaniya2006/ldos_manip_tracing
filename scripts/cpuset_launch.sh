#!/bin/bash
# cpuset_launch.sh - Launch process with cgroups v2 CPU restriction
#
# Usage: ./cpuset_launch.sh <cpus> <name> <command...>
#
# Examples:
#   ./cpuset_launch.sh "0-5" "gazebo" ros2 launch ldos_harness sim_only.launch.py
#   ./cpuset_launch.sh "6,7" "ros_stack" ros2 launch ldos_harness ros_stack.launch.py
#
# This script uses systemd-run with AllowedCPUs to enforce cgroups v2 cpuset
# restrictions on the launched process and all its children.
#
# Requirements:
#   - cgroups v2 enabled (default on Ubuntu 24.04)
#   - systemd --user session active
#   - No root required

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[CPUSET]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[CPUSET]${NC} $1"; }
log_error() { echo -e "${RED}[CPUSET ERROR]${NC} $1"; }

# Validate arguments
if [ $# -lt 3 ]; then
    log_error "Usage: $0 <cpus> <name> <command...>"
    log_error "Example: $0 \"6,7\" \"ros_stack\" ros2 launch ldos_harness ros_stack.launch.py"
    exit 1
fi

CPUS="${1}"
CGROUP_NAME="${2}"
shift 2
COMMAND=("$@")

# Validate CPU specification
if ! [[ "$CPUS" =~ ^[0-9]+([-,][0-9]+)*$ ]]; then
    log_error "Invalid CPU specification: $CPUS"
    log_error "Use format like: 0-3 or 0,1,2 or 4-7"
    exit 1
fi

# Check if systemd --user session is available
if ! systemctl --user status >/dev/null 2>&1; then
    log_warn "systemd --user session not active, trying to start..."
    # This might happen in SSH sessions without lingering enabled
    if ! loginctl enable-linger "$USER" 2>/dev/null; then
        log_warn "Could not enable lingering, proceeding anyway..."
    fi
fi

# Generate unique unit name
UNIT_NAME="${CGROUP_NAME}_$(date +%s)_$$"

log_info "Launching '$CGROUP_NAME' restricted to CPUs: $CPUS"
log_info "Unit name: $UNIT_NAME"
log_info "Command: ${COMMAND[*]}"

# Use systemd-run with AllowedCPUs (cgroups v2, no root required)
# --scope: Run in foreground scope (not background service)
# --user: Run under user session (no root)
# -p AllowedCPUs: cgroups v2 cpuset restriction
# --unit: Name for the transient unit
# --description: Human-readable description
exec systemd-run --user --scope \
    -p AllowedCPUs="$CPUS" \
    --unit="$UNIT_NAME" \
    --description="LDOS $CGROUP_NAME (CPUs: $CPUS)" \
    -- "${COMMAND[@]}"

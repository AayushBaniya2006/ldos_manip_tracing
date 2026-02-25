#!/bin/bash
# cpuset_launch_v2.sh - Launch process with REAL cgroups v2 cpuset restriction
#
# This script uses direct cgroup manipulation to ensure ALL child processes
# are restricted to the specified CPUs. The systemd-run approach doesn't work
# reliably because ROS 2 child processes can escape to the default cgroup.
#
# Usage: ./cpuset_launch_v2.sh <cpus> <name> <command...>
#
# Examples:
#   ./cpuset_launch_v2.sh "0-5" "gazebo" ros2 launch ldos_harness sim_only.launch.py
#   ./cpuset_launch_v2.sh "38-39" "ros_stack" ros2 launch ldos_harness ros_stack.launch.py
#
# PREREQUISITES:
#   1. cgroups v2 enabled (default on Ubuntu 24.04)
#   2. cpuset delegation enabled - run: sudo scripts/setup_cpuset_delegation.sh
#   3. Logout and login after enabling delegation
#
# HOW IT WORKS:
#   1. Verify cpuset is delegated to user session
#   2. Enable cpuset controller in parent's subtree_control (BEFORE creating child)
#   3. Create child cgroup
#   4. Set CPU and memory node restrictions
#   5. Move current shell into the cgroup
#   6. Launch command - all children inherit the cgroup
#
# FALLBACKS:
#   1. Prefer systemd-run --user --scope -p AllowedCPUs=... (real cgroup cpuset)
#   2. If unavailable and allowed, fall back to taskset (weaker isolation)

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[CPUSET]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[CPUSET]${NC} $1"; }
log_error() { echo -e "${RED}[CPUSET ERROR]${NC} $1"; }
log_debug() { echo -e "${CYAN}[CPUSET DEBUG]${NC} $1"; }

ALLOW_TASKSET_FALLBACK="${CPUSET_ALLOW_TASKSET_FALLBACK:-1}"
ALLOW_SYSTEMD_SCOPE_FALLBACK="${CPUSET_ALLOW_SYSTEMD_SCOPE_FALLBACK:-1}"

fallback_or_fail() {
    local reason="$1"
    log_error "$reason"
    if [ "$ALLOW_SYSTEMD_SCOPE_FALLBACK" = "1" ] && command -v systemd-run >/dev/null 2>&1; then
        local unit_name="ldos_${CGROUP_NAME}_$(date +%s)_$$"
        log_warn "Falling back to systemd-run --user scope with AllowedCPUs (real cgroup cpuset)..."
        log_info "Unit name: $unit_name"
        exec systemd-run --user --scope \
            -p AllowedCPUs="$CPUS" \
            --unit="$unit_name" \
            --description="LDOS $CGROUP_NAME (CPUs: $CPUS)" \
            -- "${COMMAND[@]}"
    fi
    if [ "$ALLOW_TASKSET_FALLBACK" = "0" ]; then
        log_error "Taskset fallback is disabled (CPUSET_ALLOW_TASKSET_FALLBACK=0)."
        log_error "Aborting so results are not mislabeled as cpuset-isolated."
        exit 2
    fi
    log_warn "Falling back to taskset (weaker isolation)..."
    log_info "Launching with taskset -c $CPUS: ${COMMAND[*]}"
    exec taskset -c "$CPUS" "${COMMAND[@]}"
}

# Validate arguments
if [ $# -lt 3 ]; then
    log_error "Usage: $0 <cpus> <name> <command...>"
    log_error "Example: $0 \"38-39\" \"ros_stack\" ros2 launch ldos_harness ros_stack.launch.py"
    exit 1
fi

CPUS="${1}"
CGROUP_NAME="${2}"
shift 2
COMMAND=("$@")

# Validate CPU specification format
if ! [[ "$CPUS" =~ ^[0-9]+([-,][0-9]+)*$ ]]; then
    log_error "Invalid CPU specification: $CPUS"
    log_error "Use format like: 0-3 or 0,1,2 or 4-7 or 38-39"
    exit 1
fi

# User cgroup base path (cgroups v2 unified hierarchy)
USER_ID=$(id -u)
CGROUP_BASE="/sys/fs/cgroup/user.slice/user-${USER_ID}.slice/user@${USER_ID}.service"
CGROUP_PATH="${CGROUP_BASE}/ldos_${CGROUP_NAME}.scope"

log_info "Target cgroup: $CGROUP_PATH"
log_info "CPU restriction: $CPUS"
CURRENT_CGROUP_REL="$(awk -F: '$1=="0"{print $3}' /proc/self/cgroup 2>/dev/null || true)"
[ -n "$CURRENT_CGROUP_REL" ] && log_debug "Current process cgroup: $CURRENT_CGROUP_REL"

# =============================================================================
# STEP 1: Verify cpuset is delegated to user session
# =============================================================================

if [ ! -d "$CGROUP_BASE" ]; then
    fallback_or_fail "User cgroup base not found: $CGROUP_BASE (systemd user session may not be active)"
fi

if [ ! -f "${CGROUP_BASE}/cgroup.controllers" ]; then
    fallback_or_fail "cgroup.controllers not found in user cgroup"
fi

# Check if cpuset is available (delegated)
AVAILABLE_CONTROLLERS=$(cat "${CGROUP_BASE}/cgroup.controllers" 2>/dev/null || echo "")
log_debug "Available controllers: $AVAILABLE_CONTROLLERS"

if ! echo "$AVAILABLE_CONTROLLERS" | grep -q "cpuset"; then
    log_error "cpuset controller NOT available in user cgroup!"
    log_error "This means cpuset delegation is not enabled."
    log_error ""
    log_error "To fix, run:"
    log_error "  sudo ./scripts/setup_cpuset_delegation.sh"
    log_error "  # Then logout and login again"
    log_error ""
    fallback_or_fail "cpuset controller NOT available in user cgroup"
fi

log_info "cpuset controller is delegated"

# =============================================================================
# STEP 2: Enable cpuset in parent's subtree_control (BEFORE creating child)
# =============================================================================

SUBTREE_CONTROL="${CGROUP_BASE}/cgroup.subtree_control"
CURRENT_SUBTREE=$(cat "$SUBTREE_CONTROL" 2>/dev/null || echo "")
log_debug "Current subtree_control: $CURRENT_SUBTREE"

if ! echo "$CURRENT_SUBTREE" | grep -q "cpuset"; then
    log_info "Enabling cpuset in subtree_control..."
    if ! echo "+cpuset" > "$SUBTREE_CONTROL" 2>/dev/null; then
        log_warn "This may be due to 'no internal process' constraint"
        log_warn "Try: logout and login again"
        fallback_or_fail "Cannot enable cpuset in subtree_control"
    fi
    log_info "cpuset enabled in subtree_control"
else
    log_debug "cpuset already enabled in subtree_control"
fi

# =============================================================================
# STEP 3: Create child cgroup (AFTER controller is enabled)
# =============================================================================

# Clean up existing cgroup if it exists (from previous failed run)
if [ -d "$CGROUP_PATH" ]; then
    log_warn "Cgroup already exists, cleaning up..."
    # Move any processes out first
    if [ -f "${CGROUP_PATH}/cgroup.procs" ]; then
        while read -r pid; do
            if [ -n "$pid" ]; then
                echo "$pid" > "${CGROUP_BASE}/cgroup.procs" 2>/dev/null || true
            fi
        done < "${CGROUP_PATH}/cgroup.procs"
    fi
    rmdir "$CGROUP_PATH" 2>/dev/null || true
fi

log_info "Creating cgroup: $CGROUP_PATH"
if ! mkdir -p "$CGROUP_PATH" 2>/dev/null; then
    fallback_or_fail "Cannot create cgroup directory: $CGROUP_PATH"
fi

# =============================================================================
# STEP 4: Set CPU and memory node restrictions
# =============================================================================

# Verify cpuset.cpus file exists (confirms controller is enabled for this cgroup)
if [ ! -f "${CGROUP_PATH}/cpuset.cpus" ]; then
    log_error "This means cpuset controller is not enabled for this cgroup"
    rmdir "$CGROUP_PATH" 2>/dev/null || true
    fallback_or_fail "cpuset.cpus not found in cgroup"
fi

# Set memory node first (required before cpuset.cpus on cgroups v2)
# Use parent's effective mems to handle NUMA systems correctly.
if [ -f "${CGROUP_PATH}/cpuset.mems" ]; then
    PARENT_MEMS=$(cat "${CGROUP_BASE}/cpuset.mems.effective" 2>/dev/null || echo "")
    [ -z "$PARENT_MEMS" ] && PARENT_MEMS=$(cat "${CGROUP_BASE}/cpuset.mems" 2>/dev/null || echo "0")
    [ -z "$PARENT_MEMS" ] && PARENT_MEMS="0"
    log_debug "Setting cpuset.mems = $PARENT_MEMS (from parent)"
    if ! echo "$PARENT_MEMS" > "${CGROUP_PATH}/cpuset.mems" 2>/dev/null; then
        log_warn "Could not set cpuset.mems=$PARENT_MEMS"
    fi
fi

# Set CPU restriction
log_info "Setting cpuset.cpus = $CPUS"
if ! echo "$CPUS" > "${CGROUP_PATH}/cpuset.cpus" 2>/dev/null; then
    rmdir "$CGROUP_PATH" 2>/dev/null || true
    fallback_or_fail "Cannot write to cpuset.cpus"
fi

# =============================================================================
# STEP 5: Move current shell into the cgroup
# =============================================================================

log_info "Moving shell (PID $$) to cgroup..."
if ! echo $$ > "${CGROUP_PATH}/cgroup.procs" 2>/dev/null; then
    log_warn "Current shell may be in a systemd session scope and cannot be migrated into delegated user@ service subtree"
    rmdir "$CGROUP_PATH" 2>/dev/null || true
    fallback_or_fail "Cannot move process to cgroup"
fi

log_info "Shell moved to restricted cgroup"

# =============================================================================
# STEP 6: Verify effective CPUs
# =============================================================================

if [ -f "${CGROUP_PATH}/cpuset.cpus.effective" ]; then
    EFFECTIVE_CPUS=$(cat "${CGROUP_PATH}/cpuset.cpus.effective")
    log_info "Effective CPUs: $EFFECTIVE_CPUS"

    # Verify effective matches requested
    if [ "$EFFECTIVE_CPUS" != "$CPUS" ]; then
        log_warn "Effective CPUs ($EFFECTIVE_CPUS) differ from requested ($CPUS)"
        log_warn "This may be due to parent cgroup restrictions"
    fi
else
    log_debug "cpuset.cpus.effective not available"
fi

# Show cgroup path for verification
log_info "Cgroup path: $CGROUP_PATH"

# =============================================================================
# STEP 7: Launch command with belt-and-suspenders
# =============================================================================

# Use taskset as additional enforcement
# Even though cgroup should restrict CPUs, taskset provides extra guarantee
log_info "Launching: ${COMMAND[*]}"
log_info "All child processes will inherit cgroup restriction"

exec taskset -c "$CPUS" "${COMMAND[@]}"

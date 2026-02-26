#!/bin/bash
# cpuset_launch_v2.sh - Launch process with CPU restriction
#
# PRIMARY: taskset (sched_setaffinity) - inherited by all fork/exec children
# ENHANCEMENT: cgroups v2 cpuset (if available) - defense-in-depth
#
# Usage: ./cpuset_launch_v2.sh <cpus> <name> <command...>
#
# Examples:
#   ./cpuset_launch_v2.sh "0-5" "gazebo" ros2 launch ldos_harness sim_only.launch.py
#   ./cpuset_launch_v2.sh "38-39" "ros_stack" ros2 launch ldos_harness ros_stack.launch.py
#
# HOW IT WORKS:
#   1. Best-effort: Try to create cgroups v2 cpuset restriction (may fail on SSH sessions)
#   2. Always: Launch command via taskset -c (sched_setaffinity, inherited by ALL children)
#
# CPU affinity set via sched_setaffinity is:
#   - Inherited across fork(2)
#   - Preserved across execve(2)
#   - Enforced by the Linux scheduler (process CANNOT run on disallowed CPUs)
#
# The cgroup enhancement adds defense-in-depth but is NOT required.
# taskset alone provides complete isolation for well-behaved processes.

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

# =============================================================================
# BEST-EFFORT: Try cgroup v2 cpuset enhancement
# =============================================================================

try_cgroup_enhancement() {
    # User cgroup base path (cgroups v2 unified hierarchy)
    local user_id
    user_id=$(id -u)
    local cgroup_base="/sys/fs/cgroup/user.slice/user-${user_id}.slice/user@${user_id}.service"
    local cgroup_path="${cgroup_base}/ldos_${CGROUP_NAME}.scope"

    log_debug "Attempting cgroup enhancement at: $cgroup_path"

    # Step 1: Verify cpuset is delegated
    if [ ! -d "$cgroup_base" ]; then
        log_debug "User cgroup base not found: $cgroup_base"
        return 1
    fi

    local available_controllers
    available_controllers=$(cat "${cgroup_base}/cgroup.controllers" 2>/dev/null || echo "")
    if ! echo "$available_controllers" | grep -q "cpuset"; then
        log_debug "cpuset controller not delegated"
        return 1
    fi

    # Step 2: Enable cpuset in parent's subtree_control
    local subtree_control="${cgroup_base}/cgroup.subtree_control"
    if ! cat "$subtree_control" 2>/dev/null | grep -q "cpuset"; then
        if ! echo "+cpuset" > "$subtree_control" 2>/dev/null; then
            log_debug "Cannot enable cpuset in subtree_control"
            return 1
        fi
    fi

    # Step 3: Create child cgroup
    if [ -d "$cgroup_path" ]; then
        # Clean up existing cgroup from previous run
        if [ -f "${cgroup_path}/cgroup.procs" ]; then
            while read -r pid; do
                [ -n "$pid" ] && echo "$pid" > "${cgroup_base}/cgroup.procs" 2>/dev/null || true
            done < "${cgroup_path}/cgroup.procs"
        fi
        rmdir "$cgroup_path" 2>/dev/null || true
    fi

    if ! mkdir -p "$cgroup_path" 2>/dev/null; then
        log_debug "Cannot create cgroup directory"
        return 1
    fi

    # Step 4: Set CPU and memory restrictions
    if [ ! -f "${cgroup_path}/cpuset.cpus" ]; then
        rmdir "$cgroup_path" 2>/dev/null || true
        log_debug "cpuset.cpus not found in cgroup"
        return 1
    fi

    if [ -f "${cgroup_path}/cpuset.mems" ]; then
        local parent_mems
        parent_mems=$(cat "${cgroup_base}/cpuset.mems.effective" 2>/dev/null || echo "")
        [ -z "$parent_mems" ] && parent_mems=$(cat "${cgroup_base}/cpuset.mems" 2>/dev/null || echo "0")
        [ -z "$parent_mems" ] && parent_mems="0"
        echo "$parent_mems" > "${cgroup_path}/cpuset.mems" 2>/dev/null || true
    fi

    if ! echo "$CPUS" > "${cgroup_path}/cpuset.cpus" 2>/dev/null; then
        rmdir "$cgroup_path" 2>/dev/null || true
        log_debug "Cannot write cpuset.cpus"
        return 1
    fi

    # Step 5: Move current shell into the cgroup
    if ! echo $$ > "${cgroup_path}/cgroup.procs" 2>/dev/null; then
        log_debug "Cannot move shell to cgroup (likely cross-branch migration from SSH session scope)"
        # Don't clean up cgroup — it might be usable later
        return 1
    fi

    # Step 6: Verify effective CPUs
    if [ -f "${cgroup_path}/cpuset.cpus.effective" ]; then
        local effective_cpus
        effective_cpus=$(cat "${cgroup_path}/cpuset.cpus.effective")
        log_info "Cgroup cpuset effective CPUs: $effective_cpus"
    fi

    log_info "Cgroup enhancement active: $cgroup_path"
    return 0
}

# Try cgroup enhancement (best-effort)
ISOLATION_METHOD="taskset_only"
if try_cgroup_enhancement; then
    ISOLATION_METHOD="cgroup+taskset"
    log_info "Cgroup cpuset enhancement: SUCCESS"
else
    log_warn "Cgroup cpuset enhancement: UNAVAILABLE (taskset will enforce isolation)"
fi

# Export method for callers to inspect
export CPUSET_ISOLATION_METHOD="$ISOLATION_METHOD"

# =============================================================================
# PRIMARY: Launch with taskset (ALWAYS runs)
# =============================================================================
# sched_setaffinity is inherited by fork(2) and preserved across execve(2).
# The Linux scheduler enforces this — processes CANNOT run on disallowed CPUs.

log_info "Launching '$CGROUP_NAME' restricted to CPUs: $CPUS"
log_info "Isolation method: $ISOLATION_METHOD"
log_info "Command: ${COMMAND[*]}"

exec taskset -c "$CPUS" "${COMMAND[@]}"

#!/bin/bash
# cpuset_cleanup.sh - Clean up any leftover cgroups from failed experiments
#
# Usage: ./cpuset_cleanup.sh
#
# This script removes any ldos_*.scope cgroups that may have been left
# behind from failed or interrupted experiments.

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[CLEANUP]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[CLEANUP]${NC} $1"; }
log_error() { echo -e "${RED}[CLEANUP ERROR]${NC} $1"; }

USER_ID=$(id -u)
CGROUP_BASE="/sys/fs/cgroup/user.slice/user-${USER_ID}.slice/user@${USER_ID}.service"

log_info "Cleaning up leftover cgroups..."
log_info "Base path: $CGROUP_BASE"

# Check if base path exists
if [ ! -d "$CGROUP_BASE" ]; then
    log_warn "Cgroup base path does not exist: $CGROUP_BASE"
    log_info "This may mean cgroups v2 user delegation is not enabled."
    exit 0
fi

# Find and clean up ldos_*.scope cgroups
CLEANED=0
for cgroup in "$CGROUP_BASE"/ldos_*.scope; do
    if [ -d "$cgroup" ]; then
        log_info "Found: $cgroup"

        # Check for remaining processes
        if [ -f "$cgroup/cgroup.procs" ]; then
            PROCS=$(cat "$cgroup/cgroup.procs" 2>/dev/null | wc -l)
            if [ "$PROCS" -gt 0 ]; then
                log_warn "  Moving $PROCS process(es) to parent cgroup..."
                # Move processes to parent cgroup
                while read -r pid; do
                    if [ -n "$pid" ]; then
                        echo "$pid" > "${CGROUP_BASE}/cgroup.procs" 2>/dev/null || \
                            log_warn "    Could not move PID $pid (may have already exited)"
                    fi
                done < "$cgroup/cgroup.procs"
            fi
        fi

        # Try to remove the cgroup
        if rmdir "$cgroup" 2>/dev/null; then
            log_info "  Removed successfully"
            CLEANED=$((CLEANED + 1))
        else
            log_warn "  Could not remove (may have processes or nested cgroups)"
        fi
    fi
done

if [ "$CLEANED" -eq 0 ]; then
    log_info "No leftover cgroups found."
else
    log_info "Cleaned up $CLEANED cgroup(s)."
fi

# Also clean up any zombie systemd units from old cpuset_launch.sh
log_info "Checking for leftover systemd units..."
UNITS=$(systemctl --user list-units --type=scope --state=failed 2>/dev/null | grep -E "(gazebo|ros_stack)" || true)
if [ -n "$UNITS" ]; then
    log_info "Resetting failed systemd units..."
    systemctl --user reset-failed 2>/dev/null || true
fi

log_info "Cleanup complete."

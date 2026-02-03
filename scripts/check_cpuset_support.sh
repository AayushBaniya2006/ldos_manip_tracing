#!/bin/bash
# check_cpuset_support.sh - Verify cgroups v2 cpuset support
#
# This script checks that the system supports cgroups v2 with cpuset
# controller and that systemd --user scope works for CPU restriction.
#
# Exit codes:
#   0 - All checks passed
#   1 - Critical check failed (cpuset won't work)
#   2 - Warning (cpuset may have limited functionality)

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

PASS="${GREEN}✓${NC}"
FAIL="${RED}✗${NC}"
WARN="${YELLOW}⚠${NC}"

ERRORS=0
WARNINGS=0

echo "=== Checking cgroups v2 cpuset support ==="
echo ""

# Check 1: cgroups v2 mounted
echo -n "Checking cgroups v2... "
if [ -f /sys/fs/cgroup/cgroup.controllers ]; then
    echo -e "$PASS cgroups v2 detected at /sys/fs/cgroup"
else
    echo -e "$FAIL cgroups v2 not found"
    echo "    cgroups v1 may be in use. Ubuntu 24.04+ uses v2 by default."
    echo "    To enable v2, add 'systemd.unified_cgroup_hierarchy=1' to kernel cmdline"
    ERRORS=$((ERRORS + 1))
fi

# Check 2: cpuset controller available
echo -n "Checking cpuset controller... "
if grep -q "cpuset" /sys/fs/cgroup/cgroup.controllers 2>/dev/null; then
    echo -e "$PASS cpuset controller available"
else
    echo -e "$FAIL cpuset controller not available"
    ERRORS=$((ERRORS + 1))
fi

# Check 3: User slice exists (for --user scope)
echo -n "Checking user slice... "
if [ -d "/sys/fs/cgroup/user.slice" ]; then
    echo -e "$PASS user.slice exists"
else
    echo -e "$WARN user.slice not found (may still work)"
    WARNINGS=$((WARNINGS + 1))
fi

# Check 4: systemd --user session
echo -n "Checking systemd --user session... "
if systemctl --user status >/dev/null 2>&1; then
    echo -e "$PASS systemd --user session active"
else
    echo -e "$WARN systemd --user session not active"
    echo "    Run: systemctl --user start default.target"
    echo "    Or enable lingering: loginctl enable-linger $USER"
    WARNINGS=$((WARNINGS + 1))
fi

# Check 5: systemd-run with AllowedCPUs
echo -n "Testing systemd-run with AllowedCPUs... "
if systemd-run --user --scope -p AllowedCPUs=0 -- true 2>/dev/null; then
    echo -e "$PASS AllowedCPUs property works"
else
    echo -e "$FAIL systemd-run with AllowedCPUs failed"
    echo "    This is required for cpuset isolation"
    ERRORS=$((ERRORS + 1))
fi

# Check 6: CPU count
echo ""
echo -n "Checking CPU count... "
NUM_CPUS=$(nproc)
echo -e "$PASS System has $NUM_CPUS CPUs available"

if [ "$NUM_CPUS" -lt 4 ]; then
    echo -e "    $WARN Less than 4 CPUs - limited isolation possible"
    echo "    Recommended: At least 4 CPUs (2 for Gazebo, 2+ for ROS)"
    WARNINGS=$((WARNINGS + 1))
elif [ "$NUM_CPUS" -lt 8 ]; then
    echo "    Recommended CPU split: Gazebo on 0-$((NUM_CPUS - 3)), ROS on $((NUM_CPUS - 2))-$((NUM_CPUS - 1))"
else
    echo "    Recommended CPU split: Gazebo on 0-$((NUM_CPUS - 3)), ROS on $((NUM_CPUS - 2))-$((NUM_CPUS - 1))"
fi

# Check 7: Current user cgroup delegation
echo ""
echo -n "Checking user cgroup delegation... "
USER_CGROUP="/sys/fs/cgroup/user.slice/user-$(id -u).slice"
if [ -d "$USER_CGROUP" ]; then
    if [ -w "$USER_CGROUP/cgroup.procs" ] 2>/dev/null; then
        echo -e "$PASS User has cgroup write access"
    else
        echo -e "$WARN User may have limited cgroup access"
        WARNINGS=$((WARNINGS + 1))
    fi
else
    echo -e "$WARN User cgroup path not found"
    WARNINGS=$((WARNINGS + 1))
fi

# Summary
echo ""
echo "=== Summary ==="
if [ "$ERRORS" -eq 0 ] && [ "$WARNINGS" -eq 0 ]; then
    echo -e "${GREEN}All checks passed! cpuset isolation is fully supported.${NC}"
    exit 0
elif [ "$ERRORS" -eq 0 ]; then
    echo -e "${YELLOW}Passed with $WARNINGS warning(s). cpuset isolation should work.${NC}"
    exit 0
else
    echo -e "${RED}$ERRORS critical error(s), $WARNINGS warning(s).${NC}"
    echo "cpuset isolation may not work correctly."
    exit 1
fi

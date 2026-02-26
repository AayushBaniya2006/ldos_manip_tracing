#!/bin/bash
# check_cpuset_support.sh - Comprehensive verification of cgroups v2 cpuset support
#
# This script checks that the system supports cgroups v2 with cpuset controller
# and that cpuset delegation is properly configured for user sessions.
#
# Exit codes:
#   0 - All checks passed
#   1 - Critical check failed (cpuset won't work)
#   2 - Warning (cpuset may have limited functionality)
#
# Usage: ./check_cpuset_support.sh

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS="${GREEN}[PASS]${NC}"
FAIL="${RED}[FAIL]${NC}"
WARN="${YELLOW}[WARN]${NC}"
INFO="${CYAN}[INFO]${NC}"

ERRORS=0
WARNINGS=0

echo ""
echo "=========================================="
echo "  cgroups v2 cpuset Support Verification"
echo "=========================================="
echo ""

# =============================================================================
# Check 1: cgroups v2 mounted
# =============================================================================

echo -n "1. Checking cgroups v2... "
if [ -f /sys/fs/cgroup/cgroup.controllers ]; then
    echo -e "$PASS cgroups v2 detected at /sys/fs/cgroup"
else
    echo -e "$FAIL cgroups v2 not found"
    echo "   cgroups v1 may be in use. Ubuntu 24.04+ uses v2 by default."
    echo "   To enable v2, add 'systemd.unified_cgroup_hierarchy=1' to kernel cmdline"
    ERRORS=$((ERRORS + 1))
fi

# =============================================================================
# Check 2: cpuset controller available at root
# =============================================================================

echo -n "2. Checking cpuset controller (root)... "
if grep -q "cpuset" /sys/fs/cgroup/cgroup.controllers 2>/dev/null; then
    echo -e "$PASS cpuset controller available"
else
    echo -e "$FAIL cpuset controller not available"
    echo "   This may require kernel configuration changes"
    ERRORS=$((ERRORS + 1))
fi

# =============================================================================
# Check 3: User slice exists
# =============================================================================

echo -n "3. Checking user slice... "
if [ -d "/sys/fs/cgroup/user.slice" ]; then
    echo -e "$PASS user.slice exists"
else
    echo -e "$WARN user.slice not found"
    echo "   This is normal if no user sessions are active"
    WARNINGS=$((WARNINGS + 1))
fi

# =============================================================================
# Check 4: systemd --user session active
# =============================================================================

echo -n "4. Checking systemd --user session... "
if systemctl --user status >/dev/null 2>&1; then
    echo -e "$PASS systemd --user session active"
else
    echo -e "$WARN systemd --user session not active"
    echo "   Run: systemctl --user start default.target"
    echo "   Or enable lingering: loginctl enable-linger $USER"
    WARNINGS=$((WARNINGS + 1))
fi

# =============================================================================
# Check 5: systemd version (cpuset delegation requires 244+)
# =============================================================================

echo -n "5. Checking systemd version... "
SYSTEMD_VERSION=$(systemctl --version 2>/dev/null | head -1 | awk '{print $2}' || echo "0")
if [ "$SYSTEMD_VERSION" -ge 244 ] 2>/dev/null; then
    echo -e "$PASS systemd $SYSTEMD_VERSION (244+ required for cpuset delegation)"
else
    echo -e "$FAIL systemd $SYSTEMD_VERSION is too old"
    echo "   cpuset delegation requires systemd 244 or later"
    ERRORS=$((ERRORS + 1))
fi

# =============================================================================
# Check 6: cpuset DELEGATION to user session (CRITICAL)
# =============================================================================

echo -n "6. Checking cpuset delegation... "
USER_CGROUP="/sys/fs/cgroup/user.slice/user-$(id -u).slice/user@$(id -u).service"

if [ -f "$USER_CGROUP/cgroup.controllers" ]; then
    if grep -q "cpuset" "$USER_CGROUP/cgroup.controllers" 2>/dev/null; then
        echo -e "$PASS cpuset is delegated to user session"
    else
        echo -e "$FAIL cpuset NOT delegated to user session"
        echo ""
        echo "   CRITICAL: cpuset delegation is required for CPU isolation!"
        echo ""
        echo "   To fix, run:"
        echo "     sudo ./scripts/setup_cpuset_delegation.sh"
        echo "     # Then logout and login again"
        echo ""
        ERRORS=$((ERRORS + 1))
    fi
else
    echo -e "$WARN Cannot check delegation (user cgroup not found)"
    echo "   Path: $USER_CGROUP"
    WARNINGS=$((WARNINGS + 1))
fi

# =============================================================================
# Check 7: Can enable cpuset in subtree_control
# =============================================================================

echo -n "7. Testing cpuset enable in subtree_control... "
if [ -f "$USER_CGROUP/cgroup.subtree_control" ]; then
    # Check if already enabled
    if grep -q "cpuset" "$USER_CGROUP/cgroup.subtree_control" 2>/dev/null; then
        echo -e "$PASS cpuset already enabled in subtree_control"
    else
        # Try to enable
        if echo "+cpuset" > "$USER_CGROUP/cgroup.subtree_control" 2>/dev/null; then
            echo -e "$PASS Can enable cpuset in subtree_control"
        else
            echo -e "$FAIL Cannot enable cpuset in subtree_control"
            echo "   This may be due to:"
            echo "   - cpuset not delegated (see check 6)"
            echo "   - 'no internal process' constraint"
            echo "   Try: logout and login again"
            ERRORS=$((ERRORS + 1))
        fi
    fi
else
    echo -e "$WARN subtree_control not found"
    WARNINGS=$((WARNINGS + 1))
fi

# =============================================================================
# Check 8: Test actual cgroup creation with cpuset
# =============================================================================

echo -n "8. Testing cgroup creation with cpuset... "
TEST_CGROUP="$USER_CGROUP/ldos_test_$$"

# Cleanup any previous test cgroup
rmdir "$TEST_CGROUP" 2>/dev/null || true

# Try to create cgroup and set cpuset
CGROUP_TEST_PASSED=false
if mkdir "$TEST_CGROUP" 2>/dev/null; then
    if [ -f "$TEST_CGROUP/cpuset.cpus" ]; then
        # cpuset.mems must be set before cpuset.cpus on cgroups v2
        TEST_MEMS=$(cat "$USER_CGROUP/cpuset.mems.effective" 2>/dev/null || cat "$USER_CGROUP/cpuset.mems" 2>/dev/null || echo "0")
        if [ -f "$TEST_CGROUP/cpuset.mems" ]; then
            echo "$TEST_MEMS" > "$TEST_CGROUP/cpuset.mems" 2>/dev/null || true
        fi

        # Pick the first CPU from the parent's effective set (don't assume CPU 0 is allowed)
        TEST_CPU_SET=$(cat "$USER_CGROUP/cpuset.cpus.effective" 2>/dev/null || cat "$USER_CGROUP/cpuset.cpus" 2>/dev/null || echo "0")
        TEST_CPU_FIRST="${TEST_CPU_SET%%,*}"
        if [[ "$TEST_CPU_FIRST" == *-* ]]; then
            TEST_CPU="${TEST_CPU_FIRST%%-*}"
        else
            TEST_CPU="$TEST_CPU_FIRST"
        fi
        [ -z "$TEST_CPU" ] && TEST_CPU="0"

        if echo "$TEST_CPU" > "$TEST_CGROUP/cpuset.cpus" 2>/dev/null; then
            CGROUP_TEST_PASSED=true
            echo -e "$PASS cgroup creation with cpuset works"
        else
            echo -e "$FAIL Cannot write to cpuset.cpus"
        fi
    else
        echo -e "$FAIL cpuset.cpus not found in test cgroup"
        echo "   cpuset controller may not be enabled in subtree_control"
    fi
    rmdir "$TEST_CGROUP" 2>/dev/null || true
else
    echo -e "$FAIL Cannot create test cgroup"
fi

if [ "$CGROUP_TEST_PASSED" = false ]; then
    ERRORS=$((ERRORS + 1))
fi

# =============================================================================
# Check 9: CPU count
# =============================================================================

echo ""
echo -n "9. Checking CPU count... "
if command -v nproc >/dev/null 2>&1; then
    NUM_CPUS=$(nproc)
elif command -v getconf >/dev/null 2>&1; then
    NUM_CPUS=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo "0")
elif command -v sysctl >/dev/null 2>&1; then
    NUM_CPUS=$(sysctl -n hw.ncpu 2>/dev/null || echo "0")
else
    NUM_CPUS=0
fi
echo -e "$INFO System has $NUM_CPUS CPUs available"

if [ "$NUM_CPUS" -lt 4 ]; then
    echo -e "   $WARN Less than 4 CPUs - limited isolation possible"
    echo "   Recommended: At least 4 CPUs (2 for Gazebo, 2+ for ROS)"
    WARNINGS=$((WARNINGS + 1))
else
    ROS_CPUS_START=$((NUM_CPUS - 2))
    ROS_CPUS_END=$((NUM_CPUS - 1))
    GAZEBO_CPUS_END=$((NUM_CPUS - 3))
    echo "   Suggested split for 2-CPU ROS restriction:"
    echo "     Gazebo:    CPUs 0-$GAZEBO_CPUS_END"
    echo "     ROS Stack: CPUs $ROS_CPUS_START-$ROS_CPUS_END (LIMITED)"
fi

# =============================================================================
# Check 10: systemd-run with AllowedCPUs (legacy check)
# =============================================================================

echo ""
echo -n "10. Testing systemd-run AllowedCPUs... "
if systemd-run --user --scope -p AllowedCPUs=0 -- true 2>/dev/null; then
    echo -e "$PASS systemd-run AllowedCPUs works"
else
    echo -e "$WARN systemd-run AllowedCPUs failed"
    echo "   This is OK - we use direct cgroup manipulation instead"
    WARNINGS=$((WARNINGS + 1))
fi

# =============================================================================
# Check 11: Delegation config file exists
# =============================================================================

echo ""
echo -n "11. Checking delegation config file... "
DELEGATE_CONF="/etc/systemd/system/user@.service.d/delegate.conf"
if [ -f "$DELEGATE_CONF" ]; then
    if grep -q "cpuset" "$DELEGATE_CONF" 2>/dev/null; then
        echo -e "$PASS delegate.conf exists with cpuset"
    else
        echo -e "$WARN delegate.conf exists but doesn't include cpuset"
        echo "   Run: sudo ./scripts/setup_cpuset_delegation.sh"
        WARNINGS=$((WARNINGS + 1))
    fi
else
    echo -e "$WARN delegate.conf not found"
    echo "   Run: sudo ./scripts/setup_cpuset_delegation.sh"
    WARNINGS=$((WARNINGS + 1))
fi

# =============================================================================
# Summary
# =============================================================================

echo ""
echo "=========================================="
echo "  Summary"
echo "=========================================="
echo ""

if [ "$ERRORS" -eq 0 ] && [ "$WARNINGS" -eq 0 ]; then
    echo -e "${GREEN}All checks passed! cpuset isolation is fully supported.${NC}"
    echo ""
    echo "You can now run:"
    echo "  make sweep_cpuset NUM_TRIALS=5"
    exit 0
elif [ "$ERRORS" -eq 0 ]; then
    echo -e "${YELLOW}Passed with $WARNINGS warning(s). cpuset isolation should work.${NC}"
    echo ""
    echo "You can try running:"
    echo "  make sweep_cpuset NUM_TRIALS=5"
    exit 0
else
    echo -e "${RED}$ERRORS critical error(s), $WARNINGS warning(s).${NC}"
    echo ""
    echo "cpuset isolation will NOT work correctly."
    echo ""
    echo "Most likely fix:"
    echo "  sudo ./scripts/setup_cpuset_delegation.sh"
    echo "  # Then logout and login again"
    echo ""
    echo "After fixing, run this check again:"
    echo "  ./scripts/check_cpuset_support.sh"
    exit 1
fi

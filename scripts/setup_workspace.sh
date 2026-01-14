#!/bin/bash
###############################################################################
# setup_workspace.sh - Workspace setup for LDOS tracing harness
#
# This script builds the workspace and verifies dependencies.
# For fresh CloudLab nodes, run bootstrap_cloudlab.sh first.
#
# Usage:
#   ./scripts/setup_workspace.sh          # Normal setup
#   ./scripts/setup_workspace.sh --check  # Check dependencies only
###############################################################################

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()  { echo -e "${BLUE}[STEP]${NC} $1"; }

echo "=== LDOS Manipulation Tracing Harness Setup ==="
echo "Workspace: $WS_ROOT"
echo ""

###############################################################################
# Check if running on fresh system (needs bootstrap)
###############################################################################

check_needs_bootstrap() {
    local needs_bootstrap=false

    # Check if ROS 2 is installed
    if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
        log_error "ROS 2 Jazzy not found at /opt/ros/jazzy/"
        needs_bootstrap=true
    fi

    # Check if Gazebo is installed
    if ! command -v gz &>/dev/null; then
        log_error "Gazebo not found"
        needs_bootstrap=true
    fi

    # Check if LTTng is installed
    if ! command -v lttng &>/dev/null; then
        log_error "LTTng not found"
        needs_bootstrap=true
    fi

    if [ "$needs_bootstrap" = true ]; then
        echo ""
        log_error "Required dependencies not found!"
        echo ""
        echo "For fresh CloudLab/Ubuntu 24.04 systems, run the bootstrap script first:"
        echo ""
        echo "    ./scripts/bootstrap_cloudlab.sh"
        echo ""
        echo "Or use make:"
        echo ""
        echo "    make bootstrap"
        echo ""
        exit 1
    fi
}

###############################################################################
# Source ROS 2
###############################################################################

setup_ros_env() {
    if [ -z "${ROS_DISTRO:-}" ]; then
        log_info "Sourcing ROS 2 Jazzy..."
        # Disable -u temporarily for ROS setup scripts (they use unbound variables)
        set +u
        source /opt/ros/jazzy/setup.bash
        set -u
    fi

    if [ "$ROS_DISTRO" != "jazzy" ]; then
        log_warn "Expected ROS 2 Jazzy, got $ROS_DISTRO"
    fi

    log_info "ROS 2 $ROS_DISTRO environment active"
}

###############################################################################
# Check Required ROS 2 Packages
###############################################################################

check_ros_packages() {
    log_step "Checking required ROS 2 packages..."

    local REQUIRED_PKGS=(
        "ros_gz_sim"
        "ros_gz_bridge"
        "moveit"
        "moveit_ros_move_group"
        "controller_manager"
        "joint_trajectory_controller"
        "joint_state_broadcaster"
        "ros2trace"
        "tracetools"
    )

    local missing=0
    for pkg in "${REQUIRED_PKGS[@]}"; do
        if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
            echo -e "  [${GREEN}OK${NC}] $pkg"
        else
            echo -e "  [${RED}MISSING${NC}] $pkg"
            missing=1
        fi
    done

    if [ $missing -eq 1 ]; then
        log_warn "Some ROS 2 packages missing. Install with:"
        echo "  sudo apt install ros-jazzy-ros-gz ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros2-tracing"
        echo ""
        log_warn "Or run: ./scripts/bootstrap_cloudlab.sh"
        return 1
    fi

    return 0
}

###############################################################################
# Check LTTng Configuration
###############################################################################

check_lttng() {
    log_step "Checking LTTng configuration..."

    if command -v lttng &>/dev/null; then
        echo -e "  [${GREEN}OK${NC}] lttng command found ($(lttng --version 2>/dev/null | head -1))"
    else
        log_error "LTTng not found. Install with: sudo apt install lttng-tools liblttng-ust-dev"
        return 1
    fi

    # Check tracing group membership
    if groups | grep -q tracing; then
        echo -e "  [${GREEN}OK${NC}] User in 'tracing' group"
    else
        log_warn "User not in 'tracing' group"
        echo "  Add with: sudo usermod -aG tracing \$USER"
        echo "  Then log out and back in"
    fi

    # Test if we can create a session
    local test_session="ldos_test_$$"
    if lttng create "$test_session" --output=/tmp/"$test_session" &>/dev/null; then
        lttng destroy "$test_session" &>/dev/null || true
        rm -rf /tmp/"$test_session" 2>/dev/null || true
        echo -e "  [${GREEN}OK${NC}] LTTng session creation works"
    else
        log_warn "Could not create LTTng session. May need to:"
        echo "  1. Add user to tracing group: sudo usermod -aG tracing \$USER"
        echo "  2. Log out and back in"
    fi

    return 0
}

###############################################################################
# Check Python Dependencies
###############################################################################

check_python() {
    log_step "Checking Python dependencies..."

    local REQUIRED_PYTHON=(
        "pandas"
        "numpy"
        "matplotlib"
        "scipy"
        "yaml:pyyaml"
    )

    local missing_pkgs=()
    for spec in "${REQUIRED_PYTHON[@]}"; do
        local module="${spec%%:*}"
        local pkg="${spec#*:}"
        if python3 -c "import $module" 2>/dev/null; then
            echo -e "  [${GREEN}OK${NC}] $module"
        else
            echo -e "  [${RED}MISSING${NC}] $module"
            missing_pkgs+=("$pkg")
        fi
    done

    # Check babeltrace2
    if python3 -c "import bt2" 2>/dev/null; then
        echo -e "  [${GREEN}OK${NC}] babeltrace2"
    else
        echo -e "  [${YELLOW}WARN${NC}] babeltrace2 (optional, for trace analysis)"
        missing_pkgs+=("babeltrace2")
    fi

    if [ ${#missing_pkgs[@]} -gt 0 ]; then
        log_warn "Some Python packages missing. Install with:"
        echo "  pip3 install --user ${missing_pkgs[*]}"
    fi

    return 0
}

###############################################################################
# Check Load Testing Tools
###############################################################################

check_load_tools() {
    log_step "Checking load testing tools..."

    if command -v stress-ng &>/dev/null; then
        echo -e "  [${GREEN}OK${NC}] stress-ng"
    else
        log_warn "stress-ng not found. Install with: sudo apt install stress-ng"
    fi

    return 0
}

###############################################################################
# Build Workspace
###############################################################################

build_workspace() {
    log_step "Building ROS 2 workspace..."

    cd "$WS_ROOT"

    # Make scripts executable
    chmod +x scripts/*.sh 2>/dev/null || true
    chmod +x scripts/*.py 2>/dev/null || true
    chmod +x src/ldos_harness/scripts/*.py 2>/dev/null || true

    # Install rosdep dependencies (if rosdep available)
    if command -v rosdep &>/dev/null; then
        log_info "Installing rosdep dependencies..."
        rosdep install --from-paths src --ignore-src -y --rosdistro=jazzy 2>/dev/null || true
    fi

    # Build with colcon
    log_info "Running colcon build..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

    if [ $? -eq 0 ]; then
        log_info "Build successful!"
    else
        log_error "Build failed. Check errors above."
        exit 1
    fi
}

###############################################################################
# Create Output Directories
###############################################################################

create_directories() {
    log_step "Creating output directories..."

    mkdir -p "$WS_ROOT/traces"
    mkdir -p "$WS_ROOT/results/baseline"
    mkdir -p "$WS_ROOT/results/cpu_load"
    mkdir -p "$WS_ROOT/results/msg_load"
    mkdir -p "$WS_ROOT/analysis/output"

    echo -e "  [${GREEN}OK${NC}] traces/"
    echo -e "  [${GREEN}OK${NC}] results/{baseline,cpu_load,msg_load}/"
    echo -e "  [${GREEN}OK${NC}] analysis/output/"
}

###############################################################################
# Main
###############################################################################

main() {
    local check_only=false

    if [ "${1:-}" = "--check" ]; then
        check_only=true
    fi

    # First check if we need bootstrap
    check_needs_bootstrap

    # Set up ROS environment
    setup_ros_env

    # Run checks
    local all_ok=true
    check_ros_packages || all_ok=false
    check_lttng || all_ok=false
    check_python || all_ok=false
    check_load_tools || all_ok=false

    if [ "$check_only" = true ]; then
        echo ""
        if [ "$all_ok" = true ]; then
            log_info "All dependency checks passed!"
        else
            log_warn "Some dependencies missing (see above)"
        fi
        exit 0
    fi

    # Build and setup
    build_workspace
    create_directories

    echo ""
    log_info "Setup complete!"
    echo ""
    echo "Next steps:"
    echo "  1. Source the workspace:"
    echo "     source install/setup.bash"
    echo ""
    echo "  2. Run smoke test:"
    echo "     make smoke_test"
    echo ""
    echo "  3. Run experiments:"
    echo "     make run_all"
    echo ""
}

main "$@"

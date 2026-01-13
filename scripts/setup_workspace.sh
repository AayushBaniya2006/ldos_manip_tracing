#!/bin/bash
# setup_workspace.sh - One-time workspace setup for LDOS tracing harness
# Run this once after cloning or copying the harness to CloudLab

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=== LDOS Manipulation Tracing Harness Setup ==="
echo "Workspace: $WS_ROOT"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check ROS 2 Jazzy is sourced
if [ -z "${ROS_DISTRO:-}" ]; then
    log_error "ROS 2 not sourced. Run: source /opt/ros/jazzy/setup.bash"
    exit 1
fi

if [ "$ROS_DISTRO" != "jazzy" ]; then
    log_warn "Expected ROS 2 Jazzy, got $ROS_DISTRO. Proceeding anyway..."
fi

log_info "ROS 2 $ROS_DISTRO detected"

# Check required packages
log_info "Checking required packages..."

check_pkg() {
    if ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo "  [OK] $1"
        return 0
    else
        echo "  [MISSING] $1"
        return 1
    fi
}

REQUIRED_PKGS=(
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

MISSING=0
for pkg in "${REQUIRED_PKGS[@]}"; do
    if ! check_pkg "$pkg"; then
        MISSING=1
    fi
done

if [ $MISSING -eq 1 ]; then
    log_warn "Some packages missing. Install with:"
    echo "  sudo apt install ros-jazzy-ros-gz ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros2-tracing"
fi

# Check LTTng
log_info "Checking LTTng..."
if command -v lttng &> /dev/null; then
    echo "  [OK] lttng command found"
    # Check if user is in tracing group
    if groups | grep -q tracing; then
        echo "  [OK] User in 'tracing' group"
    else
        log_warn "User not in 'tracing' group. Add with: sudo usermod -aG tracing $USER"
        log_warn "Then log out and back in."
    fi
else
    log_error "LTTng not found. Install with: sudo apt install lttng-tools liblttng-ust-dev"
fi

# Check stress-ng for load testing
log_info "Checking stress-ng..."
if command -v stress-ng &> /dev/null; then
    echo "  [OK] stress-ng found"
else
    log_warn "stress-ng not found. Install with: sudo apt install stress-ng"
fi

# Install Python dependencies
log_info "Installing Python dependencies..."
pip3 install --user pandas matplotlib pyyaml babeltrace2 jupyter 2>/dev/null || {
    log_warn "Some pip packages failed. May need: pip3 install pandas matplotlib pyyaml babeltrace2"
}

# Create ROS 2 package structure
log_info "Setting up ROS 2 package..."
cd "$WS_ROOT"

# Create package.xml if not exists
if [ ! -f "src/ldos_harness/package.xml" ]; then
    log_info "Creating ldos_harness package..."
fi

# Build workspace
log_info "Building workspace..."
cd "$WS_ROOT"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    log_info "Build successful!"
else
    log_error "Build failed. Check errors above."
    exit 1
fi

# Create trace output directories
mkdir -p "$WS_ROOT/traces"
mkdir -p "$WS_ROOT/results/baseline"
mkdir -p "$WS_ROOT/results/cpu_load"
mkdir -p "$WS_ROOT/results/msg_load"
mkdir -p "$WS_ROOT/analysis/output"

log_info "Setup complete!"
echo ""
echo "Next steps:"
echo "  1. Source the workspace: source install/setup.bash"
echo "  2. Test bringup: ros2 launch ldos_harness full_stack.launch.py"
echo "  3. Run experiments: ./scripts/run_experiment_suite.sh"
echo ""

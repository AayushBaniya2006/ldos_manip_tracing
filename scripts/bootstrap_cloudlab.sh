#!/bin/bash
###############################################################################
# bootstrap_cloudlab.sh - Fully Automatic CloudLab Node Setup
#
# This script installs ALL dependencies from scratch on a fresh Ubuntu 24.04
# CloudLab node. It is completely non-interactive and idempotent.
#
# Usage:
#   ./scripts/bootstrap_cloudlab.sh
#   # or from Makefile:
#   make bootstrap
#
# What it installs:
#   - ROS 2 Jazzy (desktop-full)
#   - Gazebo Harmonic (gz-sim8)
#   - ros_gz bridge packages
#   - MoveIt 2 + ros2_control + controllers
#   - LTTng tracing (kernel + userspace)
#   - ros2_tracing + tracetools_analysis
#   - Python analysis dependencies
#   - stress-ng for load testing
#
# Runtime: ~15-25 minutes on CloudLab
###############################################################################

set -euo pipefail

# Ensure script runs as non-root but has sudo access
if [ "$EUID" -eq 0 ]; then
    echo "[ERROR] Do not run as root. Run as regular user with sudo access."
    exit 1
fi

# Test sudo access
if ! sudo -n true 2>/dev/null; then
    echo "[INFO] Testing sudo access (may prompt for password)..."
    sudo true || { echo "[ERROR] sudo access required"; exit 1; }
fi

###############################################################################
# Configuration
###############################################################################

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
LOG_FILE="${WS_ROOT}/bootstrap_$(date +%Y%m%d_%H%M%S).log"
ROS_DISTRO="jazzy"
UBUNTU_CODENAME="noble"

# Python virtual environment (PEP 668 compliant)
VENV_DIR="${WS_ROOT}/.venv"

# Track if tracing group was newly added (for better user guidance)
TRACING_GROUP_CHANGED=false

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Logging
exec > >(tee -a "$LOG_FILE") 2>&1
log_info()  { echo -e "${GREEN}[INFO]${NC}  $(date +%H:%M:%S) $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $(date +%H:%M:%S) $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $(date +%H:%M:%S) $1"; }
log_step()  { echo -e "${BLUE}[STEP]${NC}  $(date +%H:%M:%S) $1"; }

# Error handler
trap 'log_error "Script failed at line $LINENO. Check $LOG_FILE for details."; exit 1' ERR

###############################################################################
# Helper Functions
###############################################################################

# Check if a command exists
has_cmd() {
    command -v "$1" &>/dev/null
}

# Check if apt package is installed
is_installed() {
    dpkg -l "$1" 2>/dev/null | grep -q "^ii"
}

# Install apt packages (non-interactive)
apt_install() {
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q "$@"
}

# Check Ubuntu version
check_ubuntu_version() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [ "$VERSION_CODENAME" != "$UBUNTU_CODENAME" ]; then
            log_warn "Expected Ubuntu $UBUNTU_CODENAME (24.04), got $VERSION_CODENAME"
            log_warn "Proceeding anyway, but some packages may not be available"
        fi
    fi
}

###############################################################################
# Phase 1: System Preparation
###############################################################################

phase1_system_prep() {
    log_step "=== PHASE 1: System Preparation ==="

    # Set locale
    log_info "Setting locale to en_US.UTF-8..."
    sudo locale-gen en_US en_US.UTF-8 2>/dev/null || true
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 2>/dev/null || true
    export LANG=en_US.UTF-8
    export LC_ALL=en_US.UTF-8

    # Update package lists
    log_info "Updating apt package lists..."
    sudo apt-get update -q

    # Install essential tools
    log_info "Installing essential tools..."
    apt_install \
        curl \
        wget \
        gnupg \
        lsb-release \
        software-properties-common \
        apt-transport-https \
        ca-certificates \
        build-essential \
        cmake \
        git \
        python3 \
        python3-pip \
        python3-venv \
        python3-dev

    # Enable universe repository (required for python3-bt2 and other packages)
    log_info "Enabling universe repository..."
    sudo add-apt-repository -y universe 2>/dev/null || true
    sudo apt-get update -q

    log_info "Phase 1 complete."
}

###############################################################################
# Phase 2: ROS 2 Jazzy Installation
###############################################################################

phase2_ros2_install() {
    log_step "=== PHASE 2: ROS 2 Jazzy Installation ==="

    # Check if ROS 2 already installed
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        log_info "ROS 2 ${ROS_DISTRO} already installed, skipping..."
        return 0
    fi

    log_info "Adding ROS 2 apt repository..."

    # Method 1: Use ros2-apt-source (preferred for Jazzy)
    # Download and install the ROS 2 apt source package
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') || ROS_APT_SOURCE_VERSION="0.2.1"

    log_info "Using ros2-apt-source version: ${ROS_APT_SOURCE_VERSION}"

    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(lsb_release -cs)_all.deb" \
        2>/dev/null || {
        # Fallback: manual key installation
        log_warn "ros2-apt-source package not available, using manual method..."
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
            | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    }

    if [ -f /tmp/ros2-apt-source.deb ]; then
        sudo dpkg -i /tmp/ros2-apt-source.deb || sudo apt-get install -f -y
        rm -f /tmp/ros2-apt-source.deb
    fi

    # Update after adding ROS repo
    sudo apt-get update -q

    # Install ROS 2 desktop (full installation)
    log_info "Installing ROS 2 ${ROS_DISTRO} desktop (this takes ~5-10 minutes)..."
    apt_install ros-${ROS_DISTRO}-desktop

    # Install development tools
    log_info "Installing ROS 2 development tools..."
    apt_install ros-dev-tools python3-rosdep python3-colcon-common-extensions

    # Initialize rosdep if not already done
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        log_info "Initializing rosdep..."
        sudo rosdep init 2>/dev/null || true
    fi
    rosdep update --rosdistro=${ROS_DISTRO} 2>/dev/null || true

    log_info "Phase 2 complete."
}

###############################################################################
# Phase 3: Gazebo Harmonic Installation
###############################################################################

phase3_gazebo_install() {
    log_step "=== PHASE 3: Gazebo Harmonic Installation ==="

    # Check if Gazebo already installed
    if has_cmd gz && gz sim --version 2>/dev/null | grep -q "Harmonic"; then
        log_info "Gazebo Harmonic already installed, skipping..."
        return 0
    fi

    log_info "Adding Gazebo repository..."

    # Add OSRF Gazebo repository
    sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

    sudo apt-get update -q

    # Install Gazebo Harmonic
    log_info "Installing Gazebo Harmonic (this takes ~3-5 minutes)..."
    apt_install gz-harmonic

    log_info "Phase 3 complete."
}

###############################################################################
# Phase 4: ROS 2 Packages (MoveIt, ros2_control, ros_gz, tracing)
###############################################################################

phase4_ros2_packages() {
    log_step "=== PHASE 4: ROS 2 Packages Installation ==="

    # Source ROS 2 for package checks (disable -u temporarily for ROS setup scripts)
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u

    log_info "Installing MoveIt 2..."
    apt_install \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-moveit-ros-move-group \
        ros-${ROS_DISTRO}-moveit-ros-planning-interface \
        ros-${ROS_DISTRO}-moveit-ros-visualization \
        ros-${ROS_DISTRO}-moveit-planners \
        ros-${ROS_DISTRO}-moveit-simple-controller-manager

    log_info "Installing ros2_control and controllers..."
    apt_install \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-joint-state-broadcaster \
        ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-position-controllers \
        ros-${ROS_DISTRO}-velocity-controllers \
        ros-${ROS_DISTRO}-effort-controllers

    log_info "Installing ros_gz (Gazebo-ROS 2 bridge)..."
    apt_install \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-ros-gz-interfaces \
        ros-${ROS_DISTRO}-ros-gz-image

    log_info "Installing gz_ros2_control..."
    apt_install \
        ros-${ROS_DISTRO}-gz-ros2-control \
        ros-${ROS_DISTRO}-gz-ros2-control-demos || {
        log_warn "gz_ros2_control not available as apt package, may need source build"
    }

    log_info "Installing ros2_tracing..."
    apt_install \
        ros-${ROS_DISTRO}-ros2trace \
        ros-${ROS_DISTRO}-tracetools \
        ros-${ROS_DISTRO}-tracetools-launch \
        ros-${ROS_DISTRO}-tracetools-trace \
        ros-${ROS_DISTRO}-tracetools-read \
        ros-${ROS_DISTRO}-tracetools-analysis || {
        log_warn "Some tracetools packages not available, installing core only"
        apt_install ros-${ROS_DISTRO}-ros2trace ros-${ROS_DISTRO}-tracetools || true
    }

    log_info "Installing CycloneDDS (recommended middleware)..."
    apt_install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp || true

    log_info "Installing additional ROS 2 packages..."
    apt_install \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-tools \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-rqt \
        ros-${ROS_DISTRO}-rqt-graph

    log_info "Phase 4 complete."
}

###############################################################################
# Phase 5: LTTng Tracing Installation
###############################################################################

phase5_lttng_install() {
    log_step "=== PHASE 5: LTTng Tracing Installation ==="

    log_info "Installing LTTng tools and libraries..."
    apt_install \
        lttng-tools \
        liblttng-ust-dev \
        python3-lttngust

    # Install python3-bt2 separately with fallback (may have dependency issues)
    log_info "Installing python3-bt2 (babeltrace2 Python bindings)..."
    if ! apt_install python3-bt2; then
        log_warn "python3-bt2 not available via apt, trying alternative..."
        # Try installing babeltrace2 package which may include bindings
        apt_install babeltrace2 || true
    fi

    # Install kernel modules (may fail on some systems)
    log_info "Installing LTTng kernel modules..."
    apt_install lttng-modules-dkms || {
        log_warn "LTTng kernel modules failed to install (Secure Boot may be enabled)"
        log_warn "Kernel tracing will not be available, userspace tracing will work"
    }

    # Create tracing group if it doesn't exist
    if ! getent group tracing >/dev/null 2>&1; then
        log_info "Creating 'tracing' group..."
        sudo groupadd tracing || true
    fi

    # Add current user to tracing group (track if changed)
    if ! id -nG "$USER" | grep -qw tracing; then
        log_info "Adding user '$USER' to 'tracing' group..."
        sudo usermod -aG tracing "$USER" || true
        TRACING_GROUP_CHANGED=true
    else
        log_info "User '$USER' already in 'tracing' group"
    fi

    # Set permissions for LTTng
    log_info "Configuring LTTng permissions..."
    if [ -d /sys/kernel/tracing ]; then
        sudo chmod -R g+rx /sys/kernel/tracing 2>/dev/null || true
        sudo chgrp -R tracing /sys/kernel/tracing 2>/dev/null || true
    fi

    log_info "Phase 5 complete."

    # Show prominent warning if group was newly added
    if [ "$TRACING_GROUP_CHANGED" = true ]; then
        echo ""
        log_warn "============================================================"
        log_warn "IMPORTANT: Tracing group membership requires logout/login"
        log_warn "============================================================"
        log_warn ""
        log_warn "The 'tracing' group was added but won't be active until you:"
        log_warn "  1. Complete this bootstrap script"
        log_warn "  2. Log out completely (exit SSH or terminal)"
        log_warn "  3. Log back in"
        log_warn ""
        log_warn "Alternatively, after bootstrap completes, run:"
        log_warn "  newgrp tracing"
        log_warn "  # Then run: make smoke_test"
        log_warn "============================================================"
        echo ""
    fi
}

###############################################################################
# Phase 6: Python Dependencies
###############################################################################

phase6_python_deps() {
    log_step "=== PHASE 6: Python Dependencies ==="

    # Create virtual environment if it doesn't exist (PEP 668 compliant)
    # Use --system-site-packages to access system python3-bt2 package
    if [ ! -d "$VENV_DIR" ]; then
        log_info "Creating Python virtual environment at $VENV_DIR..."
        python3 -m venv --system-site-packages "$VENV_DIR"
    else
        log_info "Virtual environment already exists at $VENV_DIR"
    fi

    # Activate virtual environment
    log_info "Activating virtual environment..."
    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"

    # Upgrade pip within venv
    log_info "Upgrading pip..."
    pip install --upgrade pip

    # Install packages into venv
    log_info "Installing Python packages for analysis..."
    pip install \
        pandas \
        numpy \
        scipy \
        matplotlib \
        seaborn \
        pyyaml \
        jupyter \
        jupyterlab \
        bokeh \
        statsmodels \
        psutil

    # Handle babeltrace2 - try pip first, fall back to system package symlink
    log_info "Installing babeltrace2..."
    if pip install babeltrace2 2>/dev/null; then
        log_info "  babeltrace2 installed via pip"
    else
        log_warn "pip babeltrace2 failed, attempting system package symlink..."
        # Check if system python3-bt2 is available
        if python3 -c "import bt2" 2>/dev/null; then
            SYSTEM_BT2_PATH=$(python3 -c "import bt2; import os; print(os.path.dirname(bt2.__file__))" 2>/dev/null) || true
            VENV_SITE=$("$VENV_DIR/bin/python3" -c "import site; print(site.getsitepackages()[0])" 2>/dev/null) || true
            if [ -n "$SYSTEM_BT2_PATH" ] && [ -d "$SYSTEM_BT2_PATH" ] && [ -n "$VENV_SITE" ]; then
                ln -sf "$SYSTEM_BT2_PATH" "$VENV_SITE/bt2" 2>/dev/null || true
                # Also link _bt2 if it exists (C extension)
                SYSTEM_BT2_SO=$(python3 -c "import _bt2; print(_bt2.__file__)" 2>/dev/null) || true
                if [ -n "$SYSTEM_BT2_SO" ] && [ -f "$SYSTEM_BT2_SO" ]; then
                    ln -sf "$SYSTEM_BT2_SO" "$VENV_SITE/" 2>/dev/null || true
                fi
                log_info "  Created symlink to system bt2 package"
            fi
        else
            log_warn "  System python3-bt2 not available either"
            log_warn "  Trace analysis may not work - install manually: sudo apt install python3-bt2"
        fi
    fi

    # Create requirements.txt for reproducibility
    log_info "Saving requirements.txt..."
    pip freeze > "$WS_ROOT/requirements.txt"

    # Deactivate for now (will be sourced via ~/.bashrc)
    deactivate

    # Verify critical packages using venv python
    log_info "Verifying Python packages..."
    "$VENV_DIR/bin/python3" -c "import pandas; print(f'  pandas {pandas.__version__}')" || log_warn "pandas not available"
    "$VENV_DIR/bin/python3" -c "import numpy; print(f'  numpy {numpy.__version__}')" || log_warn "numpy not available"
    "$VENV_DIR/bin/python3" -c "import matplotlib; print(f'  matplotlib {matplotlib.__version__}')" || log_warn "matplotlib not available"
    "$VENV_DIR/bin/python3" -c "import bt2; print('  babeltrace2 OK')" || log_warn "babeltrace2 not available"

    log_info "Phase 6 complete."
    log_info "Virtual environment: $VENV_DIR"
}

###############################################################################
# Phase 7: Load Testing Tools
###############################################################################

phase7_load_tools() {
    log_step "=== PHASE 7: Load Testing Tools ==="

    log_info "Installing stress-ng for CPU load testing..."
    apt_install stress-ng

    # Verify
    stress-ng --version | head -1

    log_info "Phase 7 complete."
}

###############################################################################
# Phase 8: Environment Configuration
###############################################################################

phase8_environment() {
    log_step "=== PHASE 8: Environment Configuration ==="

    BASHRC_MARKER="# LDOS Harness Environment"

    # Check if already configured
    if grep -q "$BASHRC_MARKER" ~/.bashrc 2>/dev/null; then
        log_info "Environment already configured in ~/.bashrc"
        log_info "Removing old configuration to update..."
        # Remove old configuration block
        sed -i '/# LDOS Harness Environment/,/^$/d' ~/.bashrc 2>/dev/null || true
    fi

    log_info "Adding ROS 2 and venv environment to ~/.bashrc..."
    cat >> ~/.bashrc << 'EOF'

# LDOS Harness Environment
# Detect ROS 2 installation dynamically (supports jazzy, humble, iron, rolling)
ROS_FOUND=false
for ros_distro in jazzy humble iron rolling; do
    if [ -f "/opt/ros/${ros_distro}/setup.bash" ]; then
        source "/opt/ros/${ros_distro}/setup.bash"
        ROS_FOUND=true
        break
    fi
done
if [ "$ROS_FOUND" = false ]; then
    echo "[WARN] No ROS 2 installation found in /opt/ros/"
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Python virtual environment (auto-activate if exists)
LDOS_VENV="$HOME/ldos_manip_tracing/.venv"
if [ -d "$LDOS_VENV" ] && [ -f "$LDOS_VENV/bin/activate" ]; then
    source "$LDOS_VENV/bin/activate"
fi

# Workspace setup (if exists)
if [ -f ~/ldos_manip_tracing/install/setup.bash ]; then
    source ~/ldos_manip_tracing/install/setup.bash
fi

# LTTng environment
export LTTNG_HOME=~/.lttng

EOF

    # Source ROS 2 for current session (disable -u temporarily for ROS setup scripts)
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

    log_info "Phase 8 complete."
}

###############################################################################
# Phase 9: Build Workspace
###############################################################################

phase9_build_workspace() {
    log_step "=== PHASE 9: Building LDOS Harness Workspace ==="

    # Source ROS 2 (disable -u temporarily for ROS setup scripts)
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    set -u

    cd "$WS_ROOT"

    # Make scripts executable
    log_info "Making scripts executable..."
    chmod +x scripts/*.sh 2>/dev/null || true
    chmod +x scripts/*.py 2>/dev/null || true
    chmod +x src/ldos_harness/scripts/*.py 2>/dev/null || true

    # Create output directories
    log_info "Creating output directories..."
    mkdir -p traces results/baseline results/cpu_load results/msg_load analysis/output

    # Install rosdep dependencies
    log_info "Installing rosdep dependencies..."
    rosdep install --from-paths src --ignore-src -y --rosdistro=${ROS_DISTRO} 2>/dev/null || true

    # Build workspace
    log_info "Building workspace with colcon..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

    if [ $? -eq 0 ]; then
        log_info "Workspace build successful!"
    else
        log_error "Workspace build failed!"
        return 1
    fi

    log_info "Phase 9 complete."
}

###############################################################################
# Phase 10: Verification
###############################################################################

phase10_verify() {
    log_step "=== PHASE 10: Installation Verification ==="

    # Source ROS 2 (disable -u temporarily for ROS setup scripts)
    set +u
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source "${WS_ROOT}/install/setup.bash" 2>/dev/null || true
    set -u

    PASS=0
    FAIL=0

    verify() {
        local name="$1"
        local cmd="$2"
        echo -n "  Checking $name... "
        if eval "$cmd" >/dev/null 2>&1; then
            echo -e "${GREEN}OK${NC}"
            PASS=$((PASS + 1))
        else
            echo -e "${RED}FAILED${NC}"
            FAIL=$((FAIL + 1))
        fi
    }

    log_info "Verifying installation..."

    # Check for any ROS 2 installation
    verify "ROS 2" "[ -f /opt/ros/jazzy/setup.bash ] || [ -f /opt/ros/humble/setup.bash ]"
    verify "ros2 command" "ros2 --help"
    verify "Gazebo" "gz sim --version"
    verify "MoveIt" "ros2 pkg list | grep -q moveit"
    verify "ros2_control" "ros2 pkg list | grep -q controller_manager"
    verify "ros_gz" "ros2 pkg list | grep -q ros_gz_sim"
    verify "ros2trace" "ros2 pkg list | grep -q ros2trace"
    verify "tracetools" "ros2 pkg list | grep -q tracetools"
    verify "LTTng" "lttng --version"
    verify "stress-ng" "stress-ng --version"
    # Use venv python for package verification
    verify "Python venv" "[ -f '$VENV_DIR/bin/python3' ]"
    verify "Python pandas" "'$VENV_DIR/bin/python3' -c 'import pandas'"
    verify "Python scipy" "'$VENV_DIR/bin/python3' -c 'import scipy'"
    verify "Python matplotlib" "'$VENV_DIR/bin/python3' -c 'import matplotlib'"
    verify "Python babeltrace2" "'$VENV_DIR/bin/python3' -c 'import bt2'"
    verify "Python psutil" "'$VENV_DIR/bin/python3' -c 'import psutil'"
    verify "ldos_harness package" "ros2 pkg list | grep -q ldos_harness"

    echo ""
    log_info "Verification complete: ${PASS} passed, ${FAIL} failed"

    if [ $FAIL -gt 0 ]; then
        log_warn "Some components failed verification. Check the log for details."
    fi

    # Check tracing group
    if groups | grep -q tracing; then
        echo -e "  Tracing group: ${GREEN}Active${NC}"
    else
        echo -e "  Tracing group: ${YELLOW}Pending (requires logout/login)${NC}"
    fi
}

###############################################################################
# Main Execution
###############################################################################

main() {
    echo ""
    echo "============================================================"
    echo "  LDOS Manipulation Tracing Harness - CloudLab Bootstrap"
    echo "============================================================"
    echo ""
    echo "Workspace: $WS_ROOT"
    echo "Log file:  $LOG_FILE"
    echo ""

    check_ubuntu_version

    START_TIME=$(date +%s)

    phase1_system_prep
    phase2_ros2_install
    phase3_gazebo_install
    phase4_ros2_packages
    phase5_lttng_install
    phase6_python_deps
    phase7_load_tools
    phase8_environment
    phase9_build_workspace
    phase10_verify

    END_TIME=$(date +%s)
    DURATION=$((END_TIME - START_TIME))

    echo ""
    echo "============================================================"
    echo -e "  ${GREEN}Bootstrap Complete!${NC}"
    echo "============================================================"
    echo ""
    echo "  Duration: $((DURATION / 60)) minutes $((DURATION % 60)) seconds"
    echo "  Log file: $LOG_FILE"
    echo "  Python venv: $VENV_DIR"
    echo ""
    if [ "$TRACING_GROUP_CHANGED" = true ]; then
        echo -e "  ${YELLOW}IMPORTANT: Tracing group requires logout/login${NC}"
        echo ""
        echo "  Next steps (RECOMMENDED):"
        echo "    1. exit                  # Log out completely"
        echo "    2. # SSH/login back in"
        echo "    3. cd ~/ldos_manip_tracing"
        echo "    4. source ~/.bashrc      # Activates ROS 2 + venv"
        echo "    5. make smoke_test       # Verify everything works"
        echo ""
        echo "  Alternative (no logout required):"
        echo "    newgrp tracing           # Activate tracing group"
        echo "    source ~/.bashrc         # Activate ROS 2 + venv"
        echo "    make smoke_test"
    else
        echo "  Next steps:"
        echo "    1. source ~/.bashrc      # Activate ROS 2 + venv"
        echo "    2. cd ~/ldos_manip_tracing"
        echo "    3. make smoke_test       # Verify everything works"
    fi
    echo ""
}

# Run main
main "$@"

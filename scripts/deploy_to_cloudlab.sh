#!/bin/bash
# deploy_to_cloudlab.sh - Transfer harness to CloudLab node
# Usage: ./scripts/deploy_to_cloudlab.sh user@node.cloudlab.us

set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Usage: $0 <cloudlab_ssh_target>"
    echo "Example: $0 aayush@node0.ldos-exp.MyProject.utah.cloudlab.us"
    exit 1
fi

CLOUDLAB_TARGET="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=== Deploying LDOS Harness to CloudLab ==="
echo "Target: $CLOUDLAB_TARGET"
echo "Source: $WS_ROOT"
echo ""

# Create tarball
TARBALL="/tmp/ldos_harness_$(date +%Y%m%d_%H%M%S).tar.gz"

echo "Creating tarball..."
cd "$WS_ROOT"
tar -czvf "$TARBALL" \
    --exclude='traces/*' \
    --exclude='results/*/*' \
    --exclude='analysis/output/*' \
    --exclude='build' \
    --exclude='install' \
    --exclude='log' \
    --exclude='.git' \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    .

echo ""
echo "Tarball size: $(du -h "$TARBALL" | cut -f1)"
echo ""

# Transfer
echo "Transferring to CloudLab..."
scp "$TARBALL" "${CLOUDLAB_TARGET}:~/ldos_harness.tar.gz"

# Extract and setup on remote
echo ""
echo "Setting up on CloudLab..."
ssh "$CLOUDLAB_TARGET" << 'REMOTE_SCRIPT'
set -e

echo "=== CloudLab Setup ==="

# Create workspace
mkdir -p ~/ldos_manip_tracing
cd ~/ldos_manip_tracing

# Extract
tar -xzf ~/ldos_harness.tar.gz
rm ~/ldos_harness.tar.gz

# Make scripts executable
chmod +x scripts/*.sh
chmod +x src/ldos_harness/scripts/*.py
chmod +x analysis/*.py

# Source ROS 2
source /opt/ros/jazzy/setup.bash 2>/dev/null || {
    echo "WARNING: ROS 2 Jazzy not found at /opt/ros/jazzy"
    echo "You may need to install it first."
}

# Build
echo ""
echo "Building workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "=== Deployment Complete ==="
echo ""
echo "Next steps on CloudLab:"
echo "  cd ~/ldos_manip_tracing"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source install/setup.bash"
echo "  make smoke_test"
echo "  make run_all NUM_TRIALS=10"
REMOTE_SCRIPT

# Cleanup local tarball
rm "$TARBALL"

echo ""
echo "=== Deployment Complete ==="
echo ""
echo "SSH to CloudLab and run:"
echo "  ssh $CLOUDLAB_TARGET"
echo "  cd ~/ldos_manip_tracing"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source install/setup.bash"
echo "  make smoke_test"

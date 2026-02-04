#!/bin/bash
# setup_cpuset_delegation.sh - Enable cgroups v2 cpuset delegation for user sessions
#
# This script configures systemd to delegate the cpuset controller to user sessions,
# which is required for non-root cgroup-based CPU isolation.
#
# Usage: sudo ./setup_cpuset_delegation.sh
#
# After running this script, you MUST:
#   1. Run: sudo systemctl daemon-reload
#   2. Logout and login again (or reboot)
#
# Reference: https://rootlesscontaine.rs/getting-started/common/cgroup2/

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[SETUP]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[SETUP]${NC} $1"; }
log_error() { echo -e "${RED}[SETUP ERROR]${NC} $1"; }

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    log_error "This script must be run as root (sudo)"
    echo "Usage: sudo $0"
    exit 1
fi

log_info "Setting up cgroups v2 cpuset delegation for user sessions..."

# Check if cgroups v2 is available
if [ ! -f /sys/fs/cgroup/cgroup.controllers ]; then
    log_error "cgroups v2 not detected!"
    log_error "This system may be using cgroups v1 or hybrid mode."
    log_error "Ubuntu 24.04 should have cgroups v2 by default."
    exit 1
fi

log_info "cgroups v2 detected"

# Check if cpuset controller is available at root level
if ! grep -q "cpuset" /sys/fs/cgroup/cgroup.controllers; then
    log_error "cpuset controller not available in root cgroup!"
    log_error "This may require kernel configuration changes."
    exit 1
fi

log_info "cpuset controller is available"

# Check systemd version (cpuset delegation requires 244+)
SYSTEMD_VERSION=$(systemctl --version | head -1 | awk '{print $2}')
if [ "$SYSTEMD_VERSION" -lt 244 ]; then
    log_error "systemd version $SYSTEMD_VERSION is too old"
    log_error "cpuset delegation requires systemd 244 or later"
    exit 1
fi

log_info "systemd version $SYSTEMD_VERSION (244+ required)"

# Create the configuration directory
DELEGATE_DIR="/etc/systemd/system/user@.service.d"
DELEGATE_CONF="$DELEGATE_DIR/delegate.conf"

log_info "Creating delegation configuration..."

mkdir -p "$DELEGATE_DIR"

# Check if config already exists
if [ -f "$DELEGATE_CONF" ]; then
    log_warn "Configuration file already exists: $DELEGATE_CONF"
    log_info "Current contents:"
    cat "$DELEGATE_CONF"
    echo ""

    # Check if cpuset is already delegated
    if grep -q "cpuset" "$DELEGATE_CONF"; then
        log_info "cpuset is already in delegation config"
    else
        log_warn "cpuset NOT in existing config, will update"
    fi

    # Backup existing config
    cp "$DELEGATE_CONF" "${DELEGATE_CONF}.backup.$(date +%Y%m%d_%H%M%S)"
    log_info "Backed up existing config"
fi

# Write the delegation configuration
cat > "$DELEGATE_CONF" << 'EOF'
# Enable cgroup v2 controller delegation for user sessions
# This allows non-root users to create cgroups with cpu, cpuset, io, memory, and pids controllers
#
# Created by: setup_cpuset_delegation.sh
# Reference: https://rootlesscontaine.rs/getting-started/common/cgroup2/
#
# After modifying this file:
#   1. Run: sudo systemctl daemon-reload
#   2. Logout and login again (or reboot)

[Service]
Delegate=cpu cpuset io memory pids
EOF

log_info "Created delegation config: $DELEGATE_CONF"

# Show the config
log_info "Configuration contents:"
cat "$DELEGATE_CONF"
echo ""

# Reload systemd
log_info "Reloading systemd daemon..."
systemctl daemon-reload

log_info "Systemd daemon reloaded"

# Verify the configuration was applied
echo ""
log_info "=== VERIFICATION ==="

# Check if user slice exists (may not if no user logged in via systemd)
if [ -d /sys/fs/cgroup/user.slice ]; then
    log_info "user.slice exists"
else
    log_warn "user.slice not found (normal if no user sessions active)"
fi

echo ""
log_info "=== NEXT STEPS ==="
echo ""
echo "1. You MUST logout and login again (or reboot) for changes to take effect"
echo ""
echo "2. After re-login, verify with:"
echo "   cat /sys/fs/cgroup/user.slice/user-\$(id -u).slice/user@\$(id -u).service/cgroup.controllers"
echo "   # Should include: cpuset cpu io memory pids"
echo ""
echo "3. Test cpuset isolation:"
echo "   ./scripts/check_cpuset_support.sh"
echo ""

log_info "Setup complete! Please logout and login again."

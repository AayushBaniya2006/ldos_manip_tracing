#!/bin/bash
# start_trace.sh - Start LTTng tracing session for LDOS experiments
# Usage: ./start_trace.sh <session_name> [output_dir]
#
# This script sets up and starts an LTTng session with ROS 2 and kernel tracepoints.

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

SESSION_NAME="${1:-ldos_trace_$(date +%Y%m%d_%H%M%S)}"
OUTPUT_DIR="${2:-$WS_ROOT/traces/$SESSION_NAME}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[TRACE]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check LTTng is available
if ! command -v lttng &> /dev/null; then
    log_error "lttng command not found. Install with: sudo apt install lttng-tools"
    exit 1
fi

# Check if user is in tracing group (for kernel tracing)
if ! groups | grep -q tracing; then
    log_warn "User not in 'tracing' group. Kernel events may not be available."
    log_warn "Add with: sudo usermod -aG tracing $USER && newgrp tracing"
fi

# Check if session already exists
if lttng list | grep -q "^$SESSION_NAME "; then
    log_warn "Session $SESSION_NAME already exists. Destroying..."
    lttng destroy "$SESSION_NAME" 2>/dev/null || true
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"
log_info "Trace output: $OUTPUT_DIR"

# Create session
log_info "Creating LTTng session: $SESSION_NAME"
lttng create "$SESSION_NAME" --output="$OUTPUT_DIR"

# Enable ROS 2 userspace events
log_info "Enabling ROS 2 tracepoints..."
lttng enable-event --userspace 'ros2:*' || log_warn "Some ros2 events may not be available"

# Try to enable kernel events (may fail without permissions)
log_info "Enabling kernel scheduler events..."
if lttng enable-event --kernel sched_switch,sched_wakeup,sched_process_fork 2>/dev/null; then
    log_info "Kernel events enabled"
else
    log_warn "Kernel events not available (need root or tracing group)"
fi

# Add context fields
log_info "Adding context fields..."
lttng add-context --userspace --type=vpid --type=vtid --type=procname 2>/dev/null || true

# Start tracing
log_info "Starting trace..."
lttng start

# Verify session is active
if ! lttng list "$SESSION_NAME" 2>/dev/null | grep -q "Tracing session.*ACTIVE"; then
    log_error "Failed to start tracing session"
    lttng destroy "$SESSION_NAME" 2>/dev/null || true
    exit 1
fi

# Save session info
echo "$SESSION_NAME" > "$OUTPUT_DIR/.session_name"
echo "$(date -Iseconds)" > "$OUTPUT_DIR/.trace_start_time"

log_info "Tracing started and verified. Session: $SESSION_NAME"
log_info "To stop: ./scripts/stop_trace.sh $SESSION_NAME"

# Output for scripting
echo "$SESSION_NAME"

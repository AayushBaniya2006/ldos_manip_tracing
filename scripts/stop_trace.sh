#!/bin/bash
# stop_trace.sh - Stop LTTng tracing session
# Usage: ./stop_trace.sh <session_name>

set -euo pipefail

SESSION_NAME="${1:-}"

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[TRACE]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

if [ -z "$SESSION_NAME" ]; then
    # Try to find active session
    SESSION_NAME=$(lttng list | grep -E "^ldos_trace" | head -1 | awk '{print $1}' || true)
    if [ -z "$SESSION_NAME" ]; then
        log_error "No session name provided and no active ldos_trace session found"
        exit 1
    fi
    log_info "Found active session: $SESSION_NAME"
fi

# Check session exists
if ! lttng list | grep -q "^$SESSION_NAME "; then
    log_error "Session $SESSION_NAME not found"
    exit 1
fi

# Stop and destroy
log_info "Stopping trace session: $SESSION_NAME"
lttng stop "$SESSION_NAME"

log_info "Destroying session..."
lttng destroy "$SESSION_NAME"

log_info "Trace session $SESSION_NAME stopped and saved"

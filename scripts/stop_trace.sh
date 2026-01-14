#!/bin/bash
# stop_trace.sh - Stop LTTng tracing session
# Usage: ./stop_trace.sh [session_name] [trace_dir]
#
# If no session_name is provided, looks for:
# 1. .session_name file in the trace_dir (if provided)
# 2. .session_name file in the most recent traces/* directory
# 3. Active ldos_trace* session in lttng list

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

SESSION_NAME="${1:-}"
TRACE_DIR="${2:-}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[TRACE]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Function to find session name from .session_name file
find_session_from_file() {
    local dir="$1"
    local session_file="$dir/.session_name"
    if [[ -f "$session_file" ]]; then
        cat "$session_file"
        return 0
    fi
    return 1
}

if [[ -z "$SESSION_NAME" ]]; then
    # Strategy 1: Look in specified trace directory
    if [[ -n "$TRACE_DIR" ]] && [[ -d "$TRACE_DIR" ]]; then
        SESSION_NAME=$(find_session_from_file "$TRACE_DIR" || true)
        if [[ -n "$SESSION_NAME" ]]; then
            log_info "Found session from trace directory: $SESSION_NAME"
        fi
    fi

    # Strategy 2: Look in most recent traces/* directory
    if [[ -z "$SESSION_NAME" ]]; then
        TRACES_BASE="$WS_ROOT/traces"
        if [[ -d "$TRACES_BASE" ]]; then
            # Find most recently modified directory with .session_name
            for trace_subdir in $(ls -t "$TRACES_BASE" 2>/dev/null || true); do
                candidate="$TRACES_BASE/$trace_subdir"
                if [[ -d "$candidate" ]]; then
                    SESSION_NAME=$(find_session_from_file "$candidate" || true)
                    if [[ -n "$SESSION_NAME" ]]; then
                        log_info "Found session from recent trace: $SESSION_NAME"
                        break
                    fi
                fi
            done
        fi
    fi

    # Strategy 3: Fall back to grepping lttng list (less reliable)
    if [[ -z "$SESSION_NAME" ]]; then
        SESSION_NAME=$(lttng list 2>/dev/null | grep -E "^ldos_trace" | head -1 | awk '{print $1}' || true)
        if [[ -n "$SESSION_NAME" ]]; then
            log_warn "Found active session via lttng list (no .session_name file): $SESSION_NAME"
        fi
    fi

    # Final check
    if [[ -z "$SESSION_NAME" ]]; then
        log_error "No session name provided and no active ldos_trace session found"
        log_info "Usage: ./stop_trace.sh <session_name> [trace_dir]"
        exit 1
    fi
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

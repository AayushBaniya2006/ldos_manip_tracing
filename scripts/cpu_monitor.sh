#!/bin/bash
# cpu_monitor.sh - Background CPU usage monitor
#
# Usage:
#   ./cpu_monitor.sh start <output_file> [interval_seconds]
#   ./cpu_monitor.sh stop
#
# Logs per-process CPU usage to CSV for analysis

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
PIDFILE="/tmp/ldos_cpu_monitor.pid"

start_monitor() {
    local OUTPUT_FILE="${1:-$WS_ROOT/results/cpu_usage.csv}"
    local INTERVAL="${2:-1}"

    # Create output directory
    mkdir -p "$(dirname "$OUTPUT_FILE")"

    # Write CSV header
    echo "timestamp,process,pid,cpu_percent,mem_percent,cpu_id,state" > "$OUTPUT_FILE"

    echo "[CPU_MONITOR] Starting background monitor..."
    echo "[CPU_MONITOR] Output: $OUTPUT_FILE"
    echo "[CPU_MONITOR] Interval: ${INTERVAL}s"

    # Start background monitoring loop
    (
        while true; do
            TIMESTAMP=$(date +%s.%N)

            # Get CPU usage for ROS/Gazebo processes
            # Using ps with specific format for reliability
            ps -eo pid,comm,%cpu,%mem,psr,state --no-headers 2>/dev/null | \
            grep -E "(move_group|controller|gz|robot_state|ros2|benchmark|joint_)" | \
            while read -r pid comm cpu mem psr state; do
                echo "${TIMESTAMP},${comm},${pid},${cpu},${mem},${psr},${state}"
            done >> "$OUTPUT_FILE"

            sleep "$INTERVAL"
        done
    ) &

    local MONITOR_PID=$!
    echo "$MONITOR_PID" > "$PIDFILE"
    echo "[CPU_MONITOR] Monitor started (PID: $MONITOR_PID)"
}

stop_monitor() {
    if [ -f "$PIDFILE" ]; then
        local PID
        PID=$(cat "$PIDFILE")
        if kill -0 "$PID" 2>/dev/null; then
            kill "$PID" 2>/dev/null || true
            # Also kill any child processes
            pkill -P "$PID" 2>/dev/null || true
            echo "[CPU_MONITOR] Monitor stopped (PID: $PID)"
        else
            echo "[CPU_MONITOR] Monitor not running"
        fi
        rm -f "$PIDFILE"
    else
        echo "[CPU_MONITOR] No monitor running (no pidfile)"
    fi
}

status_monitor() {
    if [ -f "$PIDFILE" ]; then
        local PID
        PID=$(cat "$PIDFILE")
        if kill -0 "$PID" 2>/dev/null; then
            echo "[CPU_MONITOR] Monitor running (PID: $PID)"
            return 0
        fi
    fi
    echo "[CPU_MONITOR] Monitor not running"
    return 1
}

case "${1:-}" in
    start)
        stop_monitor 2>/dev/null || true  # Stop any existing monitor
        start_monitor "${2:-}" "${3:-1}"
        ;;
    stop)
        stop_monitor
        ;;
    status)
        status_monitor
        ;;
    *)
        echo "Usage: $0 {start|stop|status} [output_file] [interval_seconds]"
        exit 1
        ;;
esac

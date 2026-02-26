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
PROCESS_REGEX="${CPU_MONITOR_PROCESS_REGEX:-(move_group|controller|gz|gazebo|robot_state|parameter_|benchmark|joint_|ros2|create)}"

start_monitor() {
    local OUTPUT_FILE="${1:-$WS_ROOT/results/cpu_usage.csv}"
    local INTERVAL="${2:-1}"

    # Create output directory
    mkdir -p "$(dirname "$OUTPUT_FILE")"

    # Write CSV header (includes cgroup and cpus_allowed for cpuset verification)
    echo "timestamp,process,pid,cpu_percent,mem_percent,cpu_id,state,cgroup,cpus_allowed" > "$OUTPUT_FILE"

    echo "[CPU_MONITOR] Starting background monitor..."
    echo "[CPU_MONITOR] Output: $OUTPUT_FILE"
    echo "[CPU_MONITOR] Interval: ${INTERVAL}s"

    # Start background monitoring loop
    (
        while true; do
            TIMESTAMP=$(date +%s.%N)

            # Get CPU usage for ROS/Gazebo processes
            # Using ps with specific format for reliability
            while read -r pid comm cpu mem psr state; do
                [ -n "${pid:-}" ] || continue
                # Get cgroup info and CPU affinity for cpuset verification
                cgroup=$(cat /proc/$pid/cgroup 2>/dev/null | grep -oE 'ldos_[^.]+\.scope' | head -1 || echo "default")
                cpus_allowed=$(awk '/Cpus_allowed_list/{print $2}' /proc/$pid/status 2>/dev/null || echo "")
                echo "${TIMESTAMP},${comm},${pid},${cpu},${mem},${psr},${state},${cgroup},${cpus_allowed}"
            done < <(
                { ps -eo pid,comm,%cpu,%mem,psr,state --no-headers 2>/dev/null | \
                  grep -E "$PROCESS_REGEX"; } || true
            ) >> "$OUTPUT_FILE"

            sleep "$INTERVAL"
        done
    ) &

    local MONITOR_PID=$!
    echo "$MONITOR_PID" > "$PIDFILE"
    echo "[CPU_MONITOR] Monitor started (PID: $MONITOR_PID)"
}

# Verify processes are in correct cgroup (helper function for debugging)
verify_cgroup() {
    local pid=$1
    local expected_cgroup=$2
    local actual=$(cat /proc/$pid/cgroup 2>/dev/null | grep -oE 'ldos_[^.]+\.scope' || echo "default")
    if [ "$actual" = "$expected_cgroup" ]; then
        echo "[CPU_MONITOR] PID $pid: CORRECT cgroup ($actual)"
        return 0
    else
        echo "[CPU_MONITOR] PID $pid: WRONG cgroup (expected: $expected_cgroup, actual: $actual)"
        return 1
    fi
}

# Verify CPU pinning - check that ROS processes are on expected CPUs
# Usage: verify_cpu_pinning "38-39"
verify_cpu_pinning() {
    local expected_cpus="$1"
    local mismatches=0
    local checked=0

    echo "[CPU_MONITOR] Verifying CPU pinning (expected: $expected_cpus)"
    echo ""

    # Parse expected CPU range into array
    local -a expected_array=()
    if [[ "$expected_cpus" =~ ^([0-9]+)-([0-9]+)$ ]]; then
        local start=${BASH_REMATCH[1]}
        local end=${BASH_REMATCH[2]}
        for ((i=start; i<=end; i++)); do
            expected_array+=("$i")
        done
    else
        # Comma-separated or single CPU
        IFS=',' read -ra expected_array <<< "$expected_cpus"
    fi

    # Check ROS processes (use process substitution to avoid subshell variable loss)
    while read -r pid comm psr; do
        [ -n "${pid:-}" ] || continue
        checked=$((checked + 1))
        local is_expected=false

        for cpu in "${expected_array[@]}"; do
            if [ "$psr" = "$cpu" ]; then
                is_expected=true
                break
            fi
        done

        if [ "$is_expected" = true ]; then
            echo "[OK]   PID $pid ($comm) on CPU $psr"
        else
            echo "[FAIL] PID $pid ($comm) on CPU $psr - OUTSIDE expected range!"
            mismatches=$((mismatches + 1))
        fi
    done < <(
        { ps -eo pid,comm,psr --no-headers 2>/dev/null | \
          grep -E "(move_group|controller|robot_state|parameter_|ros2|joint_|create)"; } || true
    )

    echo ""
    if [ "$mismatches" -eq 0 ]; then
        echo "[CPU_MONITOR] All processes on expected CPUs"
        return 0
    else
        echo "[CPU_MONITOR] WARNING: $mismatches process(es) on unexpected CPUs!"
        return 1
    fi
}

# Quick snapshot of current CPU usage by ROS processes
snapshot() {
    echo "[CPU_MONITOR] Current ROS/Gazebo process CPU assignments:"
    echo ""
    printf "%-8s %-20s %-6s %-8s %-10s\n" "PID" "COMMAND" "CPU" "CGROUP" "STATE"
    echo "--------------------------------------------------------------"

    while read -r pid comm psr state; do
        [ -n "${pid:-}" ] || continue
        local cgroup=$(cat /proc/$pid/cgroup 2>/dev/null | grep -oE 'ldos_[^.]+\.scope' | head -1 || echo "default")
        printf "%-8s %-20s %-6s %-8s %-10s\n" "$pid" "$comm" "$psr" "$cgroup" "$state"
    done < <(
        { ps -eo pid,comm,psr,state --no-headers 2>/dev/null | \
          grep -E "$PROCESS_REGEX"; } || true
    )

    echo ""
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

# Verify that process affinity masks (Cpus_allowed_list) match expected CPUs
# Usage: verify_affinity_mask <expected_cpus> [process_regex]
verify_affinity_mask() {
    local expected_cpus="$1"
    local proc_regex="${2:-$PROCESS_REGEX}"
    local mismatches=0
    local checked=0

    echo "[CPU_MONITOR] Verifying CPU affinity masks (expected: $expected_cpus)"
    echo ""

    while read -r pid comm; do
        [ -n "${pid:-}" ] || continue
        checked=$((checked + 1))

        local actual_mask
        actual_mask=$(awk '/Cpus_allowed_list/{print $2}' /proc/"$pid"/status 2>/dev/null || echo "unknown")

        # Normalize both for comparison: "N-N" -> "N" for single CPUs
        local norm_actual="${actual_mask}"
        local norm_expected="${expected_cpus}"
        [[ "$norm_actual" =~ ^([0-9]+)-\1$ ]] && norm_actual="${BASH_REMATCH[1]}"
        [[ "$norm_expected" =~ ^([0-9]+)-\1$ ]] && norm_expected="${BASH_REMATCH[1]}"
        if [ "$norm_actual" = "$norm_expected" ]; then
            echo "[OK]   PID $pid ($comm) Cpus_allowed_list=$actual_mask"
        else
            echo "[FAIL] PID $pid ($comm) Cpus_allowed_list=$actual_mask (expected: $expected_cpus)"
            mismatches=$((mismatches + 1))
        fi
    done < <(
        { ps -eo pid,comm --no-headers 2>/dev/null | grep -E "$proc_regex"; } || true
    )

    echo ""
    echo "[CPU_MONITOR] Checked $checked processes, $mismatches affinity violations"
    [ "$mismatches" -eq 0 ]
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
    snapshot)
        snapshot
        ;;
    verify)
        if [ -z "${2:-}" ]; then
            echo "Usage: $0 verify <expected_cpus>"
            echo "Example: $0 verify 38-39"
            exit 1
        fi
        verify_cpu_pinning "$2"
        ;;
    verify_affinity)
        if [ -z "${2:-}" ]; then
            echo "Usage: $0 verify_affinity <expected_cpus> [process_regex]"
            echo "Example: $0 verify_affinity 30-31"
            exit 1
        fi
        verify_affinity_mask "$2" "${3:-}"
        ;;
    *)
        echo "Usage: $0 {start|stop|status|snapshot|verify|verify_affinity} [options]"
        echo ""
        echo "Commands:"
        echo "  start [output_file] [interval]  Start background monitoring"
        echo "  stop                            Stop background monitoring"
        echo "  status                          Check if monitor is running"
        echo "  snapshot                        Show current CPU assignments"
        echo "  verify <cpus>                   Verify processes are on expected CPUs (PSR)"
        echo "  verify_affinity <cpus> [regex]  Verify process affinity masks match"
        echo ""
        echo "Examples:"
        echo "  $0 start /tmp/cpu.csv 0.5"
        echo "  $0 snapshot"
        echo "  $0 verify 38-39"
        echo "  $0 verify_affinity 30-31"
        exit 1
        ;;
esac

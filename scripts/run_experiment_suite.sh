#!/bin/bash
# run_experiment_suite.sh - Run complete experiment suite with all scenarios
# Usage: ./run_experiment_suite.sh [num_trials] [scenarios]
#
# This script runs:
#   1. Baseline trials (no load)
#   2. CPU load trials
#   3. Message load trials
#   4. Analysis and aggregation
#
# Each scenario runs N trials with tracing enabled.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

NUM_TRIALS="${1:-10}"
SCENARIOS="${2:-baseline,cpu_load,msg_load}"
WARMUP_TRIALS=2
COOLDOWN_SECONDS=10       # Between trials (increased from 5 for stability)
SCENARIO_COOLDOWN=30      # Between scenarios (allow full system cooldown)
MAX_RETRIES=2  # Retry failed trials up to this many times
MOVEIT_ACTION_NAME="${MOVEIT_ACTION_NAME:-/move_action}"
BENCHMARK_TIMEOUT="${BENCHMARK_TIMEOUT:-60.0}"
CONFIG_FILE="$WS_ROOT/configs/experiment_config.yaml"
ENABLE_TRACING="${ENABLE_TRACING:-1}"
FAIL_FAST_START_STATE_INVALID_STREAK="${FAIL_FAST_START_STATE_INVALID_STREAK:-3}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[SUITE]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_section() { echo -e "\n${CYAN}======================================${NC}"; echo -e "${CYAN}=== $1 ===${NC}"; echo -e "${CYAN}======================================${NC}"; }

# =============================================================================
# ROS 2 ENVIRONMENT SETUP (function-based for stability with strict mode)
# =============================================================================

source_ros() {
    # Disable -u temporarily - ROS setup scripts use unset variables
    set +u

    ROS_FOUND=false
    for distro in jazzy humble iron rolling; do
        if [ -f "/opt/ros/${distro}/setup.bash" ]; then
            source "/opt/ros/${distro}/setup.bash"
            ROS_FOUND=true
            break
        fi
    done

    if [ "$ROS_FOUND" = false ]; then
        set -u
        log_error "No ROS 2 installation found in /opt/ros/"
        exit 1
    fi

    if [ -f "$WS_ROOT/install/setup.bash" ]; then
        source "$WS_ROOT/install/setup.bash"
    else
        set -u
        log_error "Workspace not built. Run 'make setup' first."
        exit 1
    fi

    # Activate venv if exists
    if [ -f "$WS_ROOT/.venv/bin/activate" ]; then
        source "$WS_ROOT/.venv/bin/activate"
    fi

    # Re-enable strict mode
    set -u
}

# Source ROS environment (must be called before any ROS commands)
source_ros

read_config_value() {
    local key_path="$1"
    local default_value="$2"
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "$default_value"
        return 0
    fi
    python3 "$SCRIPT_DIR/update_config.py" --config "$CONFIG_FILE" --get "$key_path" --quiet 2>/dev/null || echo "$default_value"
}

CPU_LOAD_WORKERS="${CPU_LOAD_WORKERS:-$(read_config_value load_scenarios.cpu_load.num_workers 4)}"
CPU_LOAD_PERCENT="${CPU_LOAD_PERCENT:-$(read_config_value load_scenarios.cpu_load.cpu_percent 80)}"
MSG_LOAD_RATE_HZ="${MSG_LOAD_RATE_HZ:-$(read_config_value load_scenarios.msg_load.rate_hz 1000)}"
MSG_LOAD_PUBLISHERS="${MSG_LOAD_PUBLISHERS:-$(read_config_value load_scenarios.msg_load.num_publishers 4)}"
MSG_LOAD_PAYLOAD_SIZE="${MSG_LOAD_PAYLOAD_SIZE:-$(read_config_value load_scenarios.msg_load.payload_size_bytes 1024)}"

validate_positive_int() {
    local name="$1"
    local value="$2"
    if ! [[ "$value" =~ ^[0-9]+$ ]]; then
        log_error "$name must be a non-negative integer (got: $value)"
        exit 1
    fi
}

validate_positive_int "CPU_LOAD_WORKERS" "$CPU_LOAD_WORKERS"
validate_positive_int "CPU_LOAD_PERCENT" "$CPU_LOAD_PERCENT"
validate_positive_int "MSG_LOAD_RATE_HZ" "$MSG_LOAD_RATE_HZ"
validate_positive_int "MSG_LOAD_PUBLISHERS" "$MSG_LOAD_PUBLISHERS"
validate_positive_int "MSG_LOAD_PAYLOAD_SIZE" "$MSG_LOAD_PAYLOAD_SIZE"
validate_positive_int "ENABLE_TRACING" "$ENABLE_TRACING"
validate_positive_int "FAIL_FAST_START_STATE_INVALID_STREAK" "$FAIL_FAST_START_STATE_INVALID_STREAK"

# =============================================================================
# PRE-FLIGHT CHECKS
# =============================================================================

preflight_check() {
    log_info "Running pre-flight checks..."
    local errors=0

    # Note: ROS 2 and workspace checks are now handled by source_ros() function
    # which exits immediately if they fail

    # Check workspace is built
    if [ ! -d "$WS_ROOT/install" ]; then
        log_error "Workspace not built. Run: make setup"
        errors=$((errors + 1))
    fi

    # Check ldos_harness package is available (file-based check - more reliable than ros2 pkg list)
    if [ ! -d "$WS_ROOT/install/ldos_harness" ]; then
        log_error "ldos_harness package not found. Run: make setup"
        errors=$((errors + 1))
    fi

    # Check launch file exists
    local launch_file="$WS_ROOT/src/ldos_harness/launch/full_stack.launch.py"
    if [ ! -f "$launch_file" ]; then
        log_error "Launch file not found: $launch_file"
        errors=$((errors + 1))
    fi

    # Check LTTng is available
    if [ "$ENABLE_TRACING" -eq 0 ]; then
        log_info "Tracing disabled for this run (ENABLE_TRACING=0)"
    else
        if ! command -v lttng &>/dev/null; then
            log_warn "LTTng not found. Tracing will be skipped."
        else
            # Check tracing group
            if ! groups | grep -q tracing; then
                log_warn "User not in 'tracing' group. Tracing may fail."
                log_warn "  Fix: sudo usermod -aG tracing \$USER && logout/login"
            fi
        fi
    fi

    # Check common tooling used at runtime
    for cmd in ros2 colcon python3 gz; do
        if ! command -v "$cmd" &>/dev/null; then
            log_error "Required command not found: $cmd"
            errors=$((errors + 1))
        fi
    done

    # Scenario-specific tools
    local -a preflight_scenarios=()
    IFS=',' read -ra preflight_scenarios <<< "$SCENARIOS"
    for scenario in "${preflight_scenarios[@]}"; do
        case "$scenario" in
            cpu_load)
                if ! command -v stress-ng &>/dev/null; then
                    log_error "cpu_load scenario requires stress-ng"
                    errors=$((errors + 1))
                fi
                ;;
            cpuset_limited)
                if ! command -v taskset &>/dev/null; then
                    log_error "cpuset_limited scenario requires taskset"
                    errors=$((errors + 1))
                fi
                if ! command -v systemd-run &>/dev/null; then
                    log_warn "systemd-run not found; cpuset launcher may fall back to taskset only"
                fi
                if [ -x "$SCRIPT_DIR/check_cpuset_support.sh" ]; then
                    log_info "Verifying cgroups v2 cpuset delegation for cpuset_limited..."
                    if ! "$SCRIPT_DIR/check_cpuset_support.sh" >/dev/null; then
                        log_error "cpuset_limited requires real cpuset delegation; run:"
                        log_error "  sudo ./scripts/setup_cpuset_delegation.sh"
                        log_error "  # then logout/login and rerun make check_cpuset"
                        errors=$((errors + 1))
                    fi
                else
                    log_warn "check_cpuset_support.sh not executable; cannot verify cpuset delegation"
                fi
                ;;
        esac
    done

    # Check helper scripts exist and are executable
    for script in start_trace.sh stop_trace.sh; do
        if [ ! -x "$SCRIPT_DIR/$script" ]; then
            log_warn "Script not executable: $SCRIPT_DIR/$script"
        fi
    done

    if [ $errors -gt 0 ]; then
        log_error "Pre-flight checks failed with $errors error(s)"
        exit 1
    fi

    log_info "All pre-flight checks passed"
}

preflight_check

log_section "LDOS Experiment Suite"
log_info "Trials per scenario: $NUM_TRIALS"
log_info "Warmup trials: $WARMUP_TRIALS (not recorded)"
log_info "Scenarios: $SCENARIOS"
log_info "Workspace: $WS_ROOT"

# Parse scenarios
IFS=',' read -ra SCENARIO_ARRAY <<< "$SCENARIOS"

# Global experiment metadata
EXPERIMENT_ID="exp_$(date +%Y%m%d_%H%M%S)"
EXPERIMENT_LOG="$WS_ROOT/results/${EXPERIMENT_ID}_log.txt"
mkdir -p "$WS_ROOT/results"
EXPERIMENT_DIR="$WS_ROOT/results/$EXPERIMENT_ID"
MANIFEST_DIR="$EXPERIMENT_DIR/manifest"
mkdir -p "$MANIFEST_DIR"
printf '%s\n' "$EXPERIMENT_ID" > "$WS_ROOT/results/.latest_experiment_id"

log_info "Experiment ID: $EXPERIMENT_ID"
log_info "Log file: $EXPERIMENT_LOG"
log_info "Manifest dir: $MANIFEST_DIR"

# Save experiment config
cat > "$WS_ROOT/results/${EXPERIMENT_ID}_config.json" << EOF
{
    "experiment_id": "$EXPERIMENT_ID",
    "start_time": "$(date -Iseconds)",
    "num_trials": $NUM_TRIALS,
    "warmup_trials": $WARMUP_TRIALS,
    "scenarios": [$(echo "${SCENARIO_ARRAY[@]}" | sed 's/ /", "/g' | sed 's/^/"/' | sed 's/$/"/')],
    "cooldown_seconds": $COOLDOWN_SECONDS
}
EOF
cp "$WS_ROOT/results/${EXPERIMENT_ID}_config.json" "$EXPERIMENT_DIR/config.json"

# Track results
declare -A SCENARIO_SUCCESS
declare -A SCENARIO_FAILED

run_scenario() {
    local scenario="$1"
    local num_trials="$2"
    local scenario_trial_manifest="$MANIFEST_DIR/${scenario}_trials.txt"
    local scenario_trace_manifest="$MANIFEST_DIR/${scenario}_trace_sessions.txt"

    log_section "Scenario: $scenario"

    SCENARIO_SUCCESS[$scenario]=0
    SCENARIO_FAILED[$scenario]=0
    : > "$scenario_trial_manifest"
    : > "$scenario_trace_manifest"
    local start_state_invalid_streak=0
    local scenario_aborted=false
    local scenario_abort_reason=""

    # Start the ROS stack once for all trials in this scenario
    log_info "Starting ROS stack for $scenario..."

    # Handle cpuset_limited scenario with split launch
    GAZEBO_PID=""
    if [ "$scenario" = "cpuset_limited" ]; then
        # cpuset_limited: Launch Gazebo and ROS stack on separate CPU sets
        log_info "Setting up cpuset-limited experiment..."

        # Auto-detect CPU count
        TOTAL_CPUS=$(nproc)
        ROS_CPU_COUNT="${ROS_CPU_COUNT:-2}"

        # Validate CPU count
        if [ "$ROS_CPU_COUNT" -ge "$TOTAL_CPUS" ]; then
            log_error "ROS_CPU_COUNT ($ROS_CPU_COUNT) >= TOTAL_CPUS ($TOTAL_CPUS)"
            return 1
        fi

        # Calculate CPU assignments
        # Gazebo: CPUs 0 through (TOTAL - ROS_CPU_COUNT - 1)
        # ROS:    Last ROS_CPU_COUNT CPUs
        GAZEBO_CPU_END=$((TOTAL_CPUS - ROS_CPU_COUNT - 1))
        ROS_CPU_START=$((TOTAL_CPUS - ROS_CPU_COUNT))
        ROS_CPU_END=$((TOTAL_CPUS - 1))

        GAZEBO_CPUS="0-${GAZEBO_CPU_END}"
        ROS_CPUS="${ROS_CPU_START}-${ROS_CPU_END}"

        log_info "Total CPUs: $TOTAL_CPUS"
        log_info "Gazebo CPUs: $GAZEBO_CPUS (unlimited)"
        log_info "ROS Stack CPUs: $ROS_CPUS (LIMITED to $ROS_CPU_COUNT)"
        log_info "Split cpuset mode: Gazebo/gz_ros2_control remain in Gazebo scope; ROS user-space nodes run in limited scope"

        # Launch Gazebo on unlimited CPUs
        log_info "Launching Gazebo on CPUs $GAZEBO_CPUS..."
        "$SCRIPT_DIR/cpuset_launch_v2.sh" "$GAZEBO_CPUS" "gazebo" \
            ros2 launch ldos_harness sim_only.launch.py headless:=true &
        GAZEBO_PID=$!

        # Wait for Gazebo to initialize
        log_info "Waiting for Gazebo to initialize (15s)..."
        sleep 15

        # Verify Gazebo is running
        if ! kill -0 $GAZEBO_PID 2>/dev/null; then
            log_error "Gazebo process died unexpectedly"
            return 1
        fi

        # Launch ROS stack on LIMITED CPUs
        log_info "Launching ROS stack on CPUs $ROS_CPUS (LIMITED)..."
        "$SCRIPT_DIR/cpuset_launch_v2.sh" "$ROS_CPUS" "ros_stack" \
            ros2 launch ldos_harness ros_stack.launch.py &
        STACK_PID=$!

    else
        # Standard scenarios: Use full_stack.launch.py
        ros2 launch ldos_harness full_stack.launch.py headless:=true use_rviz:=false &
        STACK_PID=$!
    fi

    # Wait for stack to initialize with polling and exponential backoff (timeout: 120s)
    log_info "Waiting for stack to initialize (polling, max 120s)..."
    STACK_TIMEOUT=120
    STACK_POLL_INTERVAL=2      # Start with 2s polling (reduced from 5s)
    STACK_POLL_MAX=10          # Max polling interval
    STACK_ELAPSED=0
    STACK_FULLY_READY=false

    while [ $STACK_ELAPSED -lt $STACK_TIMEOUT ]; do
        # Check if MoveGroup action is available
        if ros2 action list 2>/dev/null | grep -q "$MOVEIT_ACTION_NAME"; then
            # Also verify controllers are active for full readiness
            if ros2 control list_controllers 2>/dev/null | grep -q "panda_arm_controller.*active"; then
                log_info "Stack fully ready after ${STACK_ELAPSED}s (action + controllers)"
                STACK_FULLY_READY=true
                break
            else
                log_info "  MoveGroup ready, waiting for controllers..."
            fi
        fi

        # Check if stack process is still running
        if ! kill -0 $STACK_PID 2>/dev/null; then
            log_error "Stack process died unexpectedly"
            return 1
        fi

        sleep $STACK_POLL_INTERVAL
        STACK_ELAPSED=$((STACK_ELAPSED + STACK_POLL_INTERVAL))
        log_info "  Still waiting... (${STACK_ELAPSED}s / ${STACK_TIMEOUT}s)"

        # Exponential backoff (double interval up to max)
        STACK_POLL_INTERVAL=$((STACK_POLL_INTERVAL * 2))
        [ $STACK_POLL_INTERVAL -gt $STACK_POLL_MAX ] && STACK_POLL_INTERVAL=$STACK_POLL_MAX
    done

    # Final verification after polling loop
    if [ "$STACK_FULLY_READY" = false ]; then
        # One more check in case we timed out but it's actually ready now
        if ! ros2 action list 2>/dev/null | grep -q "$MOVEIT_ACTION_NAME"; then
            log_error "Stack failed to start for $scenario (timeout after ${STACK_TIMEOUT}s)"
            kill $STACK_PID 2>/dev/null || true
            return 1
        else
            log_warn "Stack ready but controllers may not be fully active"
        fi
    fi

    # Verify CPU isolation for cpuset_limited scenario
    ISOLATION_VERIFIED=""
    if [ "$scenario" = "cpuset_limited" ]; then
        log_info "Verifying CPU isolation (checking process affinity masks)..."
        sleep 3  # Allow children to fully spawn

        ISOLATION_VERIFIED="true"
        local verify_mismatches=0
        local verify_checked=0
        for pid in $(pgrep -f "move_group|robot_state_pub|parameter_bridge" 2>/dev/null || true); do
            local actual_mask
            actual_mask=$(awk '/Cpus_allowed_list/{print $2}' /proc/"$pid"/status 2>/dev/null || echo "unknown")
            local proc_name
            proc_name=$(awk '/^Name:/{print $2}' /proc/"$pid"/status 2>/dev/null || echo "?")
            verify_checked=$((verify_checked + 1))
            if [ "$actual_mask" = "$ROS_CPUS" ]; then
                log_info "  [OK] PID $pid ($proc_name) affinity=$actual_mask"
            else
                log_warn "  [FAIL] PID $pid ($proc_name) affinity=$actual_mask (expected $ROS_CPUS)"
                verify_mismatches=$((verify_mismatches + 1))
                ISOLATION_VERIFIED="false"
            fi
        done

        if [ "$verify_checked" -eq 0 ]; then
            log_warn "No ROS processes found for isolation verification (processes may not have spawned yet)"
            ISOLATION_VERIFIED="unknown"
        elif [ "$verify_mismatches" -eq 0 ]; then
            log_info "CPU isolation VERIFIED: $verify_checked processes restricted to $ROS_CPUS"
        else
            log_warn "CPU isolation PARTIAL: $verify_mismatches/$verify_checked processes have wrong affinity"
        fi
    fi

    # Start load generator if needed (not for cpuset_limited - it uses CPU isolation instead)
    LOAD_PID=""
    start_load_generator() {
        case "$scenario" in
            cpuset_limited)
                log_info "cpuset_limited scenario: No load generator (using CPU isolation instead)"
                return 0
                ;;
            cpu_load)
                log_info "Starting CPU load (workers=$CPU_LOAD_WORKERS load=${CPU_LOAD_PERCENT}%)..."
                "$SCRIPT_DIR/cpu_load.sh" "$CPU_LOAD_WORKERS" 0 "$CPU_LOAD_PERCENT" &
                LOAD_PID=$!
                ;;
            msg_load)
                log_info "Starting message load (${MSG_LOAD_PUBLISHERS} pubs x ${MSG_LOAD_RATE_HZ} Hz, payload=${MSG_LOAD_PAYLOAD_SIZE}B)..."
                "$SCRIPT_DIR/msg_load.sh" "$MSG_LOAD_RATE_HZ" "$MSG_LOAD_PUBLISHERS" 0 "$MSG_LOAD_PAYLOAD_SIZE" &
                LOAD_PID=$!
                ;;
            *)
                return 0
                ;;
        esac

        log_info "Waiting for load to stabilize (10s)..."
        sleep 10
        if ! kill -0 "$LOAD_PID" 2>/dev/null; then
            log_error "Load generator died unexpectedly during startup"
            return 1
        fi
        return 0
    }

    if [ "$scenario" = "cpuset_limited" ]; then
        log_info "cpuset_limited scenario: No load generator (using CPU isolation instead)"
    else
        if ! start_load_generator; then
            kill "$STACK_PID" 2>/dev/null || true
            return 1
        fi
    fi

    # Run warmup trials
    log_info "Running $WARMUP_TRIALS warmup trials..."
    for w in $(seq 1 $WARMUP_TRIALS); do
        log_info "Warmup trial $w/$WARMUP_TRIALS"
        ros2 run ldos_harness benchmark_runner.py \
            --trial-id "warmup_${scenario}_$(printf '%03d' "$w")" \
            --scenario "$scenario" \
            --output-dir "$WS_ROOT/results/warmup" \
            --timeout "$BENCHMARK_TIMEOUT" \
            --action-name "$MOVEIT_ACTION_NAME" 2>&1 || true
        sleep 2
    done

    # Cooldown after warmup to let system stabilize before data collection
    log_info "Warmup complete. Cooling down (10s) before data collection..."
    sleep 10

    # Run actual trials
    log_info "Running $num_trials data collection trials..."
    for trial in $(seq 1 $num_trials); do
        TRIAL_ID="${scenario}_$(printf '%03d' "$trial")"
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        TRACE_SESSION="ldos_${TRIAL_ID}_${TIMESTAMP}"
        TRACE_DIR="$WS_ROOT/traces/$TRACE_SESSION"
        RESULT_DIR="$WS_ROOT/results/$scenario"

        log_info "Trial $trial/$num_trials: $TRIAL_ID"
        printf '%s\n' "$TRIAL_ID" >> "$scenario_trial_manifest"

        # Ensure load is still active before each trial (avoid silent no-load runs)
        if [ "$scenario" = "cpu_load" ] || [ "$scenario" = "msg_load" ]; then
            if [ -z "$LOAD_PID" ] || ! kill -0 "$LOAD_PID" 2>/dev/null; then
                log_warn "Load generator not running before $TRIAL_ID, restarting..."
                if ! start_load_generator; then
                    log_error "Could not restart load generator"
                    SCENARIO_FAILED[$scenario]=$((${SCENARIO_FAILED[$scenario]} + 1))
                    continue
                fi
            fi
        fi

        if [ "$ENABLE_TRACING" -eq 1 ]; then
            mkdir -p "$TRACE_DIR"
        fi
        mkdir -p "$RESULT_DIR"

        # Collect metadata using Python for safe JSON encoding
        python3 -c "
import json
import os
import subprocess
from datetime import datetime

# Get CPU model safely
try:
    with open('/proc/cpuinfo', 'r') as f:
        for line in f:
            if 'model name' in line:
                cpu_model = line.split(':')[1].strip()
                break
        else:
            cpu_model = 'unknown'
except:
    cpu_model = 'unknown'

metadata = {
    'trial_id': '$TRIAL_ID',
    'experiment_id': '$EXPERIMENT_ID',
    'scenario': '$scenario',
    'trial_number': $trial,
    'timestamp': datetime.now().isoformat(),
    'trace_session': '$TRACE_SESSION',
    'environment': {
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        'hostname': subprocess.run(['hostname'], capture_output=True, text=True).stdout.strip(),
        'kernel': subprocess.run(['uname', '-r'], capture_output=True, text=True).stdout.strip(),
        'cpu_model': cpu_model
    }
}

with open('$RESULT_DIR/${TRIAL_ID}_metadata.json', 'w') as f:
    json.dump(metadata, f, indent=4)
"

        # Retry loop for transient failures
        TRIAL_SUCCESS=false
        for retry in $(seq 0 $MAX_RETRIES); do
            if [ $retry -gt 0 ]; then
                log_info "  Retry $retry/$MAX_RETRIES for $TRIAL_ID"
                # Update trace session name for retry
                TIMESTAMP=$(date +%Y%m%d_%H%M%S)
                TRACE_SESSION="ldos_${TRIAL_ID}_retry${retry}_${TIMESTAMP}"
                TRACE_DIR="$WS_ROOT/traces/$TRACE_SESSION"
                mkdir -p "$TRACE_DIR"
                sleep 2  # Brief pause before retry
            fi
            if [ "$ENABLE_TRACING" -eq 1 ]; then
                printf '%s\n' "$TRACE_SESSION" >> "$scenario_trace_manifest"
            fi

            # Start trace (check exit code for failure detection)
            TRACE_STARTED=false
            if [ "$ENABLE_TRACING" -eq 1 ]; then
                if "$SCRIPT_DIR/start_trace.sh" "$TRACE_SESSION" "$TRACE_DIR" > "$TRACE_DIR/trace_start.log" 2>&1; then
                    TRACE_STARTED=true
                else
                    log_warn "Tracing failed to start. Check $TRACE_DIR/trace_start.log"
                    # Save diagnostic info
                    lttng list > "$TRACE_DIR/lttng_list.log" 2>&1 || true
                fi
            else
                TRACE_STARTED=false
            fi

            sleep 1

            # Run benchmark
            BENCH_EXIT=0
            ros2 run ldos_harness benchmark_runner.py \
                --trial-id "$TRIAL_ID" \
                --scenario "$scenario" \
                --output-dir "$RESULT_DIR" \
                --timeout "$BENCHMARK_TIMEOUT" \
                --action-name "$MOVEIT_ACTION_NAME" 2>&1 || BENCH_EXIT=$?

            # Stop trace
            if [ "$ENABLE_TRACING" -eq 1 ]; then
                "$SCRIPT_DIR/stop_trace.sh" "$TRACE_SESSION" > /dev/null 2>&1 || true
            fi

            # Verify result file was created
            RESULT_FILE="$RESULT_DIR/${TRIAL_ID}_result.json"
            RESULT_FILE_EXISTS=false
            if [[ -f "$RESULT_FILE" ]]; then
                RESULT_FILE_EXISTS=true
            fi

            # Check if successful
            if [ $BENCH_EXIT -eq 0 ] && [ "$RESULT_FILE_EXISTS" = true ]; then
                TRIAL_SUCCESS=true
                break
            fi

            # Log retry reason
            if [ $retry -lt $MAX_RETRIES ]; then
                if [ $BENCH_EXIT -ne 0 ]; then
                    log_warn "  Trial failed (exit code $BENCH_EXIT), will retry..."
                elif [ "$RESULT_FILE_EXISTS" = false ]; then
                    log_warn "  Result file not created, will retry..."
                fi
            fi
        done

        # Track final success/failure after all retries
        if [ "$TRIAL_SUCCESS" = true ]; then
            SCENARIO_SUCCESS[$scenario]=$((${SCENARIO_SUCCESS[$scenario]} + 1))
            if [ $retry -gt 0 ]; then
                log_info "  Result: SUCCESS (after $retry retries)"
            else
                log_info "  Result: SUCCESS"
            fi
        else
            SCENARIO_FAILED[$scenario]=$((${SCENARIO_FAILED[$scenario]} + 1))
            if [ $BENCH_EXIT -ne 0 ]; then
                log_warn "  Result: FAILED (exit code $BENCH_EXIT after $MAX_RETRIES retries)"
            elif [ "$RESULT_FILE_EXISTS" = false ]; then
                log_warn "  Result: FAILED (result file not created after $MAX_RETRIES retries)"
            fi
        fi

        # Inspect result semantics for invalid-run patterns (e.g., MoveIt START_STATE_INVALID -26)
        if [ "${RESULT_FILE_EXISTS:-false}" = true ] && [ -f "$RESULT_FILE" ]; then
            RESULT_STATUS=""
            RESULT_MOVEIT_CODE=""
            RESULT_MOVEIT_LABEL=""
            read -r RESULT_STATUS RESULT_MOVEIT_CODE RESULT_MOVEIT_LABEL < <(
                python3 - <<PY
import json
try:
    with open("$RESULT_FILE") as f:
        d = json.load(f)
    status = d.get("status", "")
    code = d.get("moveit_error_code", "")
    label = d.get("moveit_error_label", "")
    print(status, code, label)
except Exception:
    print("", "", "")
PY
            )

            if [ "$RESULT_STATUS" = "start_state_invalid" ] || [ "${RESULT_MOVEIT_CODE:-}" = "-26" ]; then
                start_state_invalid_streak=$((start_state_invalid_streak + 1))
                log_warn "Detected START_STATE_INVALID on $TRIAL_ID (streak=${start_state_invalid_streak})"

                DIAG_FILE="$RESULT_DIR/${TRIAL_ID}_diagnostics.txt"
                {
                    echo "trial_id=$TRIAL_ID"
                    echo "scenario=$scenario"
                    echo "timestamp=$(date -Iseconds)"
                    echo "result_status=$RESULT_STATUS"
                    echo "moveit_error_code=$RESULT_MOVEIT_CODE"
                    echo "moveit_error_label=$RESULT_MOVEIT_LABEL"
                    echo ""
                    echo "=== ros2 action list ==="
                    ros2 action list 2>&1 || true
                    echo ""
                    echo "=== ros2 control list_controllers ==="
                    ros2 control list_controllers 2>&1 || true
                    echo ""
                    echo "=== ros2 node list ==="
                    ros2 node list 2>&1 || true
                } > "$DIAG_FILE"
                log_warn "Captured diagnostics: $DIAG_FILE"

                if [ "$FAIL_FAST_START_STATE_INVALID_STREAK" -gt 0 ] && \
                   [ "$start_state_invalid_streak" -ge "$FAIL_FAST_START_STATE_INVALID_STREAK" ]; then
                    scenario_aborted=true
                    scenario_abort_reason="Repeated START_STATE_INVALID (MoveIt -26) streak reached ${FAIL_FAST_START_STATE_INVALID_STREAK}"
                    log_error "$scenario_abort_reason"
                    log_error "Aborting scenario early to avoid wasting invalid trials"
                    break
                fi
            else
                start_state_invalid_streak=0
            fi
        fi

        # Cooldown between trials
        if [ $trial -lt $num_trials ]; then
            sleep $COOLDOWN_SECONDS
        fi
    done

    # Stop load generator
    if [ -n "$LOAD_PID" ] && kill -0 "$LOAD_PID" 2>/dev/null; then
        log_info "Stopping load generator..."
        kill "$LOAD_PID" 2>/dev/null || true
        wait "$LOAD_PID" 2>/dev/null || true
    fi

    # Stop stack
    log_info "Stopping ROS stack..."
    kill $STACK_PID 2>/dev/null || true
    wait $STACK_PID 2>/dev/null || true

    # Stop Gazebo if started separately (cpuset_limited scenario)
    if [ -n "$GAZEBO_PID" ] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
        log_info "Stopping Gazebo..."
        kill "$GAZEBO_PID" 2>/dev/null || true
        wait "$GAZEBO_PID" 2>/dev/null || true
    fi

    # Cool down before next scenario (longer than inter-trial cooldown)
    log_info "Cooling down (${SCENARIO_COOLDOWN}s) before next scenario..."
    sleep $SCENARIO_COOLDOWN

    if [ "$scenario_aborted" = true ]; then
        log_warn "Scenario $scenario aborted early: $scenario_abort_reason"
    fi
    log_info "Scenario $scenario complete: ${SCENARIO_SUCCESS[$scenario]} success, ${SCENARIO_FAILED[$scenario]} failed"
}

# Run all scenarios
for scenario in "${SCENARIO_ARRAY[@]}"; do
    # Use process substitution instead of a pipeline so run_scenario executes in
    # the current shell and can update SCENARIO_SUCCESS/SCENARIO_FAILED.
    run_scenario "$scenario" "$NUM_TRIALS" > >(tee -a "$EXPERIMENT_LOG") 2>&1
done

log_section "Running Analysis"
"$SCRIPT_DIR/analyze_traces.sh" all 2>&1 | tee -a "$EXPERIMENT_LOG"

log_section "Experiment Complete"
log_info "Experiment ID: $EXPERIMENT_ID"
log_info ""

# Print summary
log_info "Results Summary:"
for scenario in "${SCENARIO_ARRAY[@]}"; do
    log_info "  $scenario: ${SCENARIO_SUCCESS[$scenario]:-0} success, ${SCENARIO_FAILED[$scenario]:-0} failed"
done

log_info ""
log_info "Results: $WS_ROOT/results/"
log_info "Traces:  $WS_ROOT/traces/"
log_info "Analysis: $WS_ROOT/analysis/output/"
log_info "Log: $EXPERIMENT_LOG"

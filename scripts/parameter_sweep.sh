#!/bin/bash
# parameter_sweep.sh - Automated parameter sweeping for LDOS experiments
#
# This script sweeps a parameter across multiple values and runs experiments
# at each level to understand system behavior and find optimal configurations.
#
# Usage:
#   ./scripts/parameter_sweep.sh <param_name> <values> [trials_per_level]
#
# Examples:
#   ./scripts/parameter_sweep.sh cpu_load_percent "0 25 50 75 90" 30
#   ./scripts/parameter_sweep.sh num_publishers "1 5 10 20 50" 20
#   ./scripts/parameter_sweep.sh rate_hz "100 500 1000 2000" 25
#
# Supported parameters (from experiment_config.yaml):
#   - cpu_load_percent: CPU load for stress-ng (0-100)
#   - cpu_workers: Number of CPU workers
#   - num_publishers: Message flood publisher count
#   - rate_hz: Message publish rate
#   - payload_size_bytes: Message payload size

set -euo pipefail

# =============================================================================
# CONFIGURATION
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
CONFIG_FILE="$WS_ROOT/configs/experiment_config.yaml"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# =============================================================================
# ARGUMENT PARSING
# =============================================================================

if [ $# -lt 2 ]; then
    echo "Usage: $0 <param_name> <values> [trials_per_level]"
    echo ""
    echo "Arguments:"
    echo "  param_name        Parameter to sweep (e.g., cpu_load_percent)"
    echo "  values            Space-separated values in quotes (e.g., \"0 25 50 75 90\")"
    echo "  trials_per_level  Number of trials per value (default: 30)"
    echo ""
    echo "Supported parameters:"
    echo "  cpu_load_percent  - CPU load for stress-ng (0-100)"
    echo "  cpu_workers       - Number of CPU stress workers"
    echo "  num_publishers    - Message flood publisher count"
    echo "  rate_hz           - Message publish rate"
    echo "  payload_size_bytes - Message payload size"
    echo ""
    echo "Examples:"
    echo "  $0 cpu_load_percent \"0 25 50 75 90\" 30"
    echo "  $0 num_publishers \"1 5 10 20\" 25"
    exit 1
fi

PARAM_NAME="$1"
VALUES="$2"
TRIALS_PER_LEVEL="${3:-30}"

# Timestamp for this sweep
SWEEP_ID="sweep_${PARAM_NAME}_$(date +%Y%m%d_%H%M%S)"

echo -e "${BLUE}=== LDOS Parameter Sweep ===${NC}"
echo "Parameter: $PARAM_NAME"
echo "Values: $VALUES"
echo "Trials per level: $TRIALS_PER_LEVEL"
echo "Sweep ID: $SWEEP_ID"
echo ""

# =============================================================================
# FUNCTIONS
# =============================================================================

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

update_config() {
    local param="$1"
    local value="$2"

    log_info "Setting $param = $value in config"

    # Use Python script for safe YAML manipulation
    # This handles nested parameters correctly and avoids sed whitespace issues
    if python3 "$SCRIPT_DIR/update_config.py" \
        --config "$CONFIG_FILE" \
        --set "${param}=${value}" \
        --quiet; then
        return 0
    else
        log_error "Parameter $param not found in config file"
        return 1
    fi
}

get_scenario_for_param() {
    local param="$1"

    case "$param" in
        cpu_load_percent|cpu_workers)
            echo "cpu_load"
            ;;
        num_publishers|rate_hz|payload_size_bytes)
            echo "msg_load"
            ;;
        *)
            echo "baseline"
            ;;
    esac
}

# =============================================================================
# MAIN SWEEP LOOP
# =============================================================================

# Create sweep output directory
SWEEP_OUTPUT_DIR="$WS_ROOT/results/${SWEEP_ID}"
mkdir -p "$SWEEP_OUTPUT_DIR"

# Save sweep metadata
cat > "$SWEEP_OUTPUT_DIR/sweep_metadata.json" << EOF
{
    "sweep_id": "$SWEEP_ID",
    "parameter": "$PARAM_NAME",
    "values": [$( echo "$VALUES" | sed 's/ /, /g' )],
    "trials_per_level": $TRIALS_PER_LEVEL,
    "start_time": "$(date -Iseconds)",
    "hostname": "$(hostname)",
    "config_file": "$CONFIG_FILE"
}
EOF

# Backup original config
cp "$CONFIG_FILE" "$SWEEP_OUTPUT_DIR/original_config.yaml"

# Determine scenario type
SCENARIO=$(get_scenario_for_param "$PARAM_NAME")
log_info "Using scenario: $SCENARIO"

# Track results
declare -a COMPLETED_VALUES=()
declare -a FAILED_VALUES=()

# Run sweep
for value in $VALUES; do
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  Sweeping $PARAM_NAME = $value ${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""

    # Update configuration
    if ! update_config "$PARAM_NAME" "$value"; then
        log_error "Failed to update config for $PARAM_NAME = $value"
        FAILED_VALUES+=("$value")
        continue
    fi

    # Create output directory for this value
    VALUE_LABEL="${PARAM_NAME}_${value}"
    VALUE_OUTPUT_DIR="$WS_ROOT/results/${VALUE_LABEL}"

    # Run experiment suite
    log_info "Running $TRIALS_PER_LEVEL trials for $VALUE_LABEL"

    if "$SCRIPT_DIR/run_experiment_suite.sh" "$TRIALS_PER_LEVEL" "$SCENARIO"; then
        log_info "Completed trials for $VALUE_LABEL"

        # Move results to labeled directory
        if [ -d "$WS_ROOT/results/$SCENARIO" ]; then
            mv "$WS_ROOT/results/$SCENARIO" "$VALUE_OUTPUT_DIR"
            log_info "Results saved to $VALUE_OUTPUT_DIR"
        fi

        COMPLETED_VALUES+=("$value")
    else
        log_error "Experiment failed for $VALUE_LABEL"
        FAILED_VALUES+=("$value")
    fi

    # Brief pause between sweep levels
    sleep 5
done

# =============================================================================
# ANALYSIS
# =============================================================================

echo ""
echo -e "${BLUE}=== Sweep Complete ===${NC}"
echo "Completed: ${COMPLETED_VALUES[*]:-none}"
echo "Failed: ${FAILED_VALUES[*]:-none}"

# Restore original config
cp "$SWEEP_OUTPUT_DIR/original_config.yaml" "$CONFIG_FILE"
log_info "Restored original configuration"

# Run sweep analysis if any values completed
if [ ${#COMPLETED_VALUES[@]} -gt 0 ]; then
    echo ""
    log_info "Running sweep analysis..."

    # Build list of result directories
    RESULT_DIRS=""
    for value in "${COMPLETED_VALUES[@]}"; do
        RESULT_DIRS="$RESULT_DIRS $WS_ROOT/results/${PARAM_NAME}_${value}"
    done

    # Run sweep analysis
    if [ -f "$WS_ROOT/analysis/sweep_analysis.py" ]; then
        python3 "$WS_ROOT/analysis/sweep_analysis.py" \
            --param "$PARAM_NAME" \
            --results $RESULT_DIRS \
            --output "$SWEEP_OUTPUT_DIR" \
            2>&1 | tee "$SWEEP_OUTPUT_DIR/analysis.log"
    else
        log_warn "sweep_analysis.py not found, skipping analysis"
    fi
fi

# Update sweep metadata with completion info
cat > "$SWEEP_OUTPUT_DIR/sweep_metadata.json" << EOF
{
    "sweep_id": "$SWEEP_ID",
    "parameter": "$PARAM_NAME",
    "values": [$( echo "$VALUES" | sed 's/ /, /g' )],
    "trials_per_level": $TRIALS_PER_LEVEL,
    "start_time": "$(date -Iseconds)",
    "end_time": "$(date -Iseconds)",
    "hostname": "$(hostname)",
    "completed_values": [$(IFS=,; echo "${COMPLETED_VALUES[*]}" | sed 's/,/, /g')],
    "failed_values": [$(IFS=,; echo "${FAILED_VALUES[*]:-}" | sed 's/,/, /g')]
}
EOF

echo ""
echo -e "${GREEN}=== Parameter Sweep Complete ===${NC}"
echo "Results: $SWEEP_OUTPUT_DIR"
echo ""
echo "Next steps:"
echo "  1. Review results: ls -la $SWEEP_OUTPUT_DIR"
echo "  2. View analysis: cat $SWEEP_OUTPUT_DIR/sweep_report.md"
echo "  3. Plot results: python3 analysis/sweep_analysis.py --results $RESULT_DIRS"

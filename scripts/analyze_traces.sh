#!/bin/bash
# analyze_traces.sh - Post-process all traces and aggregate results
# Usage: ./analyze_traces.sh [scenario]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

SCENARIO="${1:-all}"

GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[ANALYZE]${NC} $1"; }
log_section() { echo -e "\n${CYAN}=== $1 ===${NC}"; }

log_section "LDOS Trace Analysis"

# Scenarios to process
if [ "$SCENARIO" = "all" ]; then
    SCENARIOS=("baseline" "cpu_load" "msg_load" "cpuset_limited")
else
    SCENARIOS=("$SCENARIO")
fi

# Use latest experiment manifest if available (prevents stale result/trace aggregation)
LATEST_EXPERIMENT_ID_FILE="$WS_ROOT/results/.latest_experiment_id"
LATEST_EXPERIMENT_ID=""
MANIFEST_DIR=""
if [ -f "$LATEST_EXPERIMENT_ID_FILE" ]; then
    LATEST_EXPERIMENT_ID="$(cat "$LATEST_EXPERIMENT_ID_FILE" 2>/dev/null || true)"
    if [ -n "$LATEST_EXPERIMENT_ID" ] && [ -d "$WS_ROOT/results/$LATEST_EXPERIMENT_ID/manifest" ]; then
        MANIFEST_DIR="$WS_ROOT/results/$LATEST_EXPERIMENT_ID/manifest"
        log_info "Using latest experiment manifest: $LATEST_EXPERIMENT_ID"
    fi
fi

# Process each scenario
for scenario in "${SCENARIOS[@]}"; do
    log_section "Processing: $scenario"

    RESULT_DIR="$WS_ROOT/results/$scenario"
    OUTPUT_DIR="$WS_ROOT/analysis/output/$scenario"

    if [ -n "$MANIFEST_DIR" ] && [ ! -f "$MANIFEST_DIR/${scenario}_trials.txt" ]; then
        log_info "Scenario $scenario not present in latest experiment manifest, skipping"
        continue
    fi

    if [ ! -d "$RESULT_DIR" ]; then
        log_info "No results found for $scenario, skipping"
        continue
    fi

    mkdir -p "$OUTPUT_DIR"

    # Aggregate benchmark results
    log_info "Aggregating benchmark results..."
    AGGREGATE_ARGS=()
    if [ -n "$MANIFEST_DIR" ] && [ -f "$MANIFEST_DIR/${scenario}_trials.txt" ]; then
        AGGREGATE_ARGS+=(--trial-id-file "$MANIFEST_DIR/${scenario}_trials.txt")
    fi
    if [ ${#AGGREGATE_ARGS[@]} -gt 0 ]; then
        python3 "$WS_ROOT/analysis/analyze_trace.py" \
            --result-dir "$RESULT_DIR" \
            --output-dir "$OUTPUT_DIR" \
            --aggregate-only \
            "${AGGREGATE_ARGS[@]}"
    else
        python3 "$WS_ROOT/analysis/analyze_trace.py" \
            --result-dir "$RESULT_DIR" \
            --output-dir "$OUTPUT_DIR" \
            --aggregate-only
    fi

    # Process individual traces (if they exist)
    # Enable nullglob so glob returns empty array if no matches
    TRACE_BASE="$WS_ROOT/traces"
    trace_dirs=()
    if [ -n "$MANIFEST_DIR" ] && [ -f "$MANIFEST_DIR/${scenario}_trace_sessions.txt" ]; then
        while IFS= read -r trace_session; do
            [ -n "$trace_session" ] || continue
            [ -d "$TRACE_BASE/$trace_session" ] || continue
            trace_dirs+=("$TRACE_BASE/$trace_session")
        done < "$MANIFEST_DIR/${scenario}_trace_sessions.txt"
    else
        shopt -s nullglob
        trace_dirs=("$TRACE_BASE"/ldos_${scenario}_*)
        shopt -u nullglob
    fi

    if [ ${#trace_dirs[@]} -eq 0 ]; then
        log_info "No trace directories found for $scenario"
    else
        trace_count=0
        skip_count=0
        for trace_dir in "${trace_dirs[@]}"; do
            if [ -d "$trace_dir" ]; then
                trace_name=$(basename "$trace_dir")

                # Skip empty trace directories (e.g., created with ENABLE_TRACING=0)
                # A valid LTTng trace has a metadata file in a subdirectory
                if ! find "$trace_dir" -name "metadata" -type f 2>/dev/null | grep -q .; then
                    log_info "Skipping empty/invalid trace: $trace_name (no LTTng metadata)"
                    skip_count=$((skip_count + 1))
                    continue
                fi

                log_info "Analyzing trace: $trace_name"
                python3 "$WS_ROOT/analysis/analyze_trace.py" \
                    --trace-dir "$trace_dir" \
                    --output-dir "$OUTPUT_DIR/$trace_name" || true
                trace_count=$((trace_count + 1))
            fi
        done
        log_info "Processed $trace_count traces, skipped $skip_count empty/invalid"
    fi
done

log_section "Generating Combined Summary"

# Combine all scenario summaries
COMBINED_CSV="$WS_ROOT/analysis/output/combined_summary.csv"
rm -f "$COMBINED_CSV"
first=true
for scenario in "${SCENARIOS[@]}"; do
    if [ -n "$MANIFEST_DIR" ] && [ ! -f "$MANIFEST_DIR/${scenario}_trials.txt" ]; then
        continue
    fi
    CSV="$WS_ROOT/analysis/output/$scenario/summary.csv"
    if [ -f "$CSV" ]; then
        if $first; then
            cat "$CSV" > "$COMBINED_CSV"
            first=false
        else
            tail -n +2 "$CSV" >> "$COMBINED_CSV"
        fi
    fi
done

if [ -f "$COMBINED_CSV" ]; then
    log_info "Combined summary: $COMBINED_CSV"

    # Print summary statistics
    log_section "Summary Statistics"
    python3 << EOF
import pandas as pd
df = pd.read_csv("$COMBINED_CSV")
print(df.groupby('scenario').agg({
    'status': lambda x: (x == 'success').sum(),
    'planning_latency_ms': ['mean', 'std'],
    'execution_latency_ms': ['mean', 'std'],
    'total_latency_ms': ['mean', 'std']
}).round(2).to_string())
EOF
fi

log_info "Analysis complete!"
log_info "Results in: $WS_ROOT/analysis/output/"

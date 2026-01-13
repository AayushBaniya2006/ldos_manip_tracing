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
    SCENARIOS=("baseline" "cpu_load" "msg_load")
else
    SCENARIOS=("$SCENARIO")
fi

# Process each scenario
for scenario in "${SCENARIOS[@]}"; do
    log_section "Processing: $scenario"

    RESULT_DIR="$WS_ROOT/results/$scenario"
    OUTPUT_DIR="$WS_ROOT/analysis/output/$scenario"

    if [ ! -d "$RESULT_DIR" ]; then
        log_info "No results found for $scenario, skipping"
        continue
    fi

    mkdir -p "$OUTPUT_DIR"

    # Aggregate benchmark results
    log_info "Aggregating benchmark results..."
    python3 "$WS_ROOT/analysis/analyze_trace.py" \
        --result-dir "$RESULT_DIR" \
        --output-dir "$OUTPUT_DIR" \
        --aggregate-only

    # Process individual traces (if they exist)
    TRACE_BASE="$WS_ROOT/traces"
    for trace_dir in "$TRACE_BASE"/ldos_${scenario}_*; do
        if [ -d "$trace_dir" ]; then
            trace_name=$(basename "$trace_dir")
            log_info "Analyzing trace: $trace_name"
            python3 "$WS_ROOT/analysis/analyze_trace.py" \
                --trace-dir "$trace_dir" \
                --output-dir "$OUTPUT_DIR/$trace_name" || true
        fi
    done
done

log_section "Generating Combined Summary"

# Combine all scenario summaries
COMBINED_CSV="$WS_ROOT/analysis/output/combined_summary.csv"
first=true
for scenario in "${SCENARIOS[@]}"; do
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

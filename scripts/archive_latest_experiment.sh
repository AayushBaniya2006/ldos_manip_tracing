#!/bin/bash
# archive_latest_experiment.sh - Archive latest experiment artifacts for traceability
#
# Usage:
#   ./archive_latest_experiment.sh <tag> [scenario]
#
# Examples:
#   ./archive_latest_experiment.sh cpuset_roscpu2_trials10 cpuset_limited
#   ./archive_latest_experiment.sh baseline_trials10 baseline

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[ARCHIVE]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[ARCHIVE]${NC} $1"; }

TAG="${1:-}"
SCENARIO_FILTER="${2:-}"

if [ -z "$TAG" ]; then
    echo "Usage: $0 <tag> [scenario]" >&2
    exit 1
fi

LATEST_ID_FILE="$WS_ROOT/results/.latest_experiment_id"
if [ ! -f "$LATEST_ID_FILE" ]; then
    echo "[ARCHIVE] ERROR: Latest experiment marker not found: $LATEST_ID_FILE" >&2
    exit 1
fi

EXPERIMENT_ID="$(cat "$LATEST_ID_FILE")"
if [ -z "$EXPERIMENT_ID" ]; then
    echo "[ARCHIVE] ERROR: Latest experiment marker is empty" >&2
    exit 1
fi

RESULT_ARCHIVE_DIR="$WS_ROOT/results/archive/$TAG"
ANALYSIS_ARCHIVE_DIR="$WS_ROOT/analysis/output/archive/$TAG"
mkdir -p "$RESULT_ARCHIVE_DIR" "$ANALYSIS_ARCHIVE_DIR"

log_info "Archiving experiment: $EXPERIMENT_ID"
log_info "Results archive: $RESULT_ARCHIVE_DIR"
log_info "Analysis archive: $ANALYSIS_ARCHIVE_DIR"

# Copy experiment-level metadata and logs.
for f in \
    "$WS_ROOT/results/${EXPERIMENT_ID}_log.txt" \
    "$WS_ROOT/results/${EXPERIMENT_ID}_config.json"; do
    if [ -f "$f" ]; then
        cp "$f" "$RESULT_ARCHIVE_DIR/"
    fi
done

if [ -d "$WS_ROOT/results/$EXPERIMENT_ID" ]; then
    cp -R "$WS_ROOT/results/$EXPERIMENT_ID" "$RESULT_ARCHIVE_DIR/"
fi

# Copy scenario result JSONs/metadata.
scenario_dirs=()
if [ -n "$SCENARIO_FILTER" ]; then
    scenario_dirs+=("$WS_ROOT/results/$SCENARIO_FILTER")
else
    while IFS= read -r d; do
        scenario_dirs+=("$d")
    done < <(find "$WS_ROOT/results" -maxdepth 1 -mindepth 1 -type d \
        ! -name "archive" \
        ! -name "exp_*" \
        ! -name "warmup" | sort)
fi

for scenario_dir in "${scenario_dirs[@]}"; do
    [ -d "$scenario_dir" ] || continue
    scenario_name="$(basename "$scenario_dir")"
    dest="$RESULT_ARCHIVE_DIR/$scenario_name"
    mkdir -p "$dest"
    find "$scenario_dir" -maxdepth 1 -type f \
        \( -name "*_result.json" -o -name "*_metadata.json" \) \
        -exec cp {} "$dest/" \;
done

# Copy analysis artifacts (combined + per-scenario summaries if present)
if [ -f "$WS_ROOT/analysis/output/combined_summary.csv" ]; then
    cp "$WS_ROOT/analysis/output/combined_summary.csv" "$ANALYSIS_ARCHIVE_DIR/"
fi
if [ -f "$WS_ROOT/docs/report_skeleton.md" ]; then
    cp "$WS_ROOT/docs/report_skeleton.md" "$ANALYSIS_ARCHIVE_DIR/" 2>/dev/null || true
fi

if [ -n "$SCENARIO_FILTER" ]; then
    if [ -d "$WS_ROOT/analysis/output/$SCENARIO_FILTER" ]; then
        cp -R "$WS_ROOT/analysis/output/$SCENARIO_FILTER" "$ANALYSIS_ARCHIVE_DIR/"
    fi
else
    for scenario_output in "$WS_ROOT"/analysis/output/*; do
        [ -d "$scenario_output" ] || continue
        base="$(basename "$scenario_output")"
        [ "$base" = "archive" ] && continue
        cp -R "$scenario_output" "$ANALYSIS_ARCHIVE_DIR/" 2>/dev/null || true
    done
fi

log_info "Archive complete."
log_info "Files copied:"
find "$RESULT_ARCHIVE_DIR" "$ANALYSIS_ARCHIVE_DIR" -maxdepth 2 -type f | sort

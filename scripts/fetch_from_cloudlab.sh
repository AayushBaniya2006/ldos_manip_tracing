#!/bin/bash
# fetch_from_cloudlab.sh - Retrieve experiment results from CloudLab
# Usage: ./scripts/fetch_from_cloudlab.sh user@node.cloudlab.us [local_dir]

set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Usage: $0 <cloudlab_ssh_target> [local_output_dir]"
    echo "Example: $0 aayush@node0.ldos-exp.MyProject.utah.cloudlab.us ~/cloudlab_results"
    exit 1
fi

CLOUDLAB_TARGET="$1"
LOCAL_DIR="${2:-$(pwd)/cloudlab_results_$(date +%Y%m%d_%H%M%S)}"

echo "=== Fetching Results from CloudLab ==="
echo "Source: $CLOUDLAB_TARGET"
echo "Destination: $LOCAL_DIR"
echo ""

mkdir -p "$LOCAL_DIR"

# Package results on remote
echo "Packaging results on CloudLab..."
REMOTE_TARBALL=$(ssh "$CLOUDLAB_TARGET" << 'REMOTE_SCRIPT'
cd ~/ldos_manip_tracing

TARBALL=~/ldos_results_$(date +%Y%m%d_%H%M%S).tar.gz

tar -czvf "$TARBALL" \
    results/ \
    analysis/output/ \
    configs/ \
    experiment_*.log 2>/dev/null || true

echo "$TARBALL"
REMOTE_SCRIPT
)

REMOTE_TARBALL=$(echo "$REMOTE_TARBALL" | tail -1)
echo "Remote tarball: $REMOTE_TARBALL"

# Download
echo ""
echo "Downloading..."
scp "${CLOUDLAB_TARGET}:${REMOTE_TARBALL}" "$LOCAL_DIR/"

# Extract
TARBALL_NAME=$(basename "$REMOTE_TARBALL")
cd "$LOCAL_DIR"
tar -xzvf "$TARBALL_NAME"

echo ""
echo "=== Results Retrieved ==="
echo "Location: $LOCAL_DIR"
echo ""
echo "Contents:"
ls -la "$LOCAL_DIR"

# Show quick summary if CSV exists
if [ -f "$LOCAL_DIR/analysis/output/combined_summary.csv" ]; then
    echo ""
    echo "=== Quick Summary ==="
    python3 << EOF
import pandas as pd
df = pd.read_csv('$LOCAL_DIR/analysis/output/combined_summary.csv')
print(df.groupby('scenario')[['planning_latency_ms','total_latency_ms']].describe().round(2))
EOF
fi

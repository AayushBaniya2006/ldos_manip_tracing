# LDOS Tracing Harness Runbook

Step-by-step commands for running experiments. All commands are copy/paste ready.

---

## Prerequisites Checklist

Run these checks before any experiment:

```bash
# 1. Check ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"  # Should print "jazzy"

# 2. Check workspace built
cd ~/ldos_manip_tracing
source install/setup.bash
ros2 pkg list | grep ldos_harness  # Should show ldos_harness

# 3. Check LTTng
lttng --version
groups | grep tracing  # User should be in tracing group

# 4. Check Gazebo
gz sim --version  # Should show 8.x.x

# 5. Check disk space
df -h ~/ldos_manip_tracing  # Need >10GB free
```

---

## Quick Start (5 minutes)

```bash
cd ~/ldos_manip_tracing
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run smoke test
make smoke_test
```

If smoke test passes, proceed to experiments.

---

## Full Experiment Suite (Automated)

This runs everything automatically: baseline + cpu_load + msg_load, N=10 each.

```bash
cd ~/ldos_manip_tracing
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run full suite (takes ~45 minutes)
make run_all NUM_TRIALS=10

# Or run with logging to file
make run_all NUM_TRIALS=10 2>&1 | tee experiment_$(date +%Y%m%d_%H%M%S).log
```

---

## Manual Experiment Execution

### Step 1: Start the ROS Stack

Terminal 1:
```bash
cd ~/ldos_manip_tracing
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Start full stack (headless)
ros2 launch ldos_harness full_stack.launch.py headless:=true use_rviz:=false
```

Wait 25 seconds for initialization. Verify:
```bash
# Terminal 2:
ros2 action list | grep move_action     # Should show /move_action
ros2 topic list | grep joint_states     # Should show /joint_states
ros2 control list_controllers           # Should show active controllers
```

### Step 2: Run Baseline Trials (N=10)

```bash
cd ~/ldos_manip_tracing
source install/setup.bash

# Run 10 baseline trials
for i in $(seq 1 10); do
    TRIAL_ID="baseline_$(printf '%03d' $i)"
    TRACE_SESSION="ldos_${TRIAL_ID}_$(date +%Y%m%d_%H%M%S)"

    echo "=== Running trial $i: $TRIAL_ID ==="

    # Start trace
    ./scripts/start_trace.sh "$TRACE_SESSION" "traces/$TRACE_SESSION"

    sleep 1

    # Run benchmark
    ros2 run ldos_harness benchmark_runner.py \
        --trial-id "$TRIAL_ID" \
        --scenario baseline \
        --output-dir results/baseline \
        --timeout 60.0

    # Stop trace
    ./scripts/stop_trace.sh "$TRACE_SESSION"

    echo "Trial $TRIAL_ID complete"
    sleep 5  # Cooldown
done

echo "Baseline trials complete"
```

### Step 3: Run CPU Load Trials (N=10)

Terminal 3 (start load generator):
```bash
cd ~/ldos_manip_tracing
./scripts/cpu_load.sh 4 1200  # 4 workers, 20 minutes
```

Terminal 2:
```bash
cd ~/ldos_manip_tracing
source install/setup.bash

for i in $(seq 1 10); do
    TRIAL_ID="cpu_load_$(printf '%03d' $i)"
    TRACE_SESSION="ldos_${TRIAL_ID}_$(date +%Y%m%d_%H%M%S)"

    echo "=== Running trial $i: $TRIAL_ID ==="

    ./scripts/start_trace.sh "$TRACE_SESSION" "traces/$TRACE_SESSION"
    sleep 1

    ros2 run ldos_harness benchmark_runner.py \
        --trial-id "$TRIAL_ID" \
        --scenario cpu_load \
        --output-dir results/cpu_load \
        --timeout 60.0

    ./scripts/stop_trace.sh "$TRACE_SESSION"

    sleep 5
done

# Stop load generator (Ctrl+C in Terminal 3)
```

### Step 4: Run Message Load Trials (N=10)

Terminal 3 (start load generator):
```bash
cd ~/ldos_manip_tracing
source install/setup.bash
./scripts/msg_load.sh 1000 4 1200  # 1000 Hz, 4 publishers, 20 minutes
```

Terminal 2:
```bash
cd ~/ldos_manip_tracing
source install/setup.bash

for i in $(seq 1 10); do
    TRIAL_ID="msg_load_$(printf '%03d' $i)"
    TRACE_SESSION="ldos_${TRIAL_ID}_$(date +%Y%m%d_%H%M%S)"

    echo "=== Running trial $i: $TRIAL_ID ==="

    ./scripts/start_trace.sh "$TRACE_SESSION" "traces/$TRACE_SESSION"
    sleep 1

    ros2 run ldos_harness benchmark_runner.py \
        --trial-id "$TRIAL_ID" \
        --scenario msg_load \
        --output-dir results/msg_load \
        --timeout 60.0

    ./scripts/stop_trace.sh "$TRACE_SESSION"

    sleep 5
done

# Stop load generator (Ctrl+C in Terminal 3)
```

### Step 5: Shutdown Stack

```bash
# Ctrl+C in Terminal 1 to stop the ROS stack
```

---

## Analysis Commands

### Aggregate Results

```bash
cd ~/ldos_manip_tracing

# Analyze all scenarios
./scripts/analyze_traces.sh all

# Or individual scenarios
./scripts/analyze_traces.sh baseline
./scripts/analyze_traces.sh cpu_load
./scripts/analyze_traces.sh msg_load
```

### Generate Plots

```bash
cd ~/ldos_manip_tracing

python3 analysis/plot_results.py \
    --input analysis/output/combined_summary.csv \
    --output-dir analysis/output/plots
```

### View Summary Statistics

```bash
cd ~/ldos_manip_tracing

# Quick stats
python3 -c "
import pandas as pd
df = pd.read_csv('analysis/output/combined_summary.csv')
print(df.groupby('scenario')[['planning_latency_ms','execution_latency_ms','total_latency_ms']].describe())
"
```

### Analyze Individual Trace

```bash
cd ~/ldos_manip_tracing

# Find a trace
ls traces/

# Analyze it
python3 analysis/analyze_trace.py \
    --trace-dir traces/ldos_baseline_001_20240101_120000 \
    --output-dir analysis/output/trace_analysis
```

---

## Troubleshooting Commands

### Check ROS Stack Health

```bash
# List all nodes
ros2 node list

# Check move_group specifically
ros2 node info /move_group

# Check action server
ros2 action info /move_action

# Check controller status
ros2 control list_controllers

# Check topic rates
ros2 topic hz /joint_states
```

### Check Tracing Status

```bash
# List active sessions
lttng list

# Check session details
lttng list ldos_trace_*

# View trace events (quick check)
babeltrace2 traces/ldos_*/ust/uid/*/64-bit/ | head -100
```

### Debug Failed Trial

```bash
# Check result file
cat results/baseline/baseline_001_result.json | python3 -m json.tool

# Check metadata
cat results/baseline/baseline_001_metadata.json | python3 -m json.tool

# Check trace exists
ls -la traces/ldos_baseline_001_*/
```

### Reset After Failure

```bash
# Kill all ROS processes
pkill -f ros2
pkill -f gz

# Destroy any orphaned LTTng sessions
lttng list | grep ldos | awk '{print $1}' | xargs -I {} lttng destroy {}

# Clear incomplete results (optional)
rm -f results/*/incomplete_*
```

---

## Data Backup

```bash
cd ~/ldos_manip_tracing

# Create timestamped backup
BACKUP_NAME="ldos_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p ~/backups

# Backup results and analysis (not traces - too large)
tar -czvf ~/backups/${BACKUP_NAME}_results.tar.gz results/ analysis/output/

# Backup traces separately (large)
tar -cvf ~/backups/${BACKUP_NAME}_traces.tar traces/

echo "Backup created: ~/backups/${BACKUP_NAME}_*"
```

---

## Expected Output Structure

After running all experiments:

```
~/ldos_manip_tracing/
├── results/
│   ├── baseline/
│   │   ├── baseline_001_result.json
│   │   ├── baseline_001_metadata.json
│   │   ├── baseline_002_result.json
│   │   ├── ...
│   │   └── baseline_010_metadata.json
│   ├── cpu_load/
│   │   └── (same structure)
│   └── msg_load/
│       └── (same structure)
├── traces/
│   ├── ldos_baseline_001_20240101_120000/
│   ├── ldos_baseline_002_20240101_120100/
│   └── ...
└── analysis/output/
    ├── baseline/
    │   └── summary.csv
    ├── cpu_load/
    │   └── summary.csv
    ├── msg_load/
    │   └── summary.csv
    ├── combined_summary.csv
    └── plots/
        ├── latency_boxplot.png
        ├── scenario_comparison.png
        └── degradation_analysis.png
```

---

## Timing Estimates

| Operation | Time |
|-----------|------|
| Smoke test | ~60 seconds |
| Single trial (baseline) | ~60 seconds |
| 10 baseline trials | ~12 minutes |
| 10 cpu_load trials | ~15 minutes |
| 10 msg_load trials | ~15 minutes |
| Full suite (30 trials) | ~45 minutes |
| Analysis | ~5 minutes |
| Total | ~50 minutes |

# LDOS Manipulation Tracing Research Harness

Tracing-Driven Performance Analysis for ROS 2 Manipulation Stacks

## Quick Start (Fresh CloudLab Node)

**Complete setup for fresh Ubuntu 24.04 CloudLab nodes:**

```bash
# 1. Clone and bootstrap (installs EVERYTHING - ~15-25 min)
git clone https://github.com/AayushBaniya2006/ldos_manip_tracing.git
cd ldos_manip_tracing
make bootstrap

# 2. IMPORTANT: Log out and back in for tracing group
exit
# SSH back in

# 3. Verify and run
cd ~/ldos_manip_tracing
make smoke_test              # Verify setup works (~60 sec)
make run_all NUM_TRIALS=10   # Run full experiment suite (~30 min)

# 4. Analyze results
make analyze_all
make report
```

---

## Results Summary

Experiments conducted on CloudLab AMD EPYC nodes (January 2026):

| Scenario | Trials | Success | Planning (ms) | Execution (ms) | Total (ms) |
|----------|--------|---------|---------------|----------------|------------|
| **Baseline** | 10 | 100% | 9.1 ± 5.2 | 886 ± 337 | 897 ± 340 |
| **CPU Load** | 10 | 100% | 14.7 ± 3.4 | 681 ± 285 | 698 ± 285 |
| **Msg Load** | 10 | **0%** | - | - | **SYSTEM FAILURE** |

### Key Findings

1. **CPU load increases planning latency by 62%** (9.1ms → 14.7ms)
2. **Message flood (4000 msg/s) causes 100% system failure**
   - DDS middleware cannot handle the message throughput
   - Action client/server communication breaks
   - Controller returns `UNKNOWN` status, MoveIt reports `CONTROL_FAILED`
3. **Execution time has high variance** (~340ms std) due to varying trajectory lengths

### Interpretation

- **CPU contention** degrades planning performance but system remains functional
- **DDS message flooding** is catastrophic - the system cannot recover
- The breaking point for message load is somewhere below 4000 msg/s (4 publishers × 1000 Hz)

---

## CPU Profiling with FlameGraph

This harness includes integrated CPU profiling using Linux `perf` and Brendan Gregg's FlameGraph tools.

### Quick CPU Profile

```bash
# Option 1: Profile during an experiment (recommended)
make profile_baseline        # Runs experiment with 60s CPU profiling

# Option 2: Profile while stack is already running
# Terminal 1: Start the stack
make smoke_test

# Terminal 2: Profile the running system
make profile_cpu             # System-wide profile
make profile_moveit          # MoveIt-specific profile
```

### Output Files

Profiles are saved to `analysis/output/profiles/`:

| File | Description |
|------|-------------|
| `*_flamegraph.svg` | Interactive flamegraph (open in browser) |
| `*_flamegraph_reverse.svg` | Callee-oriented view (most-called functions) |
| `*_report.txt` | Text report with CPU percentages |
| `*.perf.data` | Raw perf data for custom analysis |

### Viewing Flamegraphs

```bash
# Start HTTP server
cd analysis/output/profiles
python3 -m http.server 8000

# Open in browser: http://<node-ip>:8000/
# Click on *_flamegraph.svg files
```

### Reading Flamegraphs

- **Width** = CPU time spent in that function
- **Height** = Call stack depth (bottom = entry point, top = leaf functions)
- **Click** on any bar to zoom in
- **Search** (Ctrl+F) for specific functions like `moveit`, `ompl`, `planning`

**What to look for:**
- `gz::sim::*` - Gazebo simulation
- `dart::constraint::*` - Physics constraint solving
- `ompl::*` - Motion planning algorithms
- `moveit::*` - MoveIt planning/execution

### Export CPU Data to CSV

```bash
# Generate text report
sudo perf report -f -i analysis/output/profiles/*.perf.data --stdio --no-children 2>/dev/null | head -100

# Export top functions to CSV
sudo perf report -f -i analysis/output/profiles/*.perf.data --stdio --no-children -F overhead,comm,symbol 2>/dev/null | grep -E "^\s+[0-9]" | head -50 | sed 's/^\s*//' | tr -s ' ' ',' > cpu_profile.csv
```

---

## What This Does

This harness measures **how system load affects ROS 2 manipulation performance** by:

1. Running a Panda robot arm simulation (Gazebo Harmonic + MoveIt 2 + ros2_control)
2. Executing motion planning benchmarks under different load conditions
3. Capturing LTTng traces of callback execution, scheduling, and DDS messaging
4. Recording CPU profiles with FlameGraph visualization
5. Analyzing latency distributions across scenarios

### Scenarios Tested

| Scenario | Description | Expected Result |
|----------|-------------|-----------------|
| `baseline` | No artificial load, clean system | All trials succeed |
| `cpu_load` | 4 workers at 80% CPU via stress-ng | Increased planning latency |
| `msg_load` | 4000 Hz DDS message flood (4 pub × 1000 Hz) | System failure (DDS overload) |

### Metrics Collected (T1-T5)

| Metric | Description | Weight |
|--------|-------------|--------|
| T1 | Planning latency (ms) | 1.0 |
| T2 | Execution latency (ms) | 1.0 |
| T3 | Control loop jitter (ms) | 0.5 |
| T4 | Total end-to-end latency (ms) | 1.5 |
| T5 | Simulation step timing (ms) | 0.3 |

---

## Make Targets

```bash
make help              # Show all targets

# === Setup ===
make bootstrap         # Install ALL dependencies (fresh CloudLab, ~20 min)
make check_deps        # Verify dependencies installed
make setup             # Build workspace only
make clean             # Remove build artifacts

# === Quick Validation ===
make smoke_test        # Quick validation (~60 sec)
make acceptance_test   # Verify all components

# === Experiments ===
make run_baseline      # Baseline experiments (NUM_TRIALS=10)
make run_cpu_load      # CPU load experiments
make run_msg_load      # Message load experiments
make run_all           # All scenarios

# === CPU Profiling ===
make profile_cpu       # System-wide CPU profile (requires running stack)
make profile_moveit    # Profile MoveIt specifically
make profile_baseline  # Run baseline with CPU profiling
make profile_cpu_load  # Run CPU load with CPU profiling
make profile_msg_load  # Run msg load with CPU profiling

# === Analysis ===
make analyze_all       # Process all traces and results
make report            # Generate summary statistics
make hardware_info     # Collect system specs

# === Parameter Sweeps ===
make sweep_cpu         # Sweep CPU load (0-90%)
make sweep_msg         # Sweep message publishers (1-50)
make find_breaking     # Find system breaking point
```

---

## Directory Structure

```
ldos_manip_tracing/
├── Makefile                     # Main entry point (make help)
├── scripts/
│   ├── bootstrap_cloudlab.sh    # Install ALL dependencies (fresh node)
│   ├── setup_workspace.sh       # Build workspace (deps installed)
│   ├── run_experiment_suite.sh  # Automated experiment runner
│   ├── run_single_trial.sh      # Single trial execution
│   ├── start_trace.sh           # LTTng session start
│   ├── stop_trace.sh            # LTTng session stop
│   ├── cpu_load.sh              # CPU contention generator (stress-ng)
│   ├── msg_load.sh              # DDS message flood generator
│   ├── cpu_profile.sh           # CPU profiling with perf + FlameGraph
│   ├── profile_moveit.sh        # MoveIt-specific CPU profiling
│   ├── profile_experiment.sh    # Integrated profiling + experiment
│   ├── analyze_traces.sh        # Post-processing pipeline
│   └── parameter_sweep.sh       # Sweep parameter values
├── src/ldos_harness/            # ROS 2 package
│   ├── launch/                  # Launch files
│   ├── config/                  # Controller/MoveIt configs
│   └── scripts/                 # Python nodes (benchmark_runner.py, etc.)
├── configs/
│   ├── experiment_config.yaml   # Experiment parameters
│   ├── tracing_config.yaml      # LTTng event selection
│   └── objectives.yaml          # Path definitions & weights
├── analysis/
│   ├── analyze_trace.py         # LTTng trace processing
│   ├── aggregate_results.py     # Combine JSON results to CSV
│   ├── output/                  # Analysis outputs
│   │   ├── baseline/            # Per-scenario summaries
│   │   ├── cpu_load/
│   │   ├── msg_load/
│   │   ├── profiles/            # CPU flamegraphs (*.svg)
│   │   └── combined_summary.csv # All results combined
│   └── plot_results.py          # Visualization
├── traces/                      # Raw LTTng traces
├── results/                     # JSON benchmark results
│   ├── baseline/
│   ├── cpu_load/
│   └── msg_load/
└── docs/
    ├── RUNBOOK.md               # Step-by-step commands
    └── ASSUMPTIONS.md           # Reproducibility notes
```

---

## Requirements

Installed automatically by `make bootstrap`:

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- Gazebo Harmonic (gz-sim8)
- MoveIt 2 Jazzy
- ros2_control + ros2_controllers
- ros_gz (Gazebo-ROS bridge)
- LTTng (lttng-tools, liblttng-ust-dev, lttng-modules-dkms)
- ros2_tracing (ros2trace, tracetools_analysis)
- Linux perf tools
- FlameGraph (brendangregg/FlameGraph)
- Python 3.12+ (pandas, scipy, matplotlib, babeltrace2)
- stress-ng

---

## Troubleshooting

### Common Issues

**"ldos_harness package not found"**
```bash
# This is a false negative from ros2 pkg list in scripts
# The setup uses file-based detection now, just run:
make smoke_test
```

**"CONTROL_FAILED" / MoveIt error code -4**
```bash
# Controllers are in bad state, clean restart:
pkill -9 -f gazebo; pkill -9 -f ros2; pkill -9 -f gzserver
sleep 10
make smoke_test
```

**Tracing permission denied**
```bash
# Check if you're in tracing group
groups | grep tracing

# If not, add yourself and re-login:
sudo usermod -aG tracing $USER
exit
# SSH back in
```

**CPU profile shows 77% idle**
```bash
# The benchmark failed quickly, profile captured mostly idle time
# Run a clean successful experiment first:
pkill -9 -f gazebo; pkill -9 -f ros2; sleep 5
make smoke_test
make profile_baseline
```

**SSH disconnects during msg_load**
```bash
# Message flood can overwhelm network stack
# This is expected - it's a finding (system breaks under load)
# SSH back in and check results:
cd ~/ldos_manip_tracing
ls results/msg_load/
make analyze_all
```

### Reset Everything

```bash
# Kill all processes
pkill -9 -f gazebo
pkill -9 -f ros2
pkill -9 -f gzserver
pkill -9 -f move_group

# Destroy stale LTTng sessions
lttng list 2>/dev/null | grep ldos | awk '{print $1}' | xargs -I {} lttng destroy {} 2>/dev/null

# Wait and verify clean
sleep 10
ps aux | grep -E "(gazebo|ros2|move_group)" | grep -v grep
```

### Verify Installation

```bash
# Run acceptance tests
make acceptance_test

# Check individual components
ros2 pkg list | grep ldos_harness
lttng --version
perf --version
ls ~/FlameGraph/flamegraph.pl
```

---

## Data Export

### Export Results to CSV

```bash
# Combined summary (all scenarios)
cat analysis/output/combined_summary.csv

# Per-scenario summaries
cat analysis/output/baseline/summary.csv
cat analysis/output/cpu_load/summary.csv
cat analysis/output/msg_load/summary.csv
```

### Copy Results to Local Machine

```bash
# From your local machine:
scp -r user@cloudlab-node:~/ldos_manip_tracing/results/ ./
scp -r user@cloudlab-node:~/ldos_manip_tracing/analysis/output/ ./
```

### Generate Quick Statistics

```bash
python3 -c "
import pandas as pd
df = pd.read_csv('analysis/output/combined_summary.csv')
print(df.groupby('scenario')[['planning_latency_ms','execution_latency_ms','total_latency_ms']].describe())
"
```

---

## Citation

If you use this harness in your research, please cite:

```
LDOS Manipulation Tracing Harness
https://github.com/AayushBaniya2006/ldos_manip_tracing
```

## License

MIT License

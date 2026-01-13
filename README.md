# LDOS Manipulation Tracing Research Harness

Tracing-Driven Performance Analysis for ROS 2 Manipulation Stacks

## Quick Start (Fresh CloudLab Node)

**One-command setup for fresh Ubuntu 24.04 CloudLab nodes:**

```bash
# Clone and bootstrap (installs EVERYTHING - ~15-25 min)
git clone https://github.com/AayushBaniya2006/ldos_manip_tracing.git
cd ldos_manip_tracing
make bootstrap

# IMPORTANT: Log out and back in for tracing group
exit
# SSH back in, then:

cd ldos_manip_tracing
source ~/.bashrc
make smoke_test     # Verify setup works
make run_all        # Run full experiment suite
```

## Quick Start (Dependencies Already Installed)

```bash
cd ~/ldos_manip_tracing
make setup                    # Build workspace
source install/setup.bash     # Source environment
make smoke_test               # Verify setup
make run_all NUM_TRIALS=10    # Run experiments
```

## What This Does

This harness measures **how system load affects ROS 2 manipulation performance** by:

1. Running a Panda robot arm simulation (Gazebo + MoveIt 2 + ros2_control)
2. Executing motion planning benchmarks under different load conditions
3. Capturing LTTng traces of callback execution, scheduling, and DDS messaging
4. Analyzing latency distributions across scenarios

### Scenarios Tested

| Scenario | Description |
|----------|-------------|
| `baseline` | No artificial load, clean system |
| `cpu_load` | 4 workers at 80% CPU via stress-ng |
| `msg_load` | 4000 Hz DDS message flood (4 publishers × 1000 Hz) |

### Metrics Collected (T1-T5)

| Metric | Description | Weight |
|--------|-------------|--------|
| T1 | Planning latency (ms) | 1.0 |
| T2 | Execution latency (ms) | 1.0 |
| T3 | Control loop jitter (ms) | 0.5 |
| T4 | Total end-to-end latency (ms) | 1.5 |
| T5 | Simulation step timing (ms) | 0.3 |

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
│   ├── cpu_load.sh              # CPU contention generator
│   ├── msg_load.sh              # DDS message flood generator
│   ├── analyze_traces.sh        # Post-processing pipeline
│   └── parameter_sweep.sh       # Sweep parameter values
├── src/ldos_harness/            # ROS 2 package
│   ├── launch/                  # Launch files
│   ├── config/                  # Controller/MoveIt configs
│   └── scripts/                 # Python nodes
├── configs/
│   ├── experiment_config.yaml   # Experiment parameters
│   ├── tracing_config.yaml      # LTTng event selection
│   └── objectives.yaml          # Path definitions & weights
├── analysis/
│   ├── analyze_trace.py         # LTTng trace processing
│   ├── metrics_schema.py        # Metric definitions
│   ├── stats_utils.py           # Statistical functions
│   ├── plot_results.py          # Visualization
│   └── sweep_analysis.py        # Parameter sweep analysis
├── traces/                      # Raw LTTng traces
├── results/                     # JSON/CSV results
└── docs/
    ├── RUNBOOK.md               # Step-by-step commands
    └── ASSUMPTIONS.md           # Reproducibility notes
```

## Make Targets

```bash
make help              # Show all targets

# Setup
make bootstrap         # Install ALL dependencies (fresh CloudLab)
make check_deps        # Verify dependencies installed
make setup             # Build workspace
make clean             # Remove build artifacts

# Experiments
make smoke_test        # Quick validation (~60 sec)
make run_baseline      # Baseline experiments
make run_cpu_load      # CPU load experiments
make run_msg_load      # Message load experiments
make run_all           # All scenarios (NUM_TRIALS=10)

# Analysis
make analyze_all       # Process all traces
make report            # Generate summary statistics
make hardware_info     # Collect system specs
```

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
- Python 3.12+ (pandas, scipy, matplotlib, babeltrace2)
- stress-ng

## Detailed Documentation

- [RUNBOOK.md](docs/RUNBOOK.md) - Copy/paste commands for all operations
- [ASSUMPTIONS.md](docs/ASSUMPTIONS.md) - Reproducibility assumptions and risks

## Troubleshooting

```bash
# Check if tracing works
lttng create test && lttng destroy test

# If permission denied, ensure you're in tracing group
groups | grep tracing
# If not: sudo usermod -aG tracing $USER && exit (log back in)

# Check ROS 2 packages
ros2 pkg list | grep -E "moveit|ros_gz|tracetools"

# Reset after failure
pkill -f ros2 && pkill -f gz
lttng list | grep ldos | xargs -I {} lttng destroy {}
```

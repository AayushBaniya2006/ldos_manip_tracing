# LDOS Manipulation Tracing Research Harness

Tracing-Driven Performance Analysis for ROS2 Manipulation Stacks

## Quick Start

```bash
# 1. Setup (run once)
cd ~/ldos_manip_tracing
./scripts/setup_workspace.sh

# 2. Source environment
source install/setup.bash

# 3. Run full experiment suite
./scripts/run_experiment_suite.sh
```

## Directory Structure

```
ldos_manip_tracing/
├── README.md                    # This file
├── scripts/                     # Top-level automation scripts
│   ├── setup_workspace.sh       # One-time workspace setup
│   ├── run_experiment_suite.sh  # Full N=10 baseline/load experiments
│   ├── run_single_trial.sh      # Single trial with tracing
│   ├── start_trace.sh           # LTTng trace start wrapper
│   ├── stop_trace.sh            # LTTng trace stop wrapper
│   ├── cpu_load.sh              # CPU contention generator
│   ├── msg_load.sh              # DDS message flood generator
│   └── analyze_traces.sh        # Post-processing pipeline
├── src/
│   └── ldos_harness/            # ROS 2 package
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── setup.py
│       ├── launch/
│       │   ├── sim_bringup.launch.py
│       │   ├── moveit_bringup.launch.py
│       │   └── full_stack.launch.py
│       ├── config/
│       │   ├── panda_controllers.yaml
│       │   ├── moveit_config.yaml
│       │   └── benchmark_poses.yaml
│       └── scripts/
│           ├── benchmark_runner.py
│           ├── msg_flood_node.py
│           └── trace_marker_node.py
├── configs/
│   ├── tracing_config.yaml      # LTTng session configuration
│   └── experiment_config.yaml   # Experiment parameters
├── traces/                      # Raw LTTng trace output
├── results/                     # Processed CSV/JSON results
│   ├── baseline/
│   ├── cpu_load/
│   └── msg_load/
├── analysis/
│   ├── analyze_trace.py         # Main analysis script
│   ├── metrics_schema.py        # Metrics definitions
│   ├── plot_results.py          # Visualization
│   └── output/                  # Generated plots/tables
└── docs/
    └── report_skeleton.md       # Report template
```

## Assumptions Made

1. **ros2_control**: Configured from scratch with gz_ros2_control plugin
2. **MoveIt 2**: Using moveit_panda_config from moveit2_tutorials
3. **Headless**: Primary mode is headless; RViz optional via `USE_RVIZ=true`
4. **Gazebo**: Using gz-sim8 (Harmonic) with ros_gz bridge

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (gz-sim8)
- ros2_tracing (lttng-tools, ros2trace, tracetools_analysis)
- MoveIt 2 Jazzy
- Python 3.12+

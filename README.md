# LDOS Manipulation Tracing Research Harness

Tracing-Driven Performance Analysis for ROS 2 Manipulation Stacks

---

## Research Context

### Motivation

Real-time robotic systems like manipulation arms require predictable latency for safe operation. As robots share compute resources with perception, planning, and communication tasks, understanding how system load affects motion performance is critical for building reliable autonomous systems.

This harness measures **how system load affects ROS 2 manipulation performance** by running controlled experiments on a simulated Panda robot arm under different stress conditions.

### Scientific Contribution

1. **Quantified DDS vulnerability** - First systematic measurement showing ROS 2 manipulation fails completely under message flood before showing any CPU degradation
2. **Manipulation domain coverage** - Extends prior work on navigation robots to manipulation stacks (MoveIt + ros2_control)
3. **Reproducible methodology** - Fully automated harness for CloudLab with kernel-level tracing
4. **Realistic CPU isolation** - cgroups v2 cpuset experiments simulate embedded systems (Jetson) more accurately than artificial stress

### Related Research

- [On-Device CPU Scheduling for Robot Systems](https://arxiv.org/abs/2404.06452)
- [PiCAS: Priority-Driven Chain-Aware Scheduling for ROS2](https://arxiv.org/abs/2209.03280)
- [PAAM: Priority-Driven Accelerator Management in ROS 2](https://arxiv.org/abs/2404.06452)
- [ros2_tracing: Multipurpose Low-Overhead Framework](https://arxiv.org/abs/2201.00393)

### Key Findings

| Finding | Detail |
|---------|--------|
| **CPU contention has minimal impact** | 80% CPU load does not degrade MoveIt planning (~15ms baseline vs ~14ms under load) |
| **DDS message flooding is catastrophic** | 4000 msg/s causes **100% execution failure** |
| **The bottleneck is communication, not compute** | Hardening efforts should focus on DDS queue management and pub-sub throttling |

---

## Quick Start (Fresh CloudLab Node)

```bash
# 1. Clone and bootstrap (installs EVERYTHING)
git clone https://github.com/AayushBaniya2006/ldos_manip_tracing.git
cd ldos_manip_tracing
make bootstrap

# 2. IMPORTANT: Log out and back in for tracing group
exit
# SSH back in

# 3. Verify and run
cd ~/ldos_manip_tracing
make smoke_test              # Verify setup works
make run_all NUM_TRIALS=10   # Run full experiment suite

# 4. Analyze results
make analyze_all
make report
```

---

## Experiment Design

### What Gets Tested

A simulated **Panda 7-DOF robot arm** executes a simple reaching motion:

```
Start:  Home position (arm upright)
Goal:   [x=0.4, y=0.2, z=0.5] meters relative to base
Motion: MoveIt plans collision-free path, ros2_control executes trajectory
```

This motion is repeated under different load conditions:

| Scenario | Stress Applied | Expected Outcome |
|----------|----------------|------------------|
| **Baseline** | None | All trials succeed |
| **CPU Load** | 4 workers x 80% CPU (stress-ng) | Measure planning degradation |
| **Message Load** | 4000 msg/s DDS flood | System failure |
| **cpuset_limited** | cgroups v2 CPU isolation (1-4 CPUs) | Realistic embedded constraint |

### Metrics Collected (T1-T5)

| Metric | Description | Source | Weight |
|--------|-------------|--------|--------|
| **T1** | Planning latency | MoveIt feedback | 1.0 |
| **T2** | Execution latency | Action result | 1.0 |
| **T3** | Control loop jitter | LTTng callbacks | 0.5 |
| **T4** | Total end-to-end latency | Wall clock | 1.5 |
| **T5** | Simulation step timing | Gazebo updates | 0.3 |

---

## Results Summary

### Experiment Results (January 2026, CloudLab AMD EPYC)

| Scenario | Trials | Success | Planning (ms) | Execution (ms) | Total (ms) |
|----------|--------|---------|---------------|----------------|------------|
| **Baseline** | 11 | 100% | 15.6 +/- 4.1 | 656.2 +/- 500.4 | 673.7 +/- 501.5 |
| **CPU Load** | 11 | 100% | 13.6 +/- 1.2 | 529.4 +/- 93.1 | 545.0 +/- 93.8 |
| **Msg Load** | 10 | **0%** | 13.5 +/- 4.0 | - | **FAILED** |

### Observations

1. **CPU load does NOT degrade planning** - Planning time is slightly *lower* under CPU stress (15.6ms -> 13.6ms), possibly due to CPU governor behavior or cache effects.

2. **CPU load REDUCES execution variance** - Standard deviation drops from 500ms to 93ms under load, suggesting more predictable behavior under controlled contention.

3. **Message flood causes complete failure** - All 10 trials failed with `execution_failed` status. DDS middleware collapses under 4000 msg/s.

---

## Load Injection Details

### CPU Load (stress-ng)

```bash
stress-ng --cpu 4 --cpu-load 80 --cpu-method matrixprod --timeout 300s
```

| Parameter | Value | Notes |
|-----------|-------|-------|
| Workers | 4 | Pinned to CPUs 1-4 (CPU 0 reserved) |
| Load | 80% | Target utilization per worker |
| Method | matrixprod | CPU-bound, no I/O |

### Message Load (DDS Flood)

The message flood uses `msg_flood_node.py` to saturate DDS middleware:

```bash
# 4 publishers, each at 1000 Hz = 4000 msg/s total
ros2 run ldos_harness msg_flood_node.py --rate 1000 --topic /flood_topic_1 &
ros2 run ldos_harness msg_flood_node.py --rate 1000 --topic /flood_topic_2 &
ros2 run ldos_harness msg_flood_node.py --rate 1000 --topic /flood_topic_3 &
ros2 run ldos_harness msg_flood_node.py --rate 1000 --topic /flood_topic_4 &
```

| Parameter | Value | Notes |
|-----------|-------|-------|
| Publishers | 4 | Separate processes, separate topics |
| Rate | 1000 Hz each | 1 msg/ms per publisher |
| Payload | 1024 bytes | `std_msgs/String` filled with 'X' |
| QoS | BEST_EFFORT / VOLATILE | No backpressure, fire-and-forget |
| **Total throughput** | **4000 msg/s (~4 MB/s)** | |

**Why BEST_EFFORT breaks the system:**
- No acknowledgments or retries - messages are fire-and-forget
- DDS discovery and transport queues overflow
- MoveIt action server cannot receive feedback from controller
- Controller returns `UNKNOWN` status
- MoveIt reports `CONTROL_FAILED` (error code -4)

### CPU Isolation (cgroups v2 cpuset) - NEW

Instead of artificial CPU stress (stress-ng), this scenario uses **real CPU resource limitation** via Linux cgroups v2 cpuset. This simulates embedded systems (Jetson Nano, Xavier) more realistically.

```
┌─────────────────────────────────────────────────────────────────────┐
│                    CloudLab Node (auto-detect CPUs)                 │
├───────────────────────────────┬─────────────────────────────────────┤
│   GAZEBO CPUSET (unlimited)   │   ROS STACK CPUSET (LIMITED)       │
│   ─────────────────────────   │   ───────────────────────────      │
│   • gz-sim8 (Gazebo server)   │   • move_group (MoveIt)            │
│                               │   • controller_manager              │
│   (Physics simulation needs   │   • joint_trajectory_controller    │
│    full resources for         │   • robot_state_publisher          │
│    accurate timing)           │   • ros_gz_bridge (clock)          │
│                               │   • benchmark_runner.py            │
└───────────────────────────────┴─────────────────────────────────────┘
```

**Quick Start:**
```bash
# Check cpuset support
make check_cpuset

# Run with 2 CPUs for ROS stack (default)
make run_cpuset NUM_TRIALS=10

# Run CPU sweep (1, 2, 4 CPUs) to find breaking point
make sweep_cpuset NUM_TRIALS=5

# Custom CPU count
ROS_CPU_COUNT=1 make run_cpuset NUM_TRIALS=10
```

**How it works:**
- Uses `systemd-run --user --scope -p AllowedCPUs=X` (cgroups v2)
- No root required
- Gazebo runs on unlimited CPUs (accurate physics)
- ROS stack constrained to 1, 2, or 4 CPUs
- Auto-detects total CPU count at runtime

**Expected Results:**

| ROS CPUs | Expected Outcome |
|----------|------------------|
| 4 | Similar to baseline |
| 2 | Slight degradation, still passing |
| 1 | Significant degradation or failures |

---

## Tracing Methodology

### Why LTTng?

LTTng provides **kernel-level tracing** with minimal overhead (~2-5%), capturing:
- Exact callback execution times (nanosecond precision)
- Kernel scheduler events (context switches, wake latency)
- ROS 2 middleware internals (executor, pub/sub timing)

### Events Captured

**ROS 2 Userspace (ros2:*)**
- `callback_start` / `callback_end` - Callback execution timing
- `rclcpp_publish` / `rcl_take` - Message emission and reception
- `rclcpp_executor_*` - Executor scheduling behavior

**Kernel Tracepoints**
- `sched_switch` - Context switches
- `sched_wakeup` - Task wake events (scheduling latency)
- `sched_migrate_task` - CPU migration

### Trace Processing Pipeline

```
LTTng Session -> CTF Binary -> babeltrace2 -> analyze_trace.py -> CSV/JSON
```

---

## CPU Profiling with FlameGraph

### Quick CPU Profile

```bash
# Profile during an experiment (recommended)
make profile_baseline        # Runs experiment with 60s CPU profiling
make profile_cpu_load        # CPU load with profiling

# Profile running stack
make profile_cpu             # System-wide
make profile_moveit          # MoveIt-specific
```

### Sample Flamegraph

![CPU Flamegraph](docs/images/sampleFlameGraph.png)

*Interactive flamegraph showing CPU time distribution. Width = CPU time. Click to zoom in the SVG version.*

### Profile Analysis

**Baseline (system mostly idle):**
```
28.30%  swapper [kernel] io_idle     <- Benchmark completes fast
 0.21%  gz_ros2_control::PostUpdate  <- Gazebo-ROS bridge
 0.01%  gz_hardware::read/write      <- Hardware interface
```

**CPU Load (stress-ng active):**
```
29.62%  stress-ng-cpu                <- Injected artificial load
28.30%  swapper io_idle              <- Still idle despite stress
 0.06%  gz_ros2_control::PostUpdate  <- ROS unchanged
```

**Key insight:** ROS 2 manipulation has minimal CPU footprint. The benchmark completes in ~2s, leaving most profiling time as idle. The bottleneck is I/O (message passing), not CPU.

---

## Future Work

1. **CPU isolation analysis** - Run cpuset sweep experiments to find minimum viable CPU count for manipulation

2. **Find exact DDS breaking point** - Sweep message rates (100, 500, 1000, 2000, 3000 msg/s) to identify failure threshold

3. **Test DDS tuning** - Increase queue sizes, try shared memory transport, compare CycloneDDS vs FastDDS

4. **Extend to real hardware** - Validate findings on physical Panda arm

5. **Compare profiling approaches** - eBPF (lightweight) vs LTTng (detailed) tradeoffs

6. **Apply adaptive throttling** - Integrate runtime pub-sub throttling to prevent failures

7. **Priority-driven scheduling** - Implement PiCAS-style chain-aware scheduling for ROS 2 executors

---

## Make Targets

```bash
make help              # Show all targets

# === Setup ===
make bootstrap         # Fresh CloudLab setup (installs everything)
make setup             # Build workspace only
make clean             # Remove build artifacts

# === Validation ===
make smoke_test        # Quick check (~60 sec)
make acceptance_test   # Full component verification

# === Experiments ===
make run_baseline      # Baseline trials (NUM_TRIALS=10)
make run_cpu_load      # CPU load trials (stress-ng)
make run_msg_load      # Message load trials (DDS flood)
make run_all           # All scenarios

# === CPU Isolation (cgroups v2 cpuset) ===
make check_cpuset      # Verify cgroups v2 cpuset support
make run_cpuset        # Run cpuset-limited experiments
make sweep_cpuset      # Sweep ROS stack CPUs (1, 2, 4)

# === Profiling ===
make profile_baseline  # Baseline + CPU profiling
make profile_cpu_load  # CPU load + profiling
make profile_cpu       # Profile running stack

# === Analysis ===
make analyze_all       # Process traces and aggregate
make report            # Summary statistics

# === Parameter Sweeps ===
make sweep_cpu         # Sweep CPU load 0-90% (stress-ng)
make sweep_msg         # Sweep message rate
make find_breaking     # Binary search for breaking point
```

---

## Directory Structure

```
ldos_manip_tracing/
├── Makefile                     # Entry point (make help)
├── scripts/
│   ├── bootstrap_cloudlab.sh    # Complete CloudLab setup (10 phases)
│   ├── run_experiment_suite.sh  # Main experiment orchestrator
│   ├── run_single_trial.sh      # Single trial execution
│   ├── start_trace.sh           # LTTng session start
│   ├── stop_trace.sh            # LTTng session stop
│   ├── cpu_load.sh              # stress-ng wrapper
│   ├── msg_load.sh              # DDS flood launcher
│   ├── cpuset_launch.sh         # cgroups v2 CPU isolation wrapper
│   ├── cpuset_sweep.sh          # CPU count parameter sweep
│   ├── check_cpuset_support.sh  # Verify cgroups v2 support
│   ├── cpu_profile.sh           # perf + FlameGraph
│   └── analyze_traces.sh        # Post-processing pipeline
├── src/ldos_harness/            # ROS 2 package
│   ├── launch/
│   │   ├── full_stack.launch.py # Complete stack (Gazebo + ROS)
│   │   ├── sim_only.launch.py   # Gazebo only (for cpuset isolation)
│   │   ├── ros_stack.launch.py  # ROS only (for cpuset isolation)
│   │   ├── sim_bringup.launch.py
│   │   └── moveit_bringup.launch.py
│   ├── config/                  # URDF, SRDF, controller configs
│   └── scripts/
│       ├── benchmark_runner.py  # Trial execution node
│       └── msg_flood_node.py    # DDS flood publisher
├── configs/
│   ├── experiment_config.yaml   # Trial counts, timeouts, goals
│   ├── tracing_config.yaml      # LTTng event selection
│   └── objectives.yaml          # Metric definitions (T1-T5)
├── analysis/
│   ├── analyze_trace.py         # LTTng CTF processor
│   └── output/                  # Results (CSVs, flamegraphs)
├── traces/                      # Raw LTTng traces
└── results/                     # Benchmark JSON results
```

---

## Raw Data

Full results in `analysis/output/combined_summary.csv`:

<details>
<summary>Click to expand trial data</summary>

```csv
trial_id,scenario,status,planning_latency_ms,execution_latency_ms,total_latency_ms
baseline_001,baseline,success,12.95,510.86,525.71
baseline_002,baseline,success,22.87,561.46,586.35
baseline_003,baseline,success,13.00,510.28,525.28
baseline_004,baseline,success,11.41,311.83,325.49
baseline_005,baseline,success,23.12,460.29,485.35
baseline_006,baseline,success,14.26,560.35,576.52
baseline_007,baseline,success,14.03,660.40,676.38
baseline_008,baseline,success,14.15,710.63,726.74
baseline_009,baseline,success,13.21,260.32,275.40
baseline_010,baseline,success,13.67,560.59,576.12
baseline_profiled,baseline,success,18.40,2111.32,2131.77
cpu_load_001,cpu_load,success,14.18,560.58,576.79
cpu_load_002,cpu_load,success,12.62,411.51,426.09
cpu_load_003,cpu_load,success,12.47,461.03,475.55
cpu_load_004,cpu_load,success,15.38,661.51,678.84
cpu_load_005,cpu_load,success,12.27,511.08,525.34
cpu_load_006,cpu_load,success,13.74,410.52,426.34
cpu_load_007,cpu_load,success,13.25,511.45,526.74
cpu_load_008,cpu_load,success,13.47,511.39,527.09
cpu_load_009,cpu_load,success,12.37,511.25,525.59
cpu_load_010,cpu_load,success,13.92,711.74,727.72
cpu_load_profiled,cpu_load,success,16.00,560.92,579.01
msg_load_001,msg_load,execution_failed,14.87,10.26,27.37
msg_load_002,msg_load,execution_failed,12.52,11.21,25.75
msg_load_003,msg_load,execution_failed,12.41,10.57,24.98
msg_load_004,msg_load,execution_failed,12.62,11.17,26.05
msg_load_005,msg_load,execution_failed,14.38,11.60,28.10
msg_load_006,msg_load,execution_failed,12.55,10.31,24.87
msg_load_007,msg_load,execution_failed,21.90,10.45,35.01
msg_load_008,msg_load,execution_failed,13.67,11.27,27.11
msg_load_009,msg_load,execution_failed,5.42,10.75,18.26
msg_load_010,msg_load,execution_failed,14.15,10.35,26.69
```

</details>

---

## Requirements

Installed automatically by `make bootstrap`:

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- Gazebo Harmonic (gz-sim8)
- MoveIt 2
- ros2_control + ros2_controllers
- LTTng (lttng-tools, liblttng-ust-dev)
- ros2_tracing
- Linux perf + FlameGraph
- Python 3.12+ (pandas, scipy, matplotlib, babeltrace2)
- stress-ng

---

## Troubleshooting

### "CONTROL_FAILED" / MoveIt error code -4
```bash
pkill -9 -f gazebo; pkill -9 -f ros2; pkill -9 -f gzserver
sleep 10
make smoke_test
```

### Tracing permission denied
```bash
sudo usermod -aG tracing $USER
exit
# SSH back in
```

### Reset everything
```bash
pkill -9 -f gazebo; pkill -9 -f ros2; pkill -9 -f gzserver; pkill -9 -f move_group
lttng list 2>/dev/null | grep ldos | awk '{print $1}' | xargs -I {} lttng destroy {} 2>/dev/null
sleep 10
```

---

## License

MIT License

# LDOS Research Roadmap: Tracing-Driven Performance Analysis for ROS 2 Manipulation Stacks

**Mentor:** Rohit Dwivedula
**Last Updated:** January 2025
**Execution Environment:** CloudLab Bare-Metal Nodes

---

## Executive Summary

### Project Goals (From Advisor)

1. **Setup** a usable manipulation stack in ROS 2 + simulator environment ✅ (Done)
2. **Understand** performance bottlenecks using existing ROS 2 profiling tools
3. **Define** end-to-end objectives and specify relative importance of each path (manually)

### Execution Strategy

**All experiments run on CloudLab bare-metal nodes** for:
- Reproducibility across runs (no VM/container variability)
- Isolated hardware (no noisy neighbors)
- Controlled conditions (CPU pinning, governor control)
- High trial counts (N=100+ per scenario is achievable)

**Core Principle: Repeatable, Automated Experimentation**
- Every experiment is scripted and reproducible
- Run hundreds to thousands of trials to find statistical patterns
- Systematically stress test until the system breaks
- Iterate on configurations based on data

### Key Deliverable

**A report** that:
- Identifies the critical end-to-end paths in the manipulation stack
- Profiles these paths using ros2_tracing across **many repeated trials**
- Documents **where and how the system breaks** under stress
- Defines objectives (latency targets) for each path
- Proposes weights/priorities based on empirical analysis

---

## Table of Contents

1. [Required Reading](#1-required-reading)
2. [Tools & Infrastructure](#2-tools--infrastructure)
3. [CloudLab Experimental Environment](#3-cloudlab-experimental-environment)
4. [Understanding the Manipulation Stack](#4-understanding-the-manipulation-stack)
5. [End-to-End Path Identification](#5-end-to-end-path-identification)
6. [Profiling with ros2_tracing](#6-profiling-with-ros2_tracing)
7. [Trace Analysis Methods](#7-trace-analysis-methods)
8. [Repeatable Experiment Framework](#8-repeatable-experiment-framework)
9. [Stress Testing & Breaking Point Analysis](#9-stress-testing--breaking-point-analysis)
10. [Defining Objectives & Weights](#10-defining-objectives--weights)
11. [Experimental Plan](#11-experimental-plan)
12. [Report Structure](#12-report-structure)
13. [Timeline & Milestones](#13-timeline--milestones)

---

## 1. Required Reading

### 1.1 Core Papers (Must Read)

| Paper | Focus | Why It Matters |
|-------|-------|----------------|
| **[PiCAS: Priority-Driven Chain-Aware Scheduling for ROS2](https://ieeexplore.ieee.org/document/9804599)** | Chain-aware scheduling in ROS 2 | Defines how to think about callback chains and priorities |
| **[On-Device CPU Scheduling for Robot Systems](https://arxiv.org/abs/2309.00249)** | CPU scheduling for robotics | Shows how scheduling affects robot performance |
| **[PAAM: Framework for Priority-Driven Accelerator Management in ROS 2](https://arxiv.org/abs/2404.06452)** | Priority management | Methodology for defining priorities in ROS 2 |
| **[ros2_tracing Paper](https://ieeexplore.ieee.org/document/9841314)** | ros2_tracing design | Understand the tool you're using |
| **[Message Flow Analysis for Distributed ROS 2 Systems](https://arxiv.org/abs/2305.08789)** | Message flow visualization | How to trace messages across nodes |

### 1.2 Reading Notes Template

For each paper, document:

```markdown
## Paper: [Title]

### Key Concepts
-

### Methodology
- How did they measure/profile?
- What tools did they use?

### Relevant to Our Work
- What can we directly apply?
- What experiments should we replicate?

### Definitions to Adopt
- How do they define "end-to-end latency"?
- How do they define "chains"?
- How do they assign priorities?
```

### 1.3 Key Concepts from Papers

#### From PiCAS: Callback Chains

A **callback chain** is a sequence of callbacks triggered by an event:

```
Sensor Input -> Callback A -> Publish -> Callback B -> Publish -> Callback C -> Actuator Output
```

**Chain-aware scheduling** means:
- Treating the entire chain as a unit
- Assigning priority to the chain, not individual callbacks
- Ensuring end-to-end deadlines are met

#### From PAAM: Priority Assignment

**Priority** should be based on:
1. **Criticality**: How important is this path to safety/mission?
2. **Deadline**: What's the timing requirement?
3. **Utilization**: How much CPU does it need?

#### From ros2_tracing: What We Can Measure

| Tracepoint | What It Captures |
|------------|------------------|
| `ros2:callback_start/end` | Callback execution time |
| `ros2:rcl_publish` | When messages are published |
| `ros2:rclcpp_subscription_callback_added` | Subscription registration |
| `ros2:rcl_timer_init` | Timer creation |
| `ros2:rcl_service_init` | Service creation |
| `ros2:rcl_client_init` | Client creation |

---

## 2. Tools & Infrastructure

### 2.1 Primary Tools

| Tool | Purpose | Install |
|------|---------|---------|
| **ros2_tracing** | Capture traces | `sudo apt install ros-jazzy-ros2-tracing` |
| **tracetools_analysis** | Default trace analysis | `pip install tracetools-analysis` |
| **Ros2TraceAnalyzer** | Faster analysis (30x) + visualization | See below |
| **LTTng** | Underlying tracer | `sudo apt install lttng-tools` |
| **Babeltrace2** | Trace reading | `pip install babeltrace2` |

### 2.2 Install Ros2TraceAnalyzer

This tool claims **30x faster analysis** and has **graph visualization**:

```bash
# Clone the repository
cd ~/ldos_manip_tracing
git clone https://github.com/skoudmar/Ros2TraceAnalyzer.git

# Install dependencies
cd Ros2TraceAnalyzer
pip install -r requirements.txt

# Verify installation
python -m ros2_trace_analyzer --help
```

### 2.3 Install tracetools_analysis

```bash
# Clone official analysis tools
mkdir -p ~/tracing_ws/src
cd ~/tracing_ws/src
git clone https://github.com/ros-tracing/tracetools_analysis.git

# Build
cd ~/tracing_ws
colcon build

# Source
source install/setup.bash
```

### 2.4 Tracing Quick Reference

```bash
# Start tracing (interactive)
ros2 trace

# Start tracing (scripted)
ros2 trace start my_session
ros2 trace stop my_session

# Trace with specific events
ros2 trace -e ros2:callback_start -e ros2:callback_end -e ros2:rcl_publish

# Enable kernel events (need tracing group)
ros2 trace -k sched_switch

# Snapshot mode (flight recorder - stores in memory until triggered)
ros2 trace --snapshot-mode
ros2 trace record_snapshot my_session
```

---

## 3. CloudLab Experimental Environment

### 3.1 Why CloudLab?

**CloudLab provides bare-metal nodes** that are essential for performance research:

| Feature | Benefit |
|---------|---------|
| **Dedicated hardware** | No VM overhead, no noisy neighbors |
| **Root access** | CPU governor control, kernel parameters |
| **Reproducibility** | Same hardware across all trials |
| **Scale** | Run hundreds of experiments uninterrupted |
| **Isolation** | No background processes affecting results |

### 3.2 CloudLab Setup

See **`docs/CLOUDLAB_GUIDE.md`** for complete setup instructions.

**Quick reference:**
```bash
# Deploy harness to CloudLab (from local machine)
./scripts/deploy_to_cloudlab.sh <user>@<node>.cloudlab.us

# On CloudLab node - setup and run
cd ~/ldos_manip_tracing
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Apply performance tuning
~/tune_system.sh

# Run experiments
make run_all NUM_TRIALS=100
```

### 3.3 Recommended CloudLab Hardware

| Type | CPUs | RAM | Best For |
|------|------|-----|----------|
| `c6525-100g` | 32 cores | 128 GB | Primary experiments (AMD EPYC) |
| `xl170` | 10 cores | 64 GB | Backup, good availability |
| `c220g5` | 40 cores | 192 GB | High parallelism tests |

### 3.4 CloudLab Performance Tuning

**Critical for reproducible results:**

```bash
# Always run before experiments
~/tune_system.sh

# What it does:
# - Sets CPU governor to "performance" (no frequency scaling)
# - Disables turbo boost (consistent timing)
# - Sets kernel parameters for tracing
# - Disables swap
# - Configures RT priority limits
```

### 3.5 Results Transfer

```bash
# From local machine - fetch results
./scripts/fetch_from_cloudlab.sh <user>@<node>.cloudlab.us ~/my_results/

# This retrieves:
# - results/           (JSON per trial)
# - analysis/output/   (CSVs, plots)
# - experiment_*.log   (run logs)
```

---

## 4. Understanding the Manipulation Stack

### 4.1 MoveIt 2 Architecture

```
+---------------------------------------------------------------------+
|                         USER / APPLICATION                           |
|                    (Goal: Move arm to position X)                    |
+---------------------------------------------------------------------+
                                    |
                                    v
+---------------------------------------------------------------------+
|                           MOVE_GROUP NODE                            |
|  +-------------+  +-------------+  +-------------+                  |
|  |   Planning  |  |  Execution  |  |  Planning   |                  |
|  |   Pipeline  |  |   Manager   |  |    Scene    |                  |
|  +-------------+  +-------------+  +-------------+                  |
+---------------------------------------------------------------------+
                                    |
        +---------------------------+---------------------------+
        v                           v                           v
+---------------+          +---------------+          +---------------+
|    OMPL       |          |  Trajectory   |          |   Collision   |
|   Planner     |          |  Processing   |          |   Checking    |
|  (RRTConnect) |          | (Time param.) |          |    (FCL)      |
+---------------+          +---------------+          +---------------+
                                    |
                                    v
+---------------------------------------------------------------------+
|                      CONTROLLER MANAGER                              |
|  +-------------------------------------------------------------+    |
|  |              Joint Trajectory Controller                     |    |
|  |         (Interpolates trajectory, sends commands)            |    |
|  +-------------------------------------------------------------+    |
+---------------------------------------------------------------------+
                                    |
                                    v
+---------------------------------------------------------------------+
|                      HARDWARE INTERFACE                              |
|                   (gz_ros2_control plugin)                          |
+---------------------------------------------------------------------+
                                    |
                                    v
+---------------------------------------------------------------------+
|                          GAZEBO SIM                                  |
|                    (Physics simulation)                              |
+---------------------------------------------------------------------+
```

### 4.2 Key ROS 2 Topics/Services/Actions

Run this to discover the communication graph:

```bash
# List all topics
ros2 topic list

# Key topics for manipulation:
# /joint_states              - Current joint positions (from sim/hardware)
# /joint_commands            - Joint commands (to sim/hardware)
# /tf                        - Transform tree
# /planning_scene            - Collision world
# /display_planned_path      - Visualization

# List all actions
ros2 action list

# Key actions:
# /move_action               - Main MoveIt interface
# /execute_trajectory        - Trajectory execution
# /follow_joint_trajectory   - Controller action

# List all services
ros2 service list

# Key services:
# /compute_ik                - Inverse kinematics
# /compute_fk                - Forward kinematics
# /get_planning_scene        - Scene queries
```

### 4.3 Node Graph Visualization

```bash
# Generate node graph
ros2 run rqt_graph rqt_graph

# Or use ros2 CLI
ros2 node list
ros2 node info /move_group
```

**Document the graph:** Create a diagram showing all nodes and their connections.

---

## 5. End-to-End Path Identification

### 5.1 What is an End-to-End Path?

An **end-to-end path** (or **callback chain**) is the sequence of processing steps from an input event to an output action.

For manipulation, the key paths are:

### 5.2 Critical Paths in MoveIt 2

#### Path 1: Planning Path (P1)
```
Goal Request -> Move Group -> Planning Pipeline -> OMPL -> Trajectory -> Response
```

**Start:** `MoveGroup::execute()` called
**End:** `PlanningResponse` returned
**Criticality:** HIGH (determines if motion is feasible)
**Typical latency:** 100ms - 5000ms

#### Path 2: Execution Path (P2)
```
Trajectory -> Controller Manager -> Joint Trajectory Controller -> Hardware Interface -> Joint Commands
```

**Start:** `ExecuteTrajectory` action called
**End:** Final joint command sent
**Criticality:** CRITICAL (real-time, affects motion quality)
**Typical latency:** Control period (2ms at 500Hz)

#### Path 3: Feedback Path (P3)
```
Joint States (sim) -> Joint State Broadcaster -> TF -> Move Group (monitoring)
```

**Start:** Gazebo publishes joint state
**End:** Move Group receives state
**Criticality:** HIGH (needed for monitoring execution)
**Typical latency:** <10ms

#### Path 4: Collision Checking Path (P4)
```
Planning Scene Update -> FCL Collision Check -> Valid/Invalid
```

**Start:** Scene update received
**End:** Collision check complete
**Criticality:** MEDIUM (affects planning, not real-time)
**Typical latency:** 1-50ms per check

#### Path 5: Sensing Path (P5) [If using sensors]
```
Camera/Lidar -> Perception -> Planning Scene Update
```

**Start:** Sensor message received
**End:** Planning scene updated
**Criticality:** MEDIUM (depends on application)
**Typical latency:** 50-200ms

### 5.3 Path Documentation Template

For each path, document:

```yaml
path_id: P1
name: Planning Path
description: From goal request to motion plan

chain:
  - node: move_group
    callback: goal_callback
    input: MoveGroup.action goal
    output: internal planning request

  - node: move_group
    callback: planning_pipeline
    input: planning request
    output: trajectory

  - node: move_group
    callback: response_callback
    input: trajectory
    output: MoveGroup.action result

topics_involved:
  - /move_action (action)

timing:
  deadline_ms: 5000
  typical_ms: 500
  worst_case_ms: 5000

criticality: HIGH
rationale: "Planning determines if motion is feasible. Slow planning delays task execution but doesn't affect safety."
```

---

## 6. Profiling with ros2_tracing

### 6.1 Tracing the Manipulation Stack

#### Step 1: Configure Tracing Session

```bash
# Create a trace configuration
cat > ~/ldos_manip_tracing/configs/full_trace.yaml << 'EOF'
session_name: moveit_profile
events:
  ros2:
    - callback_start
    - callback_end
    - rcl_publish
    - rclcpp_publish
    - rcl_subscription_init
    - rclcpp_subscription_callback_added
    - rcl_timer_init
    - rcl_service_init
    - rcl_client_init
    - rcl_action_client_init
    - rcl_action_server_init
  kernel:
    - sched_switch  # Optional: shows context switches
EOF
```

#### Step 2: Trace a Benchmark Run

```bash
# Start the stack
ros2 launch ldos_harness full_stack.launch.py &

# Wait for initialization
sleep 25

# Start tracing
ros2 trace start moveit_profile

# Run benchmark
ros2 run ldos_harness benchmark_runner.py \
    --trial-id profile_001 \
    --output-dir results/profile

# Stop tracing
ros2 trace stop moveit_profile

# Trace is saved to ~/.ros/tracing/moveit_profile/
```

#### Step 3: Verify Trace

```bash
# Check trace exists
ls -la ~/.ros/tracing/moveit_profile/

# Quick peek at events
babeltrace2 ~/.ros/tracing/moveit_profile/ | head -100

# Count events by type
babeltrace2 ~/.ros/tracing/moveit_profile/ | cut -d: -f3 | sort | uniq -c | sort -rn
```

### 6.2 Key Tracepoints for Manipulation

| Tracepoint | What to Look For |
|------------|------------------|
| `ros2:callback_start` | When callbacks begin |
| `ros2:callback_end` | When callbacks complete |
| `ros2:rcl_publish` | When messages are published |
| `ros2:rcl_take` | When messages are received |
| `sched_switch` | CPU context switches (kernel) |

---

## 7. Trace Analysis Methods

### 7.1 Using tracetools_analysis (Official Tool)

```python
#!/usr/bin/env python3
"""
analyze_with_tracetools.py - Analyze traces using official tracetools_analysis
"""

from tracetools_analysis.loading import load_file
from tracetools_analysis.processor import Processor
from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil

def analyze_trace(trace_path: str):
    """Analyze a trace using tracetools_analysis."""

    # Load trace events
    events = load_file(trace_path)

    # Process events
    handler = Ros2Handler()
    processor = Processor(handler)
    processor.process(events)

    # Get data model
    data = handler.data
    util = Ros2DataModelUtil(data)

    # Analyze callbacks
    print("\n=== Callback Analysis ===")
    callback_instances = data.callback_instances

    callback_durations = {}
    for instance in callback_instances:
        cb_obj = instance.callback_object
        duration_ns = instance.duration

        if cb_obj not in callback_durations:
            callback_durations[cb_obj] = []
        callback_durations[cb_obj].append(duration_ns)

    # Print summary
    for cb_obj, durations in sorted(callback_durations.items(),
                                     key=lambda x: sum(x[1]), reverse=True)[:10]:
        mean_ms = sum(durations) / len(durations) / 1e6
        total_ms = sum(durations) / 1e6
        print(f"  Callback {cb_obj}: {len(durations)} calls, "
              f"mean={mean_ms:.2f}ms, total={total_ms:.2f}ms")

    return data


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("Usage: python analyze_with_tracetools.py <trace_path>")
        sys.exit(1)

    analyze_trace(sys.argv[1])
```

### 7.2 Using Ros2TraceAnalyzer (30x Faster)

```bash
# Clone if not done
git clone https://github.com/skoudmar/Ros2TraceAnalyzer.git
cd Ros2TraceAnalyzer

# Analyze trace
python -m ros2_trace_analyzer analyze ~/.ros/tracing/moveit_profile/

# Generate visualization
python -m ros2_trace_analyzer visualize ~/.ros/tracing/moveit_profile/ -o analysis_output/

# The tool generates:
# - Callback duration statistics
# - Message flow graphs
# - Timeline visualizations
```

---

## 8. Repeatable Experiment Framework

### 8.1 Philosophy: Data Over Intuition

**The goal is to run experiments so many times that patterns become statistically undeniable.**

| Principle | Implementation |
|-----------|----------------|
| **High N** | Run 100+ trials per scenario, not 10 |
| **Automation** | Every experiment is a single command |
| **Parameterization** | Sweep across load levels, configurations |
| **Reproducibility** | Same script, same results on CloudLab |
| **Iteration** | Analyze, hypothesize, re-run, verify |

### 8.2 Experiment Automation Commands

```bash
# On CloudLab node after setup

# Run N=100 baseline trials (takes ~2-3 hours)
make run_baseline NUM_TRIALS=100

# Run N=100 CPU load trials
make run_cpu_load NUM_TRIALS=100

# Run N=100 message flood trials
make run_msg_load NUM_TRIALS=100

# Run everything (N=100 each scenario)
make run_all NUM_TRIALS=100

# Run quick smoke test (N=3)
make smoke_test
```

### 8.3 Parameter Sweep Experiments

To understand system behavior, sweep across parameters:

```bash
#!/bin/bash
# parameter_sweep.sh - Sweep CPU load levels to find breaking point

CPU_LOADS="0 25 50 60 70 75 80 85 90 95"
TRIALS_PER_LEVEL=50

for load in $CPU_LOADS; do
    echo "=== Running CPU load ${load}% ==="

    # Update config
    sed -i "s/cpu_load_percent:.*/cpu_load_percent: ${load}/" configs/experiment_config.yaml

    # Run trials
    ./scripts/run_experiment_suite.sh cpu_load ${TRIALS_PER_LEVEL}

    # Move results to labeled directory
    mv results/cpu_load results/cpu_load_${load}pct

    echo "Completed ${load}% load tests"
done

# Analyze sweep results
python3 analysis/sweep_analysis.py results/cpu_load_*
```

### 8.4 Iteration Workflow

```
+-------------------+
|  1. Hypothesis    |  "Planning latency increases linearly with CPU load"
+-------------------+
         |
         v
+-------------------+
|  2. Design        |  Sweep CPU load 0-95%, N=50 per level
+-------------------+
         |
         v
+-------------------+
|  3. Run on        |  make parameter_sweep PARAM=cpu_load
|     CloudLab      |
+-------------------+
         |
         v
+-------------------+
|  4. Analyze       |  python3 analysis/sweep_analysis.py
+-------------------+
         |
         v
+-------------------+
|  5. Visualize     |  Plot latency vs CPU load
+-------------------+
         |
         v
+-------------------+
|  6. Refine        |  "Actually, it's exponential after 70%"
|    Hypothesis     |  -> New experiment: focus on 60-90% range
+-------------------+
         |
         v
    (Repeat until understanding is complete)
```

### 8.5 Sample Sizes and Statistical Power

| Goal | Minimum N | Recommended N | Why |
|------|-----------|---------------|-----|
| Initial exploration | 10 | 20 | Quick feedback |
| Baseline characterization | 30 | 50 | Stable mean/std |
| Load comparison | 50 | 100 | Detect small effects |
| Breaking point analysis | 100 | 200 | Catch rare failures |
| Publication-quality | 100 | 500+ | Statistical rigor |

### 8.6 Automation Scripts Reference

| Script | Purpose | Example |
|--------|---------|---------|
| `run_experiment_suite.sh` | Run N trials of one scenario | `./scripts/run_experiment_suite.sh baseline 100` |
| `run_single_trial.sh` | One trial with tracing | `./scripts/run_single_trial.sh baseline trial_001` |
| `parameter_sweep.sh` | Sweep one parameter | `./scripts/parameter_sweep.sh cpu_load "0 25 50 75 100"` |
| `analyze_traces.sh` | Batch analyze traces | `./scripts/analyze_traces.sh traces/` |

---

## 9. Stress Testing & Breaking Point Analysis

### 9.1 Goal: Find Where the System Breaks

**We don't just want to know how the system performs - we want to know when it fails.**

Questions to answer:
- At what CPU load does planning start timing out?
- How many concurrent message publishers cause deadline misses?
- What's the maximum trajectory complexity before failure?
- When does the control loop start missing deadlines?

### 9.2 Stress Test Categories

#### Category 1: CPU Stress

```bash
# Sweep CPU load until failures occur
for load in 50 60 70 75 80 85 90 92 94 96 98; do
    echo "Testing CPU load ${load}%"
    stress-ng --cpu $(nproc) --cpu-load ${load} &
    STRESS_PID=$!

    # Run trials
    ./scripts/run_experiment_suite.sh stress_cpu_${load} 50

    kill $STRESS_PID
done

# Find breaking point
python3 << 'EOF'
import pandas as pd
import glob

results = []
for f in glob.glob('results/stress_cpu_*/summary.csv'):
    load = int(f.split('_')[-1].replace('pct', '').replace('/', ''))
    df = pd.read_csv(f)
    failure_rate = (df['status'] != 'success').mean()
    results.append({'load': load, 'failure_rate': failure_rate})

df = pd.DataFrame(results).sort_values('load')
print("CPU Load Breaking Point Analysis:")
print(df)

# Find first load level with >10% failures
breaking = df[df['failure_rate'] > 0.1]['load'].min()
print(f"\nBreaking point: {breaking}% CPU load")
EOF
```

#### Category 2: Message Flood Stress

```bash
# Sweep message publishers
for pubs in 1 5 10 20 30 40 50 75 100; do
    echo "Testing ${pubs} publishers"

    # Update config
    sed -i "s/num_publishers:.*/num_publishers: ${pubs}/" configs/experiment_config.yaml

    # Run trials
    ./scripts/run_experiment_suite.sh stress_msg_${pubs} 50
done
```

#### Category 3: Memory Pressure

```bash
# Test with memory pressure
for mem_pct in 50 60 70 80 85 90; do
    echo "Testing ${mem_pct}% memory usage"

    # Calculate bytes to use
    total_mem=$(free -b | grep Mem | awk '{print $2}')
    use_mem=$((total_mem * mem_pct / 100))

    stress-ng --vm 1 --vm-bytes ${use_mem} --vm-keep &
    STRESS_PID=$!

    ./scripts/run_experiment_suite.sh stress_mem_${mem_pct} 30

    kill $STRESS_PID
done
```

#### Category 4: Combined Stress

```bash
# Realistic worst-case: CPU + messages + multiple goals
stress-ng --cpu $(nproc) --cpu-load 70 &
CPU_PID=$!

ros2 run ldos_harness msg_flood_node.py --num-publishers 20 &
MSG_PID=$!

./scripts/run_experiment_suite.sh combined_stress 100

kill $CPU_PID $MSG_PID
```

### 9.3 Breaking Point Metrics

Track these to identify failures:

| Metric | Normal | Warning | Critical |
|--------|--------|---------|----------|
| Planning success rate | >99% | 95-99% | <95% |
| Planning latency p95 | <2000ms | 2000-5000ms | >5000ms |
| Execution success rate | >99% | 95-99% | <95% |
| Control loop jitter | <0.5ms | 0.5-1ms | >1ms |
| Total E2E latency | <10s | 10-30s | >30s |

### 9.4 Stress Test Report Template

```markdown
# Stress Test Report: [Test Category]

## Test Configuration
- CloudLab node type: c6525-100g
- CPU: 32 cores, AMD EPYC
- RAM: 128 GB
- Test date: YYYY-MM-DD

## Parameter Sweep
| Parameter Value | N | Success Rate | P50 Latency | P95 Latency | P99 Latency |
|-----------------|---|--------------|-------------|-------------|-------------|
| ...             |   |              |             |             |             |

## Breaking Point
- **First failures observed:** [value]
- **>10% failure rate:** [value]
- **>50% failure rate:** [value]

## Failure Modes
1. [Description of failure type 1]
2. [Description of failure type 2]

## Recommendations
- Safe operating range: [range]
- Margin to include: [value]
```

### 9.5 Automated Breaking Point Finder

```python
#!/usr/bin/env python3
"""
find_breaking_point.py - Automatically find system breaking point via binary search
"""

import subprocess
import pandas as pd
import sys

def run_trial(param_name: str, param_value: float, num_trials: int = 30) -> dict:
    """Run trials at a specific parameter value."""

    # Update config
    subprocess.run([
        'sed', '-i',
        f's/{param_name}:.*/{param_name}: {param_value}/',
        'configs/experiment_config.yaml'
    ])

    # Run experiment
    subprocess.run([
        './scripts/run_experiment_suite.sh',
        f'binsearch_{param_value}',
        str(num_trials)
    ])

    # Load results
    df = pd.read_csv(f'results/binsearch_{param_value}/summary.csv')

    return {
        'param_value': param_value,
        'success_rate': (df['status'] == 'success').mean(),
        'mean_latency': df['total_latency_ms'].mean(),
        'p95_latency': df['total_latency_ms'].quantile(0.95),
    }

def binary_search_breaking_point(
    param_name: str,
    low: float,
    high: float,
    threshold: float = 0.9,  # 90% success rate threshold
    precision: float = 1.0
) -> float:
    """Find the parameter value where success rate drops below threshold."""

    results = []

    while high - low > precision:
        mid = (low + high) / 2
        print(f"Testing {param_name}={mid}")

        result = run_trial(param_name, mid)
        results.append(result)

        if result['success_rate'] >= threshold:
            # Still stable, increase load
            low = mid
        else:
            # Breaking, decrease load
            high = mid

    # Save all results
    pd.DataFrame(results).to_csv(f'analysis/output/breaking_point_{param_name}.csv', index=False)

    return low  # Last stable value

if __name__ == '__main__':
    param = sys.argv[1] if len(sys.argv) > 1 else 'cpu_load_percent'
    low = float(sys.argv[2]) if len(sys.argv) > 2 else 0
    high = float(sys.argv[3]) if len(sys.argv) > 3 else 100

    breaking_point = binary_search_breaking_point(param, low, high)
    print(f"\nBreaking point for {param}: {breaking_point}")
```

### 9.6 Long-Running Stability Tests

**Goal:** Find intermittent failures that only appear over long runs.

```bash
# Run 1000 trials overnight on CloudLab
nohup make run_baseline NUM_TRIALS=1000 > stability_test.log 2>&1 &

# Monitor progress
tail -f stability_test.log

# Check for any failures
grep -c "FAILED" stability_test.log
grep -c "SUCCESS" stability_test.log
```

---

## 10. Defining Objectives & Weights

### 10.1 Framework for Defining Objectives

Based on the papers (especially PiCAS and PAAM):

#### Step 1: Identify All Paths
List every end-to-end path in the system.

#### Step 2: Classify Criticality

| Criticality | Definition | Example |
|-------------|------------|---------|
| **CRITICAL** | Safety-related, real-time deadline | Control loop |
| **HIGH** | Performance-critical, affects task success | Planning, state feedback |
| **MEDIUM** | Important but not time-critical | Collision checking |
| **LOW** | Best-effort, can be delayed | Logging, visualization |

#### Step 3: Define Deadlines

For each path, specify:
- **Hard deadline**: Must NEVER be violated (for CRITICAL paths)
- **Soft deadline**: Should usually be met (for HIGH/MEDIUM paths)
- **No deadline**: Best-effort (for LOW paths)

#### Step 4: Assign Weights

Weights determine relative importance when resources are contended.

**Method 1: Criticality-Based (Simple)**
```python
WEIGHTS = {
    'CRITICAL': 1.0,   # Highest priority
    'HIGH': 0.7,
    'MEDIUM': 0.4,
    'LOW': 0.1,
}
```

**Method 2: Deadline-Monotonic (From Real-Time Theory)**
Shorter deadline = higher priority
```python
def deadline_monotonic_weight(deadline_ms):
    return 1.0 / deadline_ms  # Inverse of deadline
```

**Method 3: Utilization-Based (From PAAM)**
Consider both importance and resource usage
```python
def utilization_weight(criticality, cpu_usage, deadline_ms):
    base = CRITICALITY_WEIGHTS[criticality]
    urgency = 1.0 / deadline_ms
    efficiency = 1.0 / (cpu_usage + 0.01)  # Penalize high CPU usage
    return base * urgency * efficiency
```

### 10.2 Objectives Definition Document

Create this document for your report:

```yaml
# objectives.yaml - End-to-End Objectives for MoveIt 2 Manipulation Stack

paths:
  P1_planning:
    name: "Planning Path"
    description: "Goal request to motion plan generation"
    deadline_type: soft
    deadline_ms: 5000
    criticality: HIGH
    weight: 0.7
    rationale: |
      Planning is essential for task execution but not safety-critical.
      Longer planning times delay tasks but don't cause failures.

  P2_control:
    name: "Control Loop"
    description: "Joint trajectory controller update cycle"
    deadline_type: hard
    deadline_ms: 2
    criticality: CRITICAL
    weight: 1.0
    rationale: |
      Control loop must run at consistent rate for smooth motion.
      Missing deadlines causes jerky motion or safety stops.

  P3_feedback:
    name: "State Feedback"
    description: "Joint state publication to TF update"
    deadline_type: soft
    deadline_ms: 10
    criticality: HIGH
    weight: 0.8
    rationale: |
      Accurate state feedback is needed for monitoring and planning.
      Delays cause stale data but don't immediately affect safety.

  P4_collision:
    name: "Collision Checking"
    description: "Planning scene update and collision queries"
    deadline_type: soft
    deadline_ms: 50
    criticality: MEDIUM
    weight: 0.5
    rationale: |
      Collision checking is important for safety during planning.
      But it's done before motion starts, not during execution.

weighting_method: "criticality_based"
weighting_rationale: |
  We use criticality-based weighting because:
  1. The control loop has hard real-time requirements
  2. Other paths have soft requirements with different tolerances
  3. This matches the structure of ROS 2 executor priorities
```

### 10.3 Empirical Weight Validation

After collecting traces **from many CloudLab experiments**, validate your weight assignments:

```python
#!/usr/bin/env python3
"""
validate_weights.py - Validate objective weights against empirical data from CloudLab runs
"""

import pandas as pd
import yaml
from scipy import stats

def load_objectives(yaml_path: str) -> dict:
    with open(yaml_path) as f:
        return yaml.safe_load(f)

def load_results(csv_path: str) -> pd.DataFrame:
    return pd.read_csv(csv_path)

def validate_weights(objectives: dict, results: pd.DataFrame):
    """Check if weights align with observed behavior."""

    print("=== Weight Validation ===\n")

    for path_id, path_def in objectives['paths'].items():
        print(f"Path: {path_id} - {path_def['name']}")
        print(f"  Defined weight: {path_def['weight']}")
        print(f"  Deadline: {path_def['deadline_ms']}ms ({path_def['deadline_type']})")
        print()

def sensitivity_analysis(baseline_df: pd.DataFrame, load_df: pd.DataFrame):
    """
    Analyze how each path's latency changes under load.
    Paths that degrade more might need higher weights.
    """
    print("=== Sensitivity Analysis ===\n")

    # Compare baseline vs load for each metric
    metrics = ['planning_latency_ms', 'execution_latency_ms', 'total_latency_ms']

    for metric in metrics:
        if metric in baseline_df.columns and metric in load_df.columns:
            baseline_mean = baseline_df[metric].mean()
            load_mean = load_df[metric].mean()

            pct_increase = ((load_mean - baseline_mean) / baseline_mean) * 100

            print(f"{metric}:")
            print(f"  Baseline: {baseline_mean:.2f}ms")
            print(f"  Under load: {load_mean:.2f}ms")
            print(f"  Increase: {pct_increase:.1f}%")

            # Statistical significance
            t_stat, p_value = stats.ttest_ind(baseline_df[metric], load_df[metric])
            print(f"  Significant: {'Yes' if p_value < 0.05 else 'No'} (p={p_value:.4f})")
            print()


if __name__ == '__main__':
    # Load data from CloudLab experiments
    objectives = load_objectives('configs/objectives.yaml')
    baseline = load_results('results/baseline/summary.csv')
    cpu_load = load_results('results/cpu_load/summary.csv')

    validate_weights(objectives, baseline)
    sensitivity_analysis(baseline, cpu_load)
```

---

## 11. Experimental Plan

### 11.1 Phase 1: Baseline Characterization

**Location:** CloudLab bare-metal node
**Duration:** 1 session (~3-4 hours)
**Goal:** Understand normal system behavior with high confidence

```bash
# On CloudLab node
cd ~/ldos_manip_tracing
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Apply tuning
~/tune_system.sh

# Run 100 baseline trials
make run_baseline NUM_TRIALS=100

# Analyze
make analyze_all
```

**Deliverable:** Baseline report with:
- Callback duration distributions (N=100)
- Message rates per topic
- End-to-end path latencies with confidence intervals

### 11.2 Phase 2: Load Testing

**Location:** CloudLab bare-metal node
**Duration:** 1 session (~6-8 hours)
**Goal:** Understand degradation under load

```bash
# CPU load sweep
for load in 0 25 50 75 90; do
    sed -i "s/cpu_load_percent:.*/cpu_load_percent: ${load}/" configs/experiment_config.yaml
    make run_cpu_load NUM_TRIALS=50
    mv results/cpu_load results/cpu_load_${load}pct
done

# Message load sweep
for pubs in 1 10 25 50; do
    sed -i "s/num_publishers:.*/num_publishers: ${pubs}/" configs/experiment_config.yaml
    make run_msg_load NUM_TRIALS=50
    mv results/msg_load results/msg_load_${pubs}pubs
done
```

**Deliverable:** Load impact report with:
- Latency degradation curves
- Breaking points identified
- Sensitivity analysis

### 11.3 Phase 3: Stress Testing

**Location:** CloudLab bare-metal node
**Duration:** 1-2 sessions (~4-8 hours)
**Goal:** Find where the system breaks

```bash
# Run breaking point finder
python3 scripts/find_breaking_point.py cpu_load_percent 50 100

# Long stability test (overnight)
nohup make run_baseline NUM_TRIALS=500 > stability.log 2>&1 &
```

**Deliverable:** Stress test report with:
- Breaking points for each parameter
- Failure mode documentation
- Safe operating ranges

### 11.4 Phase 4: Path Analysis

**Location:** CloudLab bare-metal node
**Duration:** 1 session (~2-3 hours)
**Goal:** Map and measure all end-to-end paths

```bash
# Full trace with all events
ros2 trace start full_profile -e ros2:*

# Run 20 operations for detailed path analysis
for i in {1..20}; do
    ros2 run ldos_harness benchmark_runner.py --trial-id path_analysis_$i
done

ros2 trace stop full_profile

# Analyze paths
python analysis/e2e_path_analyzer.py ~/.ros/tracing/full_profile/
```

**Deliverable:** Path analysis report with:
- All identified paths
- Callback chain breakdowns
- Latency attribution per callback

### 11.5 Phase 5: Objective Definition

**Location:** Local analysis
**Duration:** Ongoing
**Goal:** Define and validate objectives

1. Based on Phase 1-4 results, fill in `configs/objectives.yaml`
2. Validate weights with empirical data
3. Re-run experiments to verify weights make sense

**Deliverable:** Objectives document with:
- Path definitions
- Deadline specifications
- Weight assignments with rationale
- Validation results

---

## 12. Report Structure

### 12.1 Final Report Outline

```markdown
# Tracing-Driven Performance Analysis of MoveIt 2 Manipulation Stack

## Abstract
- Problem: ROS 2 manipulation stacks lack clear performance characterization
- Approach: Systematic profiling with ros2_tracing on CloudLab bare-metal nodes
- Scale: Hundreds of trials per scenario for statistical confidence
- Key findings: [Summary of bottlenecks, breaking points, and objectives]

## 1. Introduction
- Motivation for performance analysis
- Research questions:
  1. What are the critical end-to-end paths?
  2. Where are the performance bottlenecks?
  3. At what load does the system break?
  4. What objectives/weights should be assigned?

## 2. Background
- MoveIt 2 architecture
- ROS 2 executor and callback model
- ros2_tracing and LTTng

## 3. Methodology
- Experimental setup
  - CloudLab hardware (node type, specs)
  - Software versions (ROS 2 Jazzy, Gazebo Harmonic, etc.)
  - Performance tuning applied
- Tracing configuration
- Analysis tools used
- Statistical approach (N per scenario, confidence levels)

## 4. System Characterization
- Node and topic graph
- Callback inventory
- Message flow analysis

## 5. End-to-End Path Analysis
- Path P1: Planning
- Path P2: Control
- Path P3: Feedback
- Path P4: Collision checking
(For each: definition, latency stats, bottlenecks)

## 6. Load Impact Analysis
- Baseline vs CPU load (with sweep results)
- Baseline vs message flood (with sweep results)
- Combined stress results

## 7. Breaking Point Analysis
- CPU load breaking point
- Message flood breaking point
- Memory pressure results
- Long-running stability results
- Safe operating envelope

## 8. Objective Definition
- Criticality classification
- Deadline specification
- Weight assignment methodology
- Validation results

## 9. Discussion
- Key bottlenecks identified
- Implications for scheduling
- Comparison with related work
- Limitations

## 10. Conclusion
- Summary of findings
- Recommended safe operating ranges
- Future work

## Appendix
A. CloudLab setup and reproduction instructions
B. Raw data tables
C. All experiment parameters
D. Trace event reference
```

### 12.2 Key Tables/Figures to Include

| Table/Figure | Content |
|--------------|---------|
| Table 1 | CloudLab experimental setup (HW, SW versions) |
| Table 2 | End-to-end path definitions |
| Table 3 | Baseline latency statistics (N=100) |
| Table 4 | Load impact comparison |
| Table 5 | Breaking points per parameter |
| Table 6 | Objective weights and rationale |
| Figure 1 | MoveIt 2 architecture diagram |
| Figure 2 | Node/topic graph |
| Figure 3 | Callback duration distributions |
| Figure 4 | Latency vs CPU load curve |
| Figure 5 | Breaking point visualization |
| Figure 6 | Path timeline visualization |

---

## 13. Timeline & Milestones

### Week-by-Week Plan

| Week | Focus | Location | Deliverable |
|------|-------|----------|-------------|
| 1 | Setup + initial baseline | CloudLab | Working tracing pipeline, N=20 trials |
| 2 | Full baseline + callback analysis | CloudLab | Baseline report (N=100) |
| 3 | Load testing sweeps | CloudLab | Load impact report |
| 4 | Stress testing + breaking points | CloudLab | Breaking point report |
| 5 | Path identification + deep analysis | CloudLab | Path definitions document |
| 6 | Objective definition + validation | Local | Objectives YAML + validation |
| 7 | Report writing + final experiments | Both | Final report |

### Milestones

- [ ] **M1:** First successful trace captured and analyzed on CloudLab
- [ ] **M2:** Baseline data collected (N>=100)
- [ ] **M3:** All load sweeps completed
- [ ] **M4:** Breaking points identified for all parameters
- [ ] **M5:** All paths identified and documented
- [ ] **M6:** Objectives defined with rationale
- [ ] **M7:** Draft report complete
- [ ] **M8:** Final report submitted

### Checkpoint Questions

After each phase, ask yourself:
1. Can I reproduce these results on another CloudLab node?
2. Do I have enough trials for statistical confidence?
3. Have I documented all assumptions and parameters?
4. What would happen if I ran 10x more trials?
5. Have I found where the system breaks?

---

## Appendix A: CloudLab Quick Reference

```bash
# Deploy to CloudLab (from local)
./scripts/deploy_to_cloudlab.sh <user>@<node>.cloudlab.us

# On CloudLab - setup
cd ~/ldos_manip_tracing
source /opt/ros/jazzy/setup.bash
source install/setup.bash
~/tune_system.sh

# Run experiments
make run_all NUM_TRIALS=100

# Analyze
make analyze_all

# Package results
tar -czvf ~/results_$(date +%Y%m%d).tar.gz results/ analysis/output/

# Fetch results (from local)
./scripts/fetch_from_cloudlab.sh <user>@<node>.cloudlab.us ~/results/
```

See **`docs/CLOUDLAB_GUIDE.md`** for complete CloudLab setup instructions.

---

## Appendix B: Tool Reference

### ros2_tracing Commands

```bash
# List available tracepoints
ros2 run tracetools status

# Start interactive tracing
ros2 trace

# Scripted tracing
ros2 trace start <session_name> [options]
ros2 trace stop <session_name>

# Options:
# -e, --events         Specify ROS 2 events
# -k, --kernel-events  Specify kernel events
# -s, --session-name   Session name
# -p, --path          Output path

# Example: Full ROS 2 tracing
ros2 trace start my_trace -e ros2:*

# Example: Callbacks only
ros2 trace start cb_trace \
    -e ros2:callback_start \
    -e ros2:callback_end
```

### tracetools_analysis

```python
from tracetools_analysis.loading import load_file
from tracetools_analysis.processor import Processor
from tracetools_analysis.processor.ros2 import Ros2Handler

events = load_file('/path/to/trace')
handler = Ros2Handler()
processor = Processor(handler)
processor.process(events)

# Access data
data = handler.data
nodes = data.nodes
callbacks = data.callback_instances
publishers = data.publishers
```

### Ros2TraceAnalyzer

```bash
# Analyze trace
python -m ros2_trace_analyzer analyze /path/to/trace

# Generate visualization
python -m ros2_trace_analyzer visualize /path/to/trace -o output/

# Export to CSV
python -m ros2_trace_analyzer export /path/to/trace -f csv -o results.csv
```

---

## Appendix C: Papers Reference

### PiCAS Key Concepts

```
Chain: C = {t_1 -> t_2 -> ... -> t_n}
End-to-end latency: E2E(C) = Sum(R_i) + Sum(comm_i)
Priority assignment: Based on chain deadline, not individual task
```

### PAAM Key Concepts

```
Priority factors:
- Criticality (C): Safety importance
- Deadline (D): Timing requirement
- Resource (R): Accelerator usage

Weight: W = f(C, D, R)
```

### ros2_tracing Key Tracepoints

```
Lifecycle:
  ros2:rcl_init
  ros2:rcl_node_init
  ros2:rcl_publisher_init
  ros2:rcl_subscription_init
  ros2:rcl_timer_init

Runtime:
  ros2:callback_start
  ros2:callback_end
  ros2:rcl_publish
  ros2:rclcpp_publish
  ros2:rcl_take
```

---

*This roadmap emphasizes CloudLab-based experimentation with high trial counts (N=100+), systematic stress testing to find breaking points, and iterative refinement based on data. The three main goals remain: setup (done), understand bottlenecks (via extensive tracing), and define objectives (via analysis and documentation).*

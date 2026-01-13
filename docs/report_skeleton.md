# Tracing-Driven Performance Analysis for ROS 2 Manipulation Stacks

**LDOS Undergraduate Research Report**

Authors: Aayush Baniya
Mentor: Rohit Dwivedula
Date: [DATE]

---

## Abstract

[2-3 sentences summarizing: (1) the problem, (2) the approach, (3) key findings]

---

## 1. Introduction

### 1.1 Motivation

ROS 2 manipulation stacks (MoveIt 2 + ros2_control + Gazebo) involve complex pipelines with multiple real-time and non-real-time components. Under load, these systems can degrade unpredictably because:

1. The system cannot prioritize important computations
2. Latency bottlenecks are difficult to identify without instrumentation
3. End-to-end timing relationships are not explicit

### 1.2 Research Questions

- **RQ1**: What are the dominant latency contributors in a ROS 2 manipulation pipeline?
- **RQ2**: How does CPU contention affect planning vs. execution latency?
- **RQ3**: How does DDS message flooding affect control loop timing?

### 1.3 Contributions

1. A reproducible measurement harness for ROS 2 manipulation performance
2. Quantitative characterization of latency distributions under load
3. Identification of bottleneck locations via callback-level tracing

---

## 2. Background

### 2.1 ROS 2 Architecture

[Brief overview of ROS 2 executor, DDS, callback model]

### 2.2 MoveIt 2 Pipeline

[Planning → Trajectory → Execution → Control]

### 2.3 LTTng and ros2_tracing

[How tracepoints work, what events are captured]

---

## 3. Methodology

### 3.1 Experimental Setup

| Component | Version/Config |
|-----------|---------------|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (gz-sim8) |
| MoveIt 2 | [version] |
| Tracing | ros2_tracing + LTTng |
| Hardware | [CloudLab node spec] |

### 3.2 Benchmark Task

**Canonical Task**: Move Panda end-effector from home pose to target pose:
- Target: (0.4, 0.2, 0.5) in base_link frame
- Planner: RRTConnect
- Planning time limit: 5.0s
- Velocity scaling: 50%

### 3.3 Load Scenarios

| Scenario | Description | Parameters |
|----------|-------------|------------|
| Baseline | No artificial load | - |
| CPU Load | stress-ng CPU workers | 4 workers, 80% target |
| Msg Load | DDS message flood | 4 publishers, 1000 Hz each, 1KB payload |

### 3.4 Metrics

| Metric ID | Name | Definition |
|-----------|------|------------|
| T1 | Planning Latency | Time from planning request to plan received |
| T2 | Execution Latency | Time from execute request to motion complete |
| T3 | Control Loop Jitter | Std dev of controller update intervals |
| T4 | Total E2E Latency | Time from goal sent to goal achieved |
| T5 | Sim Step Timing | Gazebo simulation step period consistency |

### 3.5 Trial Protocol

1. Warmup: 2 discarded trials
2. Data collection: 10 trials per scenario
3. Cooldown: 5 seconds between trials
4. Each trial: full tracing enabled

---

## 4. Results

### 4.1 Baseline Performance

**Table 4.1: Baseline Latency Distribution (N=10)**

| Metric | Mean (ms) | Std (ms) | P50 (ms) | P95 (ms) | P99 (ms) |
|--------|-----------|----------|----------|----------|----------|
| T1 Planning | [VALUE] | [VALUE] | [VALUE] | [VALUE] | [VALUE] |
| T2 Execution | [VALUE] | [VALUE] | [VALUE] | [VALUE] | [VALUE] |
| T4 Total E2E | [VALUE] | [VALUE] | [VALUE] | [VALUE] | [VALUE] |

**Figure 4.1**: [Histogram of baseline planning latency distribution]

### 4.2 CPU Load Impact

**Table 4.2: CPU Load vs. Baseline Comparison**

| Metric | Baseline Mean | CPU Load Mean | Delta (%) | p-value |
|--------|--------------|---------------|-----------|---------|
| T1 | [VALUE] | [VALUE] | [VALUE] | [VALUE] |
| T2 | [VALUE] | [VALUE] | [VALUE] | [VALUE] |
| T4 | [VALUE] | [VALUE] | [VALUE] | [VALUE] |

**Figure 4.2**: [Box plot comparing baseline vs. CPU load latencies]

### 4.3 Message Load Impact

**Table 4.3: Message Load vs. Baseline Comparison**

| Metric | Baseline Mean | Msg Load Mean | Delta (%) | p-value |
|--------|--------------|---------------|-----------|---------|
| T1 | [VALUE] | [VALUE] | [VALUE] | [VALUE] |
| T2 | [VALUE] | [VALUE] | [VALUE] | [VALUE] |
| T4 | [VALUE] | [VALUE] | [VALUE] | [VALUE] |

**Figure 4.3**: [Box plot comparing baseline vs. message load latencies]

### 4.4 Callback Analysis

**Table 4.4: Top 5 Callbacks by Total Duration**

| Callback | Count | Mean (ms) | Total (ms) | % of Total |
|----------|-------|-----------|------------|------------|
| [NAME] | [VALUE] | [VALUE] | [VALUE] | [VALUE] |
| ... | ... | ... | ... | ... |

**Figure 4.4**: [Flame graph or timeline of callback execution]

---

## 5. Discussion

### 5.1 Interpretation of Results

[What do the numbers mean? Which component is the bottleneck?]

### 5.2 Bottleneck Identification

Based on callback analysis:
1. [First bottleneck]
2. [Second bottleneck]

### 5.3 Implications for Scheduling

[How could priority-based scheduling help?]

### 5.4 Limitations

1. Simulation vs. real hardware timing differences
2. Single benchmark task (may not generalize)
3. LTTng overhead on measurements
4. [Other limitations]

---

## 6. Future Work

1. Extend to real Panda hardware
2. Implement and evaluate priority-based executor
3. Test with more complex manipulation tasks
4. Measure additional metrics (memory, network)

---

## 7. Conclusion

[2-3 sentences summarizing findings and significance]

---

## References

[1] MoveIt 2 Documentation. https://moveit.ros.org/

[2] Casini et al., "Response-Time Analysis of ROS 2 Processing Chains Under Reservation-Based Scheduling", ECRTS 2019.

[3] Blass et al., "A ROS 2 Response-Time Analysis Exploiting Starvation Freedom and Execution-Time Variance", RTAS 2021.

[4] ros2_tracing. https://github.com/ros2/ros2_tracing

---

## Appendix A: Raw Data Tables

[Include full CSV data or links to data files]

## Appendix B: Trace Event Counts

[Summary of LTTng event counts per scenario]

## Appendix C: Reproduction Instructions

```bash
# Clone and setup
git clone [repo]
cd ldos_manip_tracing
make setup

# Run full experiment suite
make run_all NUM_TRIALS=10

# Analyze and generate report
make analyze_all report
```

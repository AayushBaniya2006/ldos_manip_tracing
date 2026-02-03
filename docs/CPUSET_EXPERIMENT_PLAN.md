# CPUSET Experiment Execution Plan

## Overview

This plan guides you through running the new cpuset-limited experiments on CloudLab to measure how CPU resource limitation affects ROS 2 manipulation performance.

---

## Pre-requisites

- [ ] CloudLab node provisioned (Ubuntu 24.04)
- [ ] Repository cloned and bootstrapped (`make bootstrap`)
- [ ] Logged out and back in (for tracing group)
- [ ] Smoke test passes (`make smoke_test`)

---

## Phase 1: Verify cpuset Support

**Goal:** Confirm cgroups v2 cpuset works on your CloudLab node.

```bash
cd ~/ldos_manip_tracing

# Run cpuset support checker
make check_cpuset
```

**Expected Output:**
```
✓ cgroups v2 detected at /sys/fs/cgroup
✓ cpuset controller available
✓ user.slice exists
✓ systemd --user session active
✓ AllowedCPUs property works
✓ System has N CPUs available
```

**If check fails:**
- Ensure you're on Ubuntu 24.04 (cgroups v2 default)
- Run `loginctl enable-linger $USER` if systemd --user fails
- Check kernel: `uname -r` (should be 6.x+)

---

## Phase 2: Quick Validation Run

**Goal:** Verify the cpuset scenario works with a short test.

```bash
# Run 3 trials with default 2 CPUs for ROS stack
make run_cpuset NUM_TRIALS=3
```

**What to watch:**
- Gazebo launches on unlimited CPUs
- ROS stack launches on limited CPUs (last 2)
- Trials complete successfully

**Monitor in another terminal:**
```bash
# Watch CPU usage
htop

# Or check process CPU affinity
watch -n 1 'ps -eo pid,comm,psr | grep -E "(gz|move_group|controller)"'
```

**Expected:** Most gz-sim processes on CPUs 0-N, move_group on last 2 CPUs.

---

## Phase 3: Baseline Comparison

**Goal:** Establish baseline performance without CPU limitation.

```bash
# Run baseline (no load, all CPUs available)
make run_baseline NUM_TRIALS=10
```

**Record these metrics:**
- Planning latency (T1): ~15ms expected
- Execution latency (T2): ~500-700ms expected
- Success rate: 100% expected

---

## Phase 4: cpuset Sweep Experiment

**Goal:** Find the breaking point where limited CPUs cause degradation.

```bash
# Run CPU sweep: 1, 2, 4 CPUs for ROS stack
make sweep_cpuset NUM_TRIALS=10
```

**This runs experiments with:**
- 1 CPU for ROS stack (extreme constraint)
- 2 CPUs for ROS stack (moderate constraint)
- 4 CPUs for ROS stack (light constraint)

**Results saved to:** `results/cpuset_sweep_<timestamp>/`

---

## Phase 5: Custom CPU Count

**Goal:** Test specific CPU configurations.

```bash
# Test with 1 CPU (expect failures/degradation)
ROS_CPU_COUNT=1 make run_cpuset NUM_TRIALS=10

# Test with 4 CPUs (should be similar to baseline)
ROS_CPU_COUNT=4 make run_cpuset NUM_TRIALS=10
```

---

## Phase 6: Analysis

**Goal:** Analyze results and compare scenarios.

```bash
# Process all traces and generate summary
make analyze_all

# View summary
make report

# Generate plots
make plot_all
```

**Compare:**
- Baseline vs cpuset_limited (2 CPU)
- cpuset_limited (1 CPU) vs (2 CPU) vs (4 CPU)

**Key questions:**
1. At what CPU count does success rate drop?
2. How does planning latency (T1) scale with CPU count?
3. Does control loop jitter (T3) increase under constraint?

---

## Expected Results Table

| Scenario | ROS CPUs | Success Rate | Planning (ms) | Notes |
|----------|----------|--------------|---------------|-------|
| baseline | All | 100% | ~15 | Reference |
| cpuset_limited | 4 | ~100% | ~15-20 | Similar to baseline |
| cpuset_limited | 2 | ~95-100% | ~20-30 | Slight degradation |
| cpuset_limited | 1 | ~70-90% | ~30-50+ | Significant impact |

---

## Troubleshooting

### "AllowedCPUs failed"
```bash
# Enable user lingering
loginctl enable-linger $USER

# Restart user session
systemctl --user daemon-reload
```

### Gazebo dies during cpuset launch
```bash
# Kill all and retry
pkill -9 -f gazebo; pkill -9 -f ros2
sleep 10
make run_cpuset NUM_TRIALS=1
```

### Processes not on expected CPUs
```bash
# Verify cgroup assignment
cat /proc/<PID>/cgroup
cat /sys/fs/cgroup/user.slice/user-$(id -u).slice/*/cpuset.cpus
```

---

## Data Collection Checklist

- [ ] Phase 1: cpuset support verified
- [ ] Phase 2: Quick validation passed
- [ ] Phase 3: Baseline results collected (10 trials)
- [ ] Phase 4: CPU sweep completed (1, 2, 4 CPUs)
- [ ] Phase 5: Custom configurations tested (optional)
- [ ] Phase 6: Analysis and plots generated

---

## Output Files

After experiments:

```
results/
├── baseline/                    # Baseline experiment results
├── cpuset_limited/              # Default cpuset results
├── cpuset_sweep_<timestamp>/    # Sweep results
│   ├── 1cpu/                    # 1 CPU results
│   ├── 2cpu/                    # 2 CPU results
│   ├── 4cpu/                    # 4 CPU results
│   ├── sweep_config.json
│   ├── sweep_results.csv
│   └── sweep_summary.txt

analysis/output/
├── combined_summary.csv
├── plots/
│   ├── latency_boxplot.png
│   ├── cpu_utilization.png
│   └── ...
```

---

## Next Steps

After collecting data:

1. **Write findings** - Document which CPU count causes degradation
2. **Identify bottleneck paths** - Which T1-T5 metrics degrade first?
3. **Compare with DDS flood** - Is CPU or message flooding more impactful?
4. **Consider scheduling** - Would PiCAS-style priority scheduling help?

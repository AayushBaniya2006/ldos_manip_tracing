# CloudLab Deployment Guide

Complete end-to-end instructions for running the LDOS tracing harness on CloudLab.

---

## Overview

CloudLab provides dedicated bare-metal nodes ideal for reproducible performance experiments. This guide covers:
1. Creating/using a CloudLab profile
2. Instantiating an experiment
3. Setting up the node
4. Transferring and running the harness
5. Collecting results

---

## Part 1: CloudLab Profile

### Option A: Use Existing Ubuntu 24.04 Profile

1. Go to [CloudLab](https://cloudlab.us)
2. Click **Experiments** → **Start Experiment**
3. Select profile: `small-lan` or any single-node profile
4. Choose **Ubuntu 24.04** as the OS image
5. Select hardware type (recommended: `c6525-100g`, `xl170`, or `c220g5`)

### Option B: Create Custom Profile (Recommended)

Create a profile with ROS 2 pre-installed to save setup time.

**Profile script (`ldos-ros2-jazzy.py`):**

```python
"""LDOS ROS 2 Jazzy Tracing Profile

Single bare-metal node with Ubuntu 24.04 and ROS 2 Jazzy pre-installed.
Optimized for robotics performance experiments.
"""

import geni.portal as portal
import geni.rspec.pg as rspec

# Create a portal context
pc = portal.Context()

# Create a Request object
request = pc.makeRequestRSpec()

# Define parameters
pc.defineParameter("nodeType", "Hardware Type",
                   portal.ParameterType.NODETYPE, "c6525-100g",
                   longDescription="Select hardware. c6525-100g or xl170 recommended.")

pc.defineParameter("nodeCount", "Number of Nodes",
                   portal.ParameterType.INTEGER, 1,
                   longDescription="Number of nodes (1 for single experiments)")

# Get parameters
params = pc.bindParameters()

# Create node
node = request.RawPC("node0")
node.hardware_type = params.nodeType
node.disk_image = "urn:publicid:IDN+emulab.net+image+emulab-ops//UBUNTU24-64-STD"

# Install script runs at boot
node.addService(rspec.Execute(shell="bash", command="""
#!/bin/bash
set -e

# Log everything
exec > /var/log/ldos-setup.log 2>&1

echo "=== LDOS Setup Starting ==="
date

# Update system
apt-get update
apt-get upgrade -y

# Install ROS 2 Jazzy
apt-get install -y software-properties-common curl gnupg lsb-release

# Add ROS 2 repository
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

apt-get update

# Install ROS 2 Jazzy desktop + dev tools
apt-get install -y ros-jazzy-desktop ros-jazzy-ros-dev-tools

# Install MoveIt 2
apt-get install -y ros-jazzy-moveit

# Install ros2_control
apt-get install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control

# Install Gazebo Harmonic
apt-get install -y ros-jazzy-ros-gz

# Install tracing tools
apt-get install -y ros-jazzy-ros2-tracing lttng-tools liblttng-ust-dev python3-babeltrace python3-lttng

# Install stress-ng for load testing
apt-get install -y stress-ng cpupower-gui linux-tools-common linux-tools-generic

# Install Python dependencies
pip3 install pandas matplotlib pyyaml --break-system-packages

# Create tracing group and add default users
groupadd -f tracing
for user in /users/*; do
    username=$(basename $user)
    if id "$username" &>/dev/null; then
        usermod -aG tracing $username
    fi
done

# Set kernel parameters for better tracing
echo 'kernel.perf_event_paranoid = -1' >> /etc/sysctl.conf
echo 'kernel.kptr_restrict = 0' >> /etc/sysctl.conf
sysctl -p

# Set CPU governor to performance
for gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance > $gov 2>/dev/null || true
done

echo "=== LDOS Setup Complete ==="
date
"""))

# Output
pc.printRequestRSpec(request)
```

**To create the profile:**
1. Go to **Experiments** → **Create Experiment Profile**
2. Choose **Git Repo** and paste the script, or upload directly
3. Name it: `ldos-ros2-jazzy`
4. Click **Create**

---

## Part 2: Instantiate Experiment

1. **Start Experiment**
   - Go to **Experiments** → **Start Experiment**
   - Select your profile (`ldos-ros2-jazzy` or `small-lan`)
   - Click **Next**

2. **Configure**
   - Name: `ldos-tracing-exp-001`
   - Project: Select your project
   - Cluster: Select site (Utah, Wisconsin, Clemson - Utah often has best availability)

3. **Schedule**
   - Duration: 16 hours minimum (experiments take ~1 hour, plus setup)
   - Click **Finish**

4. **Wait for Ready**
   - Status will change from "created" → "provisioning" → "booting" → "ready"
   - Takes 10-20 minutes

5. **Get SSH Access**
   - Once ready, click on experiment
   - Note the SSH command: `ssh <username>@<node>.cloudlab.us`

---

## Part 3: Transfer Harness to CloudLab

### From your local machine:

```bash
# Set your CloudLab node address
CLOUDLAB_NODE="your-username@node0.your-experiment.your-project.cloudlab.us"

# Create tarball of harness (excluding empty directories)
cd ~/ldos_manip_tracing
tar -czvf /tmp/ldos_harness.tar.gz \
    --exclude='traces/*' \
    --exclude='results/*' \
    --exclude='analysis/output/*' \
    --exclude='build' \
    --exclude='install' \
    --exclude='log' \
    .

# Transfer to CloudLab
scp /tmp/ldos_harness.tar.gz ${CLOUDLAB_NODE}:~/
```

### On CloudLab node:

```bash
# SSH to node
ssh ${CLOUDLAB_NODE}

# Extract harness
mkdir -p ~/ldos_manip_tracing
cd ~/ldos_manip_tracing
tar -xzvf ~/ldos_harness.tar.gz

# Verify files
ls -la
```

---

## Part 4: CloudLab Node Setup

### 4.1 Verify Installation

```bash
# Check ROS 2
source /opt/ros/jazzy/setup.bash
ros2 --version

# Check Gazebo
gz sim --version

# Check LTTng
lttng --version

# Check tracing group
groups | grep tracing
# If not in group, add yourself:
sudo usermod -aG tracing $USER
newgrp tracing
```

### 4.2 System Tuning for Experiments

```bash
# Create tuning script
cat > ~/tune_system.sh << 'EOF'
#!/bin/bash
# CloudLab performance tuning for LDOS experiments

echo "=== Applying CloudLab Performance Tuning ==="

# 1. Set CPU governor to performance (all cores)
for gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance | sudo tee $gov > /dev/null
done
echo "[OK] CPU governor set to performance"

# 2. Disable CPU frequency scaling
sudo cpupower frequency-set -g performance 2>/dev/null || true
echo "[OK] CPU frequency scaling disabled"

# 3. Disable turbo boost for consistent timing (Intel)
if [ -f /sys/devices/system/cpu/intel_pstate/no_turbo ]; then
    echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo > /dev/null
    echo "[OK] Intel Turbo Boost disabled"
fi

# 4. Set kernel parameters for tracing
sudo sysctl -w kernel.perf_event_paranoid=-1
sudo sysctl -w kernel.kptr_restrict=0
echo "[OK] Kernel parameters set for tracing"

# 5. Increase file descriptor limits
ulimit -n 65535
echo "[OK] File descriptor limit increased"

# 6. Disable swap to prevent timing variability
sudo swapoff -a
echo "[OK] Swap disabled"

# 7. Set real-time priority limits
echo "* soft rtprio 99" | sudo tee -a /etc/security/limits.conf > /dev/null
echo "* hard rtprio 99" | sudo tee -a /etc/security/limits.conf > /dev/null
echo "[OK] RT priority limits set"

# 8. Check NUMA topology
echo ""
echo "=== System Info ==="
echo "CPUs: $(nproc)"
echo "Memory: $(free -h | grep Mem | awk '{print $2}')"
lscpu | grep -E "Model name|Socket|Core|Thread"
numactl --hardware 2>/dev/null || echo "NUMA: single node"

echo ""
echo "=== Tuning Complete ==="
EOF

chmod +x ~/tune_system.sh
~/tune_system.sh
```

### 4.3 Build Workspace

```bash
cd ~/ldos_manip_tracing
source /opt/ros/jazzy/setup.bash

# Make scripts executable
chmod +x scripts/*.sh
chmod +x src/ldos_harness/scripts/*.py
chmod +x analysis/*.py

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash
```

### 4.4 Verify Setup

```bash
cd ~/ldos_manip_tracing
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run acceptance tests
make acceptance_test

# Run smoke test
make smoke_test
```

---

## Part 5: Run Experiments

### 5.1 Pre-flight Checklist

```bash
# Run before each experiment session
cd ~/ldos_manip_tracing
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Apply tuning
~/tune_system.sh

# Check disk space (need >10GB)
df -h ~

# Verify no other users/processes
who
top -bn1 | head -20

# Kill any stale ROS processes
pkill -f ros2 2>/dev/null || true
pkill -f gz 2>/dev/null || true

# Clear old traces (optional, saves space)
# rm -rf traces/* results/*
```

### 5.2 Run Full Experiment Suite

```bash
cd ~/ldos_manip_tracing
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run all scenarios (baseline + cpu_load + msg_load), N=10 each
# Estimated time: 45-60 minutes
make run_all NUM_TRIALS=10 2>&1 | tee experiment_$(date +%Y%m%d_%H%M%S).log
```

### 5.3 Run Individual Scenarios (if needed)

```bash
# Just baseline
make run_baseline NUM_TRIALS=10

# Just CPU load
make run_cpu_load NUM_TRIALS=10

# Just message load
make run_msg_load NUM_TRIALS=10
```

### 5.4 Monitor Progress

```bash
# In another terminal, watch progress
watch -n 5 'ls -la ~/ldos_manip_tracing/results/*/baseline_*.json 2>/dev/null | wc -l'

# Or tail the log
tail -f ~/ldos_manip_tracing/experiment_*.log
```

---

## Part 6: Analyze and Collect Results

### 6.1 Run Analysis

```bash
cd ~/ldos_manip_tracing
source install/setup.bash

# Analyze all results
make analyze_all

# Generate plots
python3 analysis/plot_results.py \
    --input analysis/output/combined_summary.csv \
    --output-dir analysis/output/plots

# Generate report
make report
```

### 6.2 Quick Results Summary

```bash
# Print summary statistics
python3 << 'EOF'
import pandas as pd
df = pd.read_csv('analysis/output/combined_summary.csv')
print("\n=== EXPERIMENT RESULTS ===\n")
print(df.groupby('scenario').agg({
    'status': lambda x: f"{(x=='success').sum()}/{len(x)} success",
    'planning_latency_ms': ['mean', 'std'],
    'total_latency_ms': ['mean', 'std']
}).round(2))
EOF
```

### 6.3 Transfer Results Back

**On CloudLab node:**
```bash
cd ~/ldos_manip_tracing

# Create results archive
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
tar -czvf ~/ldos_results_${TIMESTAMP}.tar.gz \
    results/ \
    analysis/output/ \
    experiment_*.log \
    configs/
```

**On local machine:**
```bash
CLOUDLAB_NODE="your-username@node0.your-experiment.your-project.cloudlab.us"

# Download results
scp ${CLOUDLAB_NODE}:~/ldos_results_*.tar.gz ~/Downloads/

# Extract
cd ~/Downloads
tar -xzvf ldos_results_*.tar.gz
```

### 6.4 Transfer Traces (if needed)

Traces are large (can be 1-10GB). Only transfer if you need detailed analysis.

```bash
# On CloudLab - compress traces
cd ~/ldos_manip_tracing
tar -cvf ~/ldos_traces_${TIMESTAMP}.tar traces/

# On local machine
scp ${CLOUDLAB_NODE}:~/ldos_traces_*.tar ~/Downloads/
```

---

## Part 7: CloudLab Best Practices

### Experiment Duration
- Request at least 16 hours (experiments take ~1 hour + setup + buffer)
- Extend if needed: **Experiment** → **Extend**

### Resource Cleanup
- CloudLab nodes are shared resources
- Terminate when done: **Experiment** → **Terminate**
- Don't leave experiments running unused

### Reproducibility
- Record the CloudLab profile and node type
- Save the experiment manifest
- Include hardware specs in your results:
```bash
# Add to metadata
lscpu > ~/ldos_manip_tracing/results/hardware_info.txt
free -h >> ~/ldos_manip_tracing/results/hardware_info.txt
uname -a >> ~/ldos_manip_tracing/results/hardware_info.txt
```

### Recommended Hardware Types

| Type | CPUs | RAM | Notes |
|------|------|-----|-------|
| `c6525-100g` | 32 cores | 128 GB | AMD EPYC, excellent for isolation |
| `xl170` | 10 cores | 64 GB | Intel Xeon, good availability |
| `c220g5` | 40 cores | 192 GB | High core count, Intel |
| `r320` | 8 cores | 16 GB | Older, but often available |

### Troubleshooting

**"Permission denied" for LTTng:**
```bash
sudo usermod -aG tracing $USER
newgrp tracing
# Or logout and login again
```

**Gazebo won't start:**
```bash
# Check if X forwarding needed
export DISPLAY=:0
# Or run fully headless (default in harness)
```

**Node runs out of disk:**
```bash
# Check usage
df -h
# Clear old traces
rm -rf ~/ldos_manip_tracing/traces/*
```

**Experiment terminated unexpectedly:**
- CloudLab auto-terminates at scheduled end time
- Always extend before deadline
- Results in /tmp are lost; always save to home directory

---

## Quick Reference Commands

```bash
# SSH to CloudLab
ssh <user>@<node>.cloudlab.us

# Setup (first time)
cd ~/ldos_manip_tracing && make setup

# Tune system (each session)
~/tune_system.sh

# Run experiments
make run_all NUM_TRIALS=10

# Analyze
make analyze_all report

# Package results
tar -czvf ~/ldos_results_$(date +%Y%m%d).tar.gz results/ analysis/output/
```

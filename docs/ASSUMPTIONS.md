# Assumptions and Reproducibility Notes

This document lists all assumptions made in the LDOS tracing harness that could affect reproducibility. Each assumption is categorized by risk level and includes mitigation strategies.

---

## Critical Assumptions (Must Verify)

### A1. ROS 2 Jazzy Installation
**Assumption**: ROS 2 Jazzy is installed at `/opt/ros/jazzy/`
**Risk**: HIGH - Scripts will fail if ROS 2 is not found
**Verification**:
```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
# Expected: ros2 0.x.x (Jazzy)
```
**Mitigation**: The setup script checks for ROS_DISTRO

### A2. Gazebo Harmonic (gz-sim8)
**Assumption**: Gazebo Harmonic is installed and `gz sim` works
**Risk**: HIGH - Simulation will not start
**Verification**:
```bash
gz sim --version
# Expected: Gazebo Sim, version 8.x.x
```
**Mitigation**: Setup script verifies ros_gz_sim package

### A3. gz_ros2_control Plugin
**Assumption**: The `gz_ros2_control` package is installed and the plugin can be loaded
**Risk**: HIGH - Controllers will not spawn
**Verification**:
```bash
ros2 pkg list | grep gz_ros2_control
# Should show: gz_ros2_control
```
**Known Issue**: On some systems, the plugin path may not be found. Set:
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
```

### A4. LTTng Tracing Permissions
**Assumption**: User has permission to create LTTng sessions
**Risk**: HIGH - Tracing will fail silently or with permission errors
**Verification**:
```bash
groups | grep tracing
lttng create test_session && lttng destroy test_session
```
**Mitigation**: Add user to tracing group:
```bash
sudo usermod -aG tracing $USER
# Then log out and back in
```

### A5. MoveIt 2 Action Interface
**Assumption**: MoveIt's move_group provides `/move_action` action server
**Risk**: HIGH - Benchmark will timeout waiting for action
**Verification**:
```bash
# After launching full_stack:
ros2 action list | grep move_action
# Expected: /move_action
```
**Note**: The action name may vary between MoveIt versions. Check with `ros2 action list`.

---

## Medium-Risk Assumptions

### A6. Panda URDF Geometry
**Assumption**: Simplified Panda geometry is sufficient for benchmarking
**Risk**: MEDIUM - Collision checking may differ from real Panda
**Impact**: Planning times may differ from production configs
**Mitigation**: For production use, replace `panda_gz.urdf.xacro` with official `franka_description`

### A7. Controller Update Rate
**Assumption**: 500 Hz controller update rate is achievable
**Risk**: MEDIUM - On slower systems, controller may not keep up
**Verification**:
```bash
# After launching, check actual rate:
ros2 topic hz /joint_states
# Should show ~500 Hz
```
**Mitigation**: Reduce rate in `panda_controllers.yaml` if needed

### A8. Simulation Real-Time Factor
**Assumption**: Gazebo runs at real-time (RTF >= 1.0)
**Risk**: MEDIUM - Slow simulation affects timing measurements
**Verification**:
```bash
# Check Gazebo stats:
gz topic -e -t /stats
# Look for real_time_factor
```
**Mitigation**: Reduce physics complexity or use faster hardware

### A9. stress-ng Availability
**Assumption**: `stress-ng` is installed for CPU load tests
**Risk**: MEDIUM - CPU load scenario will fail
**Verification**:
```bash
stress-ng --version
```
**Mitigation**: `sudo apt install stress-ng`

### A10. Python Dependencies
**Assumption**: babeltrace2, pandas, matplotlib are installed
**Risk**: MEDIUM - Analysis will fail
**Verification**:
```bash
python3 -c "import bt2, pandas, matplotlib"
```
**Mitigation**: `pip install babeltrace2 pandas matplotlib`

---

## Low-Risk Assumptions

### A11. CPU Affinity
**Assumption**: System allows CPU affinity control via `taskset`
**Risk**: LOW - Load tests may not be isolated
**Mitigation**: Load will still work, just without core pinning

### A12. Network Configuration
**Assumption**: Default DDS configuration (localhost, no multicast issues)
**Risk**: LOW - May affect msg_load scenario
**Mitigation**: Set `ROS_LOCALHOST_ONLY=1` if issues occur

### A13. Available Disk Space
**Assumption**: At least 10GB free for traces
**Risk**: LOW - Traces may be truncated
**Verification**: `df -h ~/ldos_manip_tracing`

### A14. System Clock Stability
**Assumption**: System clock is stable (not drifting significantly)
**Risk**: LOW - Timing measurements may drift
**Mitigation**: Use NTP: `sudo systemctl status chronyd`

---

## Environment Variables

These environment variables affect behavior:

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_DISTRO` | jazzy | Must be "jazzy" |
| `RMW_IMPLEMENTATION` | (system default) | DDS implementation |
| `ROS_LOCALHOST_ONLY` | 0 | Set to 1 for isolated testing |
| `GZ_SIM_SYSTEM_PLUGIN_PATH` | (system default) | Path to Gazebo plugins |
| `AMENT_PREFIX_PATH` | (from setup.bash) | ROS 2 package paths |

---

## Hardware Requirements

### Minimum (for smoke tests)
- CPU: 4 cores
- RAM: 8 GB
- Disk: 20 GB free
- GPU: Not required (headless mode)

### Recommended (for full experiments)
- CPU: 8+ cores (to isolate load from ROS)
- RAM: 16 GB
- Disk: 50 GB free (for traces)
- GPU: Optional (for visualization)

---

## Known Platform Issues

### CloudLab Specific
1. **Kernel tracing**: May require `sudo` for kernel events
2. **CPU governor**: Set to "performance" for consistent results:
   ```bash
   sudo cpupower frequency-set -g performance
   ```
3. **Numa**: Pin processes to single NUMA node for consistency

### Ubuntu 24.04 Specific
1. **AppArmor**: May block LTTng. Check with `aa-status`
2. **Snap confinement**: If ROS installed via snap, tracing may not work

---

## Version Pinning

For exact reproducibility, pin these versions:

```bash
# Record versions
ros2 pkg xml moveit_ros_move_group | grep version
ros2 pkg xml gz_ros2_control | grep version
dpkg -l | grep lttng
pip freeze | grep -E "babeltrace|pandas"
```

Include version info in experiment metadata (done automatically by `run_single_trial.sh`).

---

## Checklist Before Running Experiments

```bash
# Run this checklist before collecting data:

# 1. Verify ROS 2
[ ] ros2 doctor  # Should show no errors

# 2. Verify Gazebo
[ ] gz sim --version

# 3. Verify tracing
[ ] lttng create test && lttng destroy test

# 4. Run smoke test
[ ] make smoke_test

# 5. Check disk space
[ ] df -h ~/ldos_manip_tracing  # >10GB free

# 6. Set CPU governor (optional but recommended)
[ ] sudo cpupower frequency-set -g performance

# 7. Close unnecessary applications
[ ] # Reduce background load
```

---

## Troubleshooting

### "MoveGroup action not found"
1. Check if move_group started: `ros2 node list | grep move_group`
2. Increase `moveit_delay` in launch file
3. Check for MoveIt errors in terminal output

### "LTTng session creation failed"
1. Check group membership: `groups | grep tracing`
2. Check if LTTng daemon running: `lttng-sessiond --version`
3. Try with sudo: `sudo lttng create test`

### "Controller failed to load"
1. Check controller_manager: `ros2 control list_controllers`
2. Verify plugin path: `echo $GZ_SIM_SYSTEM_PLUGIN_PATH`
3. Check Gazebo console for plugin errors

### "Benchmark timeout"
1. Increase timeout: `--timeout 120.0`
2. Check if robot is in valid start state
3. Verify planning scene is not in collision

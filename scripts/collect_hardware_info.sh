#!/bin/bash
# collect_hardware_info.sh - Collect system hardware information for reproducibility
#
# This script gathers comprehensive system specifications for inclusion in
# research reports, ensuring experiments can be reproduced on similar hardware.
#
# Usage:
#   ./scripts/collect_hardware_info.sh [output_dir]
#
# Outputs:
#   - hardware_info.json - Machine-readable specifications
#   - hardware_info.tex - LaTeX table for reports
#   - hardware_info.md - Markdown summary

set -euo pipefail

# =============================================================================
# CONFIGURATION
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
OUTPUT_DIR="${1:-$WS_ROOT/analysis/output}"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

# =============================================================================
# FUNCTIONS
# =============================================================================

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_section() {
    echo -e "${BLUE}=== $1 ===${NC}"
}

# Get CPU info
get_cpu_info() {
    if [ -f /proc/cpuinfo ]; then
        # Linux
        CPU_MODEL=$(grep -m1 "model name" /proc/cpuinfo | cut -d: -f2 | xargs)
        CPU_CORES=$(grep -c "^processor" /proc/cpuinfo)
        CPU_THREADS=$(nproc 2>/dev/null || echo "$CPU_CORES")

        # Get physical cores (may differ from logical)
        CPU_PHYSICAL=$(grep "^cpu cores" /proc/cpuinfo | head -1 | cut -d: -f2 | xargs || echo "$CPU_CORES")

        # Get CPU frequency
        if [ -f /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq ]; then
            CPU_FREQ_MHZ=$(($(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq) / 1000))
        else
            CPU_FREQ_MHZ=$(grep -m1 "cpu MHz" /proc/cpuinfo | cut -d: -f2 | xargs | cut -d. -f1)
        fi
    elif command -v sysctl &>/dev/null; then
        # macOS
        CPU_MODEL=$(sysctl -n machdep.cpu.brand_string 2>/dev/null || echo "Unknown")
        CPU_CORES=$(sysctl -n hw.physicalcpu 2>/dev/null || echo "0")
        CPU_THREADS=$(sysctl -n hw.logicalcpu 2>/dev/null || echo "0")
        CPU_PHYSICAL=$CPU_CORES
        CPU_FREQ_MHZ=$(sysctl -n hw.cpufrequency 2>/dev/null | awk '{printf "%.0f", $1/1000000}' || echo "0")
    else
        CPU_MODEL="Unknown"
        CPU_CORES="0"
        CPU_THREADS="0"
        CPU_PHYSICAL="0"
        CPU_FREQ_MHZ="0"
    fi
}

# Get memory info
get_memory_info() {
    if [ -f /proc/meminfo ]; then
        # Linux
        MEM_TOTAL_KB=$(grep "MemTotal" /proc/meminfo | awk '{print $2}')
        MEM_TOTAL_GB=$(echo "scale=1; $MEM_TOTAL_KB / 1024 / 1024" | bc)
        MEM_FREE_KB=$(grep "MemAvailable" /proc/meminfo | awk '{print $2}' || grep "MemFree" /proc/meminfo | awk '{print $2}')
        MEM_FREE_GB=$(echo "scale=1; $MEM_FREE_KB / 1024 / 1024" | bc)
    elif command -v sysctl &>/dev/null; then
        # macOS
        MEM_TOTAL_BYTES=$(sysctl -n hw.memsize 2>/dev/null || echo "0")
        MEM_TOTAL_GB=$(echo "scale=1; $MEM_TOTAL_BYTES / 1024 / 1024 / 1024" | bc)
        MEM_FREE_GB="N/A"
    else
        MEM_TOTAL_GB="0"
        MEM_FREE_GB="0"
    fi
}

# Get disk info
get_disk_info() {
    # Get workspace disk info
    if command -v df &>/dev/null; then
        DISK_INFO=$(df -h "$WS_ROOT" 2>/dev/null | tail -1)
        DISK_SIZE=$(echo "$DISK_INFO" | awk '{print $2}')
        DISK_AVAIL=$(echo "$DISK_INFO" | awk '{print $4}')
        DISK_DEVICE=$(echo "$DISK_INFO" | awk '{print $1}')
    else
        DISK_SIZE="Unknown"
        DISK_AVAIL="Unknown"
        DISK_DEVICE="Unknown"
    fi
}

# Get OS info
get_os_info() {
    if [ -f /etc/os-release ]; then
        # Linux
        OS_NAME=$(grep "^NAME=" /etc/os-release | cut -d= -f2 | tr -d '"')
        OS_VERSION=$(grep "^VERSION=" /etc/os-release | cut -d= -f2 | tr -d '"')
        OS_ID=$(grep "^ID=" /etc/os-release | cut -d= -f2 | tr -d '"')
    elif command -v sw_vers &>/dev/null; then
        # macOS
        OS_NAME="macOS"
        OS_VERSION=$(sw_vers -productVersion)
        OS_ID="darwin"
    else
        OS_NAME=$(uname -s)
        OS_VERSION=$(uname -r)
        OS_ID="unknown"
    fi

    KERNEL_VERSION=$(uname -r)
    ARCH=$(uname -m)
}

# Get ROS info
get_ros_info() {
    if [ -n "${ROS_DISTRO:-}" ]; then
        ROS_DISTRO_VAL="$ROS_DISTRO"
    else
        ROS_DISTRO_VAL="Not sourced"
    fi

    if command -v ros2 &>/dev/null; then
        ROS2_VERSION=$(ros2 --version 2>/dev/null | head -1 || echo "Unknown")
    else
        ROS2_VERSION="Not installed"
    fi
}

# Get GPU info (if available)
get_gpu_info() {
    if command -v nvidia-smi &>/dev/null; then
        GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || echo "None")
        GPU_MEMORY=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader 2>/dev/null | head -1 || echo "N/A")
        GPU_DRIVER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 || echo "N/A")
    else
        GPU_NAME="None detected"
        GPU_MEMORY="N/A"
        GPU_DRIVER="N/A"
    fi
}

# Get network info (for CloudLab)
get_network_info() {
    HOSTNAME_VAL=$(hostname -f 2>/dev/null || hostname)

    # Check if on CloudLab
    if echo "$HOSTNAME_VAL" | grep -q "cloudlab\|emulab"; then
        CLOUDLAB="Yes"
    else
        CLOUDLAB="Unknown"
    fi

    # Get primary IP
    if command -v ip &>/dev/null; then
        PRIMARY_IP=$(ip route get 1.1.1.1 2>/dev/null | grep -oP 'src \K\S+' || echo "Unknown")
    else
        PRIMARY_IP=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "Unknown")
    fi
}

# =============================================================================
# MAIN
# =============================================================================

log_info "Collecting hardware information..."

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Collect all info
log_section "CPU"
get_cpu_info
echo "  Model: $CPU_MODEL"
echo "  Physical cores: $CPU_PHYSICAL"
echo "  Logical cores: $CPU_THREADS"
echo "  Max frequency: ${CPU_FREQ_MHZ} MHz"

log_section "Memory"
get_memory_info
echo "  Total: ${MEM_TOTAL_GB} GB"

log_section "Disk"
get_disk_info
echo "  Device: $DISK_DEVICE"
echo "  Size: $DISK_SIZE"
echo "  Available: $DISK_AVAIL"

log_section "Operating System"
get_os_info
echo "  OS: $OS_NAME $OS_VERSION"
echo "  Kernel: $KERNEL_VERSION"
echo "  Architecture: $ARCH"

log_section "ROS 2"
get_ros_info
echo "  Distro: $ROS_DISTRO_VAL"
echo "  Version: $ROS2_VERSION"

log_section "GPU"
get_gpu_info
echo "  GPU: $GPU_NAME"
echo "  Memory: $GPU_MEMORY"

log_section "Network"
get_network_info
echo "  Hostname: $HOSTNAME_VAL"
echo "  CloudLab: $CLOUDLAB"

# =============================================================================
# GENERATE JSON
# =============================================================================

JSON_FILE="$OUTPUT_DIR/hardware_info.json"
cat > "$JSON_FILE" << EOF
{
    "timestamp": "$(date -Iseconds)",
    "hostname": "$HOSTNAME_VAL",
    "cloudlab": "$CLOUDLAB",
    "cpu": {
        "model": "$CPU_MODEL",
        "physical_cores": $CPU_PHYSICAL,
        "logical_cores": $CPU_THREADS,
        "max_freq_mhz": $CPU_FREQ_MHZ
    },
    "memory": {
        "total_gb": $MEM_TOTAL_GB
    },
    "disk": {
        "device": "$DISK_DEVICE",
        "size": "$DISK_SIZE",
        "available": "$DISK_AVAIL"
    },
    "os": {
        "name": "$OS_NAME",
        "version": "$OS_VERSION",
        "id": "$OS_ID",
        "kernel": "$KERNEL_VERSION",
        "arch": "$ARCH"
    },
    "ros2": {
        "distro": "$ROS_DISTRO_VAL",
        "version": "$ROS2_VERSION"
    },
    "gpu": {
        "name": "$GPU_NAME",
        "memory": "$GPU_MEMORY",
        "driver": "$GPU_DRIVER"
    }
}
EOF

log_info "Saved: $JSON_FILE"

# =============================================================================
# GENERATE LATEX TABLE
# =============================================================================

TEX_FILE="$OUTPUT_DIR/hardware_info.tex"
cat > "$TEX_FILE" << EOF
% Hardware specifications table
% Generated: $(date)

\\begin{table}[htbp]
\\centering
\\caption{Experiment Hardware Specifications}
\\label{tab:hardware}
\\begin{tabular}{ll}
\\toprule
\\textbf{Component} & \\textbf{Specification} \\\\
\\midrule
\\multicolumn{2}{l}{\\textit{CPU}} \\\\
\\quad Model & $CPU_MODEL \\\\
\\quad Physical Cores & $CPU_PHYSICAL \\\\
\\quad Logical Cores & $CPU_THREADS \\\\
\\quad Max Frequency & ${CPU_FREQ_MHZ} MHz \\\\
\\midrule
\\multicolumn{2}{l}{\\textit{Memory}} \\\\
\\quad Total RAM & ${MEM_TOTAL_GB} GB \\\\
\\midrule
\\multicolumn{2}{l}{\\textit{Storage}} \\\\
\\quad Disk Size & $DISK_SIZE \\\\
\\quad Available & $DISK_AVAIL \\\\
\\midrule
\\multicolumn{2}{l}{\\textit{Operating System}} \\\\
\\quad OS & $OS_NAME $OS_VERSION \\\\
\\quad Kernel & $KERNEL_VERSION \\\\
\\quad Architecture & $ARCH \\\\
\\midrule
\\multicolumn{2}{l}{\\textit{ROS 2}} \\\\
\\quad Distribution & $ROS_DISTRO_VAL \\\\
\\bottomrule
\\end{tabular}
\\end{table}
EOF

log_info "Saved: $TEX_FILE"

# =============================================================================
# GENERATE MARKDOWN
# =============================================================================

MD_FILE="$OUTPUT_DIR/hardware_info.md"
cat > "$MD_FILE" << EOF
# Hardware Specifications

**Generated:** $(date)
**Hostname:** $HOSTNAME_VAL

## CPU
| Property | Value |
|----------|-------|
| Model | $CPU_MODEL |
| Physical Cores | $CPU_PHYSICAL |
| Logical Cores | $CPU_THREADS |
| Max Frequency | ${CPU_FREQ_MHZ} MHz |

## Memory
| Property | Value |
|----------|-------|
| Total RAM | ${MEM_TOTAL_GB} GB |

## Storage
| Property | Value |
|----------|-------|
| Device | $DISK_DEVICE |
| Size | $DISK_SIZE |
| Available | $DISK_AVAIL |

## Operating System
| Property | Value |
|----------|-------|
| OS | $OS_NAME $OS_VERSION |
| Kernel | $KERNEL_VERSION |
| Architecture | $ARCH |

## ROS 2
| Property | Value |
|----------|-------|
| Distribution | $ROS_DISTRO_VAL |
| Version | $ROS2_VERSION |

## GPU
| Property | Value |
|----------|-------|
| Name | $GPU_NAME |
| Memory | $GPU_MEMORY |
| Driver | $GPU_DRIVER |

## CloudLab Status
| Property | Value |
|----------|-------|
| CloudLab Node | $CLOUDLAB |
| Primary IP | $PRIMARY_IP |
EOF

log_info "Saved: $MD_FILE"

# =============================================================================
# SUMMARY
# =============================================================================

echo ""
echo -e "${GREEN}=== Hardware Info Collection Complete ===${NC}"
echo "Output directory: $OUTPUT_DIR"
echo ""
echo "Files generated:"
echo "  - hardware_info.json (machine readable)"
echo "  - hardware_info.tex (LaTeX table)"
echo "  - hardware_info.md (Markdown summary)"
echo ""
echo "Include in reports with:"
echo "  \\input{hardware_info.tex}"

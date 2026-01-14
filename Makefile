# LDOS Manipulation Tracing Harness - Makefile
# Top-level targets for reproducible experiments

SHELL := /bin/bash
.ONESHELL:
.SHELLFLAGS := -euo pipefail -c

# Configuration
WS_ROOT := $(shell pwd)
NUM_TRIALS ?= 10
SMOKE_TRIALS ?= 1

# ROS setup
ROS_SETUP := source /opt/ros/jazzy/setup.bash 2>/dev/null || true; \
             source $(WS_ROOT)/install/setup.bash 2>/dev/null || true

.PHONY: all bootstrap setup build clean smoke_test \
        run_baseline run_cpu_load run_msg_load run_all \
        analyze_all report help \
        sweep_cpu sweep_msg find_breaking \
        analyze_paths validate_weights hardware_info \
        jupyter full_report check_deps

# Default target
all: help

#------------------------------------------------------------------------------
# Bootstrap (Fresh CloudLab Node)
#------------------------------------------------------------------------------

## Bootstrap fresh CloudLab/Ubuntu 24.04 node (installs ALL dependencies)
bootstrap:
	@echo "=== Bootstrapping CloudLab Node ==="
	@echo "This installs ROS 2, Gazebo, MoveIt, LTTng, and all dependencies."
	@echo "Runtime: ~15-25 minutes"
	@echo ""
	chmod +x scripts/*.sh
	./scripts/bootstrap_cloudlab.sh

## Check if all dependencies are installed
check_deps:
	@echo "=== Checking Dependencies ==="
	chmod +x scripts/*.sh
	./scripts/setup_workspace.sh --check

#------------------------------------------------------------------------------
# Setup and Build
#------------------------------------------------------------------------------

## Setup workspace (run after bootstrap or if deps already installed)
setup:
	@echo "=== Setting up LDOS harness ==="
	chmod +x scripts/*.sh
	chmod +x src/ldos_harness/scripts/*.py 2>/dev/null || true
	./scripts/setup_workspace.sh

## Build the ROS 2 workspace
build:
	@echo "=== Building workspace ==="
	$(ROS_SETUP)
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

## Clean build artifacts
clean:
	@echo "=== Cleaning workspace ==="
	rm -rf build install log
	rm -rf traces/*
	rm -rf results/*
	rm -rf analysis/output/*

## Deep clean (including traces and results)
distclean: clean
	rm -rf traces results analysis/output

#------------------------------------------------------------------------------
# Smoke Test (quick validation, <60 seconds)
#------------------------------------------------------------------------------

## Run smoke test to verify setup works
smoke_test:
	@echo "=== Running smoke test ==="
	@echo "This validates the full pipeline in <60 seconds"
	$(ROS_SETUP)
	./scripts/smoke_test.sh

#------------------------------------------------------------------------------
# Experiment Execution
#------------------------------------------------------------------------------

## Run baseline experiments (no load)
run_baseline:
	@echo "=== Running baseline experiments (N=$(NUM_TRIALS)) ==="
	$(ROS_SETUP)
	./scripts/run_experiment_suite.sh $(NUM_TRIALS) baseline

## Run CPU load experiments
run_cpu_load:
	@echo "=== Running CPU load experiments (N=$(NUM_TRIALS)) ==="
	$(ROS_SETUP)
	./scripts/run_experiment_suite.sh $(NUM_TRIALS) cpu_load

## Run message load experiments
run_msg_load:
	@echo "=== Running message load experiments (N=$(NUM_TRIALS)) ==="
	$(ROS_SETUP)
	./scripts/run_experiment_suite.sh $(NUM_TRIALS) msg_load

## Run all experiments (baseline + cpu_load + msg_load)
run_all:
	@echo "=== Running full experiment suite (N=$(NUM_TRIALS) per scenario) ==="
	$(ROS_SETUP)
	./scripts/run_experiment_suite.sh $(NUM_TRIALS) baseline,cpu_load,msg_load

#------------------------------------------------------------------------------
# Analysis
#------------------------------------------------------------------------------

## Analyze all collected traces and results
analyze_all:
	@echo "=== Analyzing all experiments ==="
	$(ROS_SETUP)
	./scripts/analyze_traces.sh all

## Generate report from analysis
report: analyze_all
	@echo "=== Generating report ==="
	@echo "Report skeleton: $(WS_ROOT)/docs/report_skeleton.md"
	@echo "Combined results: $(WS_ROOT)/analysis/output/combined_summary.csv"
	@if [ -f "$(WS_ROOT)/analysis/output/combined_summary.csv" ]; then \
		echo ""; \
		echo "=== Summary Statistics ==="; \
		python3 -c "import pandas as pd; \
			df = pd.read_csv('$(WS_ROOT)/analysis/output/combined_summary.csv'); \
			print(df.groupby('scenario')[['planning_latency_ms','execution_latency_ms','total_latency_ms']].describe().round(2))"; \
	fi

#------------------------------------------------------------------------------
# Acceptance Tests
#------------------------------------------------------------------------------

## Run acceptance tests to verify all components
acceptance_test:
	@echo "=== Running acceptance tests ==="
	@PASS=0; FAIL=0
	@# Test 1: ROS 2 packages available
	@echo -n "Test: ROS 2 ldos_harness package... "
	@$(ROS_SETUP) && ros2 pkg list 2>/dev/null | grep -q ldos_harness && echo "PASS" || { echo "FAIL"; FAIL=$$((FAIL+1)); }
	@# Test 2: LTTng available
	@echo -n "Test: LTTng command available... "
	@command -v lttng >/dev/null 2>&1 && echo "PASS" || { echo "FAIL"; FAIL=$$((FAIL+1)); }
	@# Test 3: stress-ng available
	@echo -n "Test: stress-ng available... "
	@command -v stress-ng >/dev/null 2>&1 && echo "PASS" || { echo "FAIL (optional)"; }
	@# Test 4: Python dependencies
	@echo -n "Test: Python babeltrace2... "
	@python3 -c "import bt2" 2>/dev/null && echo "PASS" || { echo "FAIL"; FAIL=$$((FAIL+1)); }
	@echo -n "Test: Python pandas... "
	@python3 -c "import pandas" 2>/dev/null && echo "PASS" || { echo "FAIL"; FAIL=$$((FAIL+1)); }
	@# Test 5: Scripts executable
	@echo -n "Test: Scripts executable... "
	@[ -x scripts/run_experiment_suite.sh ] && echo "PASS" || { echo "FAIL"; FAIL=$$((FAIL+1)); }
	@# Summary
	@echo ""
	@echo "Acceptance tests complete. Failures: $$FAIL"
	@[ $$FAIL -eq 0 ] || exit 1

#------------------------------------------------------------------------------
# Parameter Sweeps
#------------------------------------------------------------------------------

## Run CPU load parameter sweep
sweep_cpu:
	@echo "=== Running CPU load parameter sweep ==="
	$(ROS_SETUP)
	./scripts/parameter_sweep.sh cpu_load_percent "0 25 50 75 90" $(NUM_TRIALS)

## Run message load parameter sweep
sweep_msg:
	@echo "=== Running message load parameter sweep ==="
	$(ROS_SETUP)
	./scripts/parameter_sweep.sh num_publishers "1 5 10 20 50" $(NUM_TRIALS)

## Find breaking point for CPU load
find_breaking:
	@echo "=== Finding CPU load breaking point ==="
	$(ROS_SETUP)
	python3 scripts/find_breaking_point.py \
		--param cpu_load_percent \
		--low 50 --high 100 \
		--threshold 0.9 \
		--trials 20 \
		--output analysis/output

#------------------------------------------------------------------------------
# Advanced Analysis
#------------------------------------------------------------------------------

## Run end-to-end path analysis
analyze_paths:
	@echo "=== Running path analysis ==="
	python3 analysis/e2e_path_analyzer.py \
		--trace-dir traces/ \
		--output analysis/output/paths

## Run sweep analysis on results
analyze_sweep:
	@echo "=== Running sweep analysis ==="
	python3 analysis/sweep_analysis.py \
		--results results/*/ \
		--output analysis/output

## Validate objective weights against data
validate_weights:
	@echo "=== Validating objective weights ==="
	python3 analysis/validate_weights.py \
		--objectives configs/objectives.yaml \
		--baseline results/baseline \
		--load results/cpu_load \
		--output analysis/output

## Collect hardware information
hardware_info:
	@echo "=== Collecting hardware info ==="
	./scripts/collect_hardware_info.sh analysis/output

#------------------------------------------------------------------------------
# Notebooks and Reporting
#------------------------------------------------------------------------------

## Launch Jupyter notebook server
jupyter:
	@echo "=== Starting Jupyter notebook ==="
	cd notebooks && jupyter notebook

## Generate full report (all analysis + LaTeX compile)
full_report: analyze_all analyze_paths validate_weights hardware_info
	@echo "=== Generating full report ==="
	@echo "Analysis outputs: analysis/output/"
	@echo "LaTeX template: docs/report_template.tex"
	@if command -v pdflatex >/dev/null 2>&1; then \
		cd docs && pdflatex -interaction=nonstopmode report_template.tex; \
		echo "Report generated: docs/report_template.pdf"; \
	else \
		echo "pdflatex not found. Install with: apt install texlive-latex-base"; \
	fi

#------------------------------------------------------------------------------
# Help
#------------------------------------------------------------------------------

## Show this help
help:
	@echo "LDOS Manipulation Tracing Harness"
	@echo ""
	@echo "Usage: make <target> [NUM_TRIALS=N]"
	@echo ""
	@echo "First-Time Setup (Fresh CloudLab Node):"
	@echo "  bootstrap      - Install ALL dependencies (~15-25 min)"
	@echo "  check_deps     - Verify all dependencies installed"
	@echo ""
	@echo "Setup & Build:"
	@echo "  setup          - Setup workspace (after bootstrap)"
	@echo "  build          - Build ROS 2 packages"
	@echo "  clean          - Remove build artifacts"
	@echo ""
	@echo "Quick Validation:"
	@echo "  smoke_test     - Quick validation (<60 seconds)"
	@echo "  acceptance_test - Verify all components installed"
	@echo ""
	@echo "Experiments:"
	@echo "  run_baseline   - Run baseline experiments (N=$(NUM_TRIALS))"
	@echo "  run_cpu_load   - Run CPU load experiments (N=$(NUM_TRIALS))"
	@echo "  run_msg_load   - Run message load experiments (N=$(NUM_TRIALS))"
	@echo "  run_all        - Run all scenarios (N=$(NUM_TRIALS) each)"
	@echo ""
	@echo "Parameter Sweeps:"
	@echo "  sweep_cpu      - Sweep CPU load (0-90%)"
	@echo "  sweep_msg      - Sweep message publishers (1-50)"
	@echo "  find_breaking  - Find system breaking point"
	@echo ""
	@echo "Analysis:"
	@echo "  analyze_all    - Process traces and aggregate results"
	@echo "  analyze_paths  - End-to-end path analysis"
	@echo "  analyze_sweep  - Analyze parameter sweep results"
	@echo "  validate_weights - Validate objective weights"
	@echo "  report         - Generate summary report"
	@echo ""
	@echo "Reporting:"
	@echo "  hardware_info  - Collect system specifications"
	@echo "  jupyter        - Launch Jupyter notebook server"
	@echo "  full_report    - Generate complete PDF report"
	@echo ""
	@echo "Quick Start (Fresh CloudLab):"
	@echo "  make bootstrap           # Install everything (~20 min)"
	@echo "  # Log out and back in    # For tracing group"
	@echo "  make smoke_test          # Verify it works"
	@echo "  make run_all             # Run experiments"
	@echo ""
	@echo "Examples:"
	@echo "  make bootstrap              # First-time setup"
	@echo "  make smoke_test             # Quick validation"
	@echo "  make run_all NUM_TRIALS=30  # Full experiment suite"
	@echo "  make analyze_all report     # Analyze and report"

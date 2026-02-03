"""
sim_only.launch.py - Launch ONLY Gazebo Harmonic simulation

This launch file starts ONLY the Gazebo physics simulation:
1. Gazebo Harmonic (gz-sim8) with empty world
2. (Optional) GUI

This is used for cpuset isolation experiments where Gazebo runs on
unlimited CPUs while the ROS stack runs on limited CPUs.

The ROS stack (robot spawn, controllers, MoveIt) should be launched
separately using ros_stack.launch.py.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run Gazebo in headless mode (no GUI)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="",
            description="World file to load (empty for default)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_plugin_path",
            default_value="/opt/ros/jazzy/lib",
            description="Path to Gazebo system plugins (gz_ros2_control)",
        )
    )

    # Get configurations
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    gz_plugin_path = LaunchConfiguration("gz_plugin_path")

    # Set Gazebo plugin path environment variable
    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=gz_plugin_path,
    )

    # Environment variables for headless operation
    set_qt_platform = SetEnvironmentVariable(
        name="QT_QPA_PLATFORM",
        value="offscreen",
        condition=IfCondition(headless),
    )

    # Unset DISPLAY to prevent X11 connection attempts in headless mode
    unset_display = SetEnvironmentVariable(
        name="DISPLAY",
        value="",
        condition=IfCondition(headless),
    )

    # Find package paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Log mode
    log_mode = LogInfo(
        msg=["Gazebo-only launch (for cpuset isolation). Headless: ", headless],
    )

    # Gazebo Harmonic launch
    # When headless=true: use -s (server only) + --headless-rendering
    # When headless=false: just -r (run immediately with GUI)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression([
                "'-s --headless-rendering -r empty.sdf' if '",
                headless,
                "' == 'true' else '-r empty.sdf'"
            ]),
            "on_exit_shutdown": "true",
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            # Set environment variables before Gazebo starts
            set_qt_platform,
            unset_display,
            set_gz_plugin_path,
            log_mode,
            # Start Gazebo only
            gz_sim,
        ]
    )

"""
full_stack.launch.py - Complete stack: Gazebo + Panda + Controllers + MoveIt

This is the main entry point for the LDOS tracing harness.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run Gazebo without GUI",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz visualization",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_delay",
            default_value="12.0",
            description="Seconds to wait before starting MoveIt (allow sim to stabilize)",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    moveit_delay = LaunchConfiguration("moveit_delay")

    pkg_ldos_harness = get_package_share_directory("ldos_harness")

    # Launch simulation (Gazebo + Panda + Controllers)
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ldos_harness, "launch", "sim_bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "headless": headless,
        }.items(),
    )

    # Launch MoveIt (delayed to allow controllers to start)
    moveit_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ldos_harness, "launch", "moveit_bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
        }.items(),
    )

    delayed_moveit = TimerAction(
        period=12.0,  # Wait for Gazebo + controllers
        actions=[moveit_bringup],
    )

    return LaunchDescription(
        declared_arguments
        + [
            sim_bringup,
            delayed_moveit,
        ]
    )

"""
sim_bringup.launch.py - Launch Gazebo Harmonic with Panda robot and ros2_control

This launch file:
1. Starts Gazebo Harmonic (gz-sim8) with empty world
2. Spawns Panda robot URDF with gz_ros2_control plugin
3. Loads and activates controllers (joint_state_broadcaster + joint_trajectory_controller)
4. Starts robot_state_publisher for TF
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock",
        )
    )
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_delay",
            default_value="3.0",
            description="Delay before spawning robot (seconds)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_delay",
            default_value="5.0",
            description="Delay before loading controllers (seconds)",
        )
    )

    # Get configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    gz_plugin_path = LaunchConfiguration("gz_plugin_path")
    spawn_delay = LaunchConfiguration("spawn_delay")
    controller_delay = LaunchConfiguration("controller_delay")

    # Set Gazebo plugin path environment variable
    # This ensures gz_ros2_control plugin can be found
    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=gz_plugin_path,
    )

    # Find package paths
    pkg_ldos_harness = get_package_share_directory("ldos_harness")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Panda URDF with gz_ros2_control plugin
    # We generate URDF with xacro, including the Gazebo control plugin
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [pkg_ldos_harness, "config", "panda_gz.urdf.xacro"]
                ),
                " ",
                "use_sim_time:=",
                use_sim_time,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Gazebo Harmonic launch
    gz_args = ["-r"]  # -r for running immediately
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": " ".join(gz_args) + " empty.sdf",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "panda",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        output="screen",
    )

    # Load controllers after robot is spawned
    # Joint State Broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller", "--set-state", "active",
            "joint_state_broadcaster"
        ],
        output="screen",
    )

    # Panda Arm Controller (Joint Trajectory Controller)
    load_panda_arm_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller", "--set-state", "active",
            "panda_arm_controller"
        ],
        output="screen",
    )

    # Panda Hand Controller
    load_panda_hand_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller", "--set-state", "active",
            "panda_hand_controller"
        ],
        output="screen",
    )

    # Bridge for clock (sim time)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Delay controller loading to ensure Gazebo and robot are ready
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[
            load_joint_state_broadcaster,
        ],
    )

    delayed_arm_controller = TimerAction(
        period=7.0,
        actions=[
            load_panda_arm_controller,
        ],
    )

    delayed_hand_controller = TimerAction(
        period=8.0,
        actions=[
            load_panda_hand_controller,
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            # Set plugin path before starting Gazebo
            set_gz_plugin_path,
            robot_state_publisher,
            gz_sim,
            clock_bridge,
            TimerAction(period=3.0, actions=[spawn_robot]),
            delayed_controllers,
            delayed_arm_controller,
            delayed_hand_controller,
        ]
    )

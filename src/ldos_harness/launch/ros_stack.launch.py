"""
ros_stack.launch.py - Launch entire ROS stack (everything except Gazebo)

This launch file starts ALL ROS components:
1. robot_state_publisher (TF)
2. ros_gz_bridge (clock bridge from Gazebo)
3. Robot spawn into Gazebo (ros_gz_sim create)
4. Controllers (joint_state_broadcaster, panda_arm_controller, panda_hand_controller)
5. MoveIt move_group

This is used for cpuset isolation experiments where the ROS stack runs on
limited CPUs while Gazebo runs on unlimited CPUs.

Gazebo should be launched separately using sim_only.launch.py.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    spawn_delay = LaunchConfiguration("spawn_delay")
    controller_delay = LaunchConfiguration("controller_delay")
    moveit_delay = LaunchConfiguration("moveit_delay")

    pkg_ldos_harness = get_package_share_directory("ldos_harness")

    # Evaluate configurations
    use_sim_time_bool = use_sim_time.perform(context).lower() == "true"
    spawn_delay_val = float(spawn_delay.perform(context))
    controller_delay_val = float(controller_delay.perform(context))
    moveit_delay_val = float(moveit_delay.perform(context))

    # Robot description
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

    # SRDF for MoveIt
    robot_description_semantic_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [pkg_ldos_harness, "config", "panda.srdf.xacro"]
                ),
            ]
        ),
        value_type=str,
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # -------------------------------------------------------------------------
    # 1. Robot State Publisher (TF)
    # -------------------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time_bool}],
    )

    # -------------------------------------------------------------------------
    # 2. Clock Bridge (Gazebo → ROS)
    # -------------------------------------------------------------------------
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Delay clock bridge slightly to ensure Gazebo is ready
    delayed_clock_bridge = TimerAction(
        period=2.0,
        actions=[clock_bridge],
    )

    # -------------------------------------------------------------------------
    # 3. Spawn Robot in Gazebo
    # -------------------------------------------------------------------------
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

    delayed_spawn = TimerAction(
        period=spawn_delay_val,
        actions=[spawn_robot],
    )

    # -------------------------------------------------------------------------
    # 4. Load Controllers
    # -------------------------------------------------------------------------
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller", "--set-state", "active",
            "joint_state_broadcaster"
        ],
        output="screen",
    )

    load_panda_arm_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller", "--set-state", "active",
            "panda_arm_controller"
        ],
        output="screen",
    )

    load_panda_hand_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller", "--set-state", "active",
            "panda_hand_controller"
        ],
        output="screen",
    )

    delayed_controllers = [
        TimerAction(
            period=controller_delay_val,
            actions=[load_joint_state_broadcaster],
        ),
        TimerAction(
            period=controller_delay_val + 2.0,
            actions=[load_panda_arm_controller],
        ),
        TimerAction(
            period=controller_delay_val + 3.0,
            actions=[load_panda_hand_controller],
        ),
    ]

    # -------------------------------------------------------------------------
    # 5. MoveIt move_group
    # -------------------------------------------------------------------------
    moveit_config_file = os.path.join(pkg_ldos_harness, "config", "moveit.yaml")

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            ParameterFile(moveit_config_file, allow_substs=False),
            {"use_sim_time": use_sim_time_bool},
        ],
    )

    delayed_moveit = TimerAction(
        period=moveit_delay_val,
        actions=[move_group_node],
    )

    # -------------------------------------------------------------------------
    # Optional: RViz
    # -------------------------------------------------------------------------
    rviz_config_file = PathJoinSubstitution(
        [pkg_ldos_harness, "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ParameterFile(moveit_config_file, allow_substs=False),
            {"use_sim_time": use_sim_time_bool},
        ],
        condition=IfCondition(use_rviz),
    )

    delayed_rviz = TimerAction(
        period=moveit_delay_val + 5.0,
        actions=[rviz_node],
    )

    return [
        robot_state_publisher,
        delayed_clock_bridge,
        delayed_spawn,
    ] + delayed_controllers + [
        delayed_moveit,
        delayed_rviz,
    ]


def generate_launch_description():
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
            "use_rviz",
            default_value="false",
            description="Launch RViz for visualization",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_delay",
            default_value="3.0",
            description="Delay before spawning robot (seconds) - wait for Gazebo",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_delay",
            default_value="8.0",
            description="Delay before loading controllers (seconds) - wait for spawn",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_delay",
            default_value="15.0",
            description="Delay before starting MoveIt (seconds) - wait for controllers",
        )
    )

    log_info = LogInfo(
        msg=["ROS Stack launch (for cpuset isolation). Includes: RSP, bridge, spawn, controllers, MoveIt"],
    )

    return LaunchDescription(
        declared_arguments + [
            log_info,
            OpaqueFunction(function=launch_setup),
        ]
    )

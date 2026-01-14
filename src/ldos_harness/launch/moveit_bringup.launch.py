"""
moveit_bringup.launch.py - Launch MoveIt 2 for Panda robot

This launch file starts:
1. MoveIt move_group node
2. Optional RViz with MoveIt plugin
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    """Load a yaml file from a package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except Exception as e:
        return None


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    pkg_ldos_harness = get_package_share_directory("ldos_harness")

    # Robot description (must match sim_bringup)
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [pkg_ldos_harness, "config", "panda_gz.urdf.xacro"]
                ),
                " ",
                "use_sim_time:=true",
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

    # Kinematics configuration
    kinematics_yaml = load_yaml("ldos_harness", "config/kinematics.yaml")
    if kinematics_yaml is None:
        kinematics_yaml = {
            "panda_arm": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05,
            }
        }

    # Planning configuration - MoveIt 2 Jazzy structure
    # Parameters must be under planning_pipelines structure
    ompl_planning_yaml_raw = load_yaml("ldos_harness", "config/ompl_planning.yaml")
    if ompl_planning_yaml_raw is None:
        ompl_planning_yaml_raw = {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }

    # Wrap in proper MoveIt 2 planning pipeline structure
    ompl_planning_yaml = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning_yaml_raw,
    }

    # Joint limits
    joint_limits_yaml = {
        "robot_description_planning": {
            "joint_limits": {
                "panda_joint1": {"has_velocity_limits": True, "max_velocity": 2.175},
                "panda_joint2": {"has_velocity_limits": True, "max_velocity": 2.175},
                "panda_joint3": {"has_velocity_limits": True, "max_velocity": 2.175},
                "panda_joint4": {"has_velocity_limits": True, "max_velocity": 2.175},
                "panda_joint5": {"has_velocity_limits": True, "max_velocity": 2.610},
                "panda_joint6": {"has_velocity_limits": True, "max_velocity": 2.610},
                "panda_joint7": {"has_velocity_limits": True, "max_velocity": 2.610},
            }
        }
    }

    # Trajectory execution
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Controller configuration for MoveIt
    moveit_controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["panda_arm_controller", "panda_hand_controller"],
        },
        "panda_arm_controller": {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "joints": [
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ],
        },
        "panda_hand_controller": {
            "type": "GripperCommand",
            "action_ns": "gripper_cmd",
            "joints": ["panda_finger_joint1"],
        },
    }

    # Planning scene monitor
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": True},
        ],
    )

    # RViz (optional)
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
            kinematics_yaml,
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_rviz),
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
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
            "use_rviz",
            default_value="false",
            description="Launch RViz for visualization",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
